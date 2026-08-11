/**
 * @file LiDAR.cpp
 * @brief LiDAR激光雷达数据接收、解析、写入LAZ文件的核心实现
 * @details 包含三个核心类：
 *          1. LiDARReceiver: 负责UDP接收LiDAR原始数据包
 *          2. LiDARParser: 负责解析原始数据包为点云数据
 *          3. LiDARWriter: 负责将点云数据写入LAZ压缩文件
 */
#include "LiDAR.h"
#include "Logger.h"
#include "config_manager.h"
#include "RayTracing.h"
#include "utils.h"

// 系统头文件
#include <unistd.h>         // POSIX系统调用
#include <cstring>          // 内存操作函数
#include <thread>           // 线程相关
#include <chrono>           // 时间相关
#include <sched.h>          // 调度策略/实时优先级
#include <errno.h>          // 错误码定义
#include <limits>           // 数值极限
#include <ctime>            // 时间处理

// 匿名命名空间 - 仅当前编译单元可见的常量和变量
namespace {
    // // 统计信息（暂时注释）
    // size_t invalidPackets = 0;    // 无效数据包计数
    // size_t discardedPackets = 0;  // 丢弃数据包计数

    // 数据包头部校验常量 - LiDAR协议规定的固定头部标识
    constexpr uint16_t EXPECTED_HEAD1 = 0xA35C;  // 头部标识1
    constexpr uint16_t EXPECTED_HEAD2 = 0xC53A;  // 头部标识2
    constexpr uint16_t EXPECTED_HEAD3 = 0xA35C;  // 头部标识3
    constexpr uint16_t EXPECTED_HEAD4 = 0xC53A;  // 头部标识4
    
    // GPS时间校验常量
    constexpr uint16_t MIN_GPS_WEEK = 2380;              // 最小有效GPS周数
    constexpr uint32_t MAX_MILLSECONDS_IN_WEEK = 604800000; // GPS一周最大毫秒数 (7*24*60*60*1000)
}

/**
 * @brief LiDARReceiver构造函数
 * @param outQ 输出队列，用于将接收到的完整数据包传递给解析器
 * @param running 原子布尔变量，控制线程运行状态
 * @details 初始化UDP接收参数，创建UDP套接字，配置接收缓冲区
 */
LiDARReceiver::LiDARReceiver(moodycamel::ConcurrentQueue<std::shared_ptr<PacketData>>& outQ, 
                           const std::atomic<bool>& running)
    : gRunning(running),  // 线程运行状态标志（引用）
      recvBuffer(BUFFER_SIZE),  // 接收缓冲区，大小由BUFFER_SIZE宏定义
      outputQueue(outQ)  // 输出队列（引用）
{
    // 从配置管理器读取LiDAR接收配置
    ifaceIp = ConfigManager::getInstance().get<std::string>("LIDAR.RECEIVER.IP");    // 绑定的网卡IP
    udpPort = ConfigManager::getInstance().get<int>("LIDAR.RECEIVER.PORT");          // 监听的UDP端口
    
    // 创建并初始化UDP套接字
    sockfd = createUDPSocket(udpPort, ifaceIp);
    
    // 记录启动日志
    Logger::lidarReceiverLogger()->info("UDP服务器启动，监听 {}:{}", ifaceIp, udpPort);
    
    // 预分配数据包缓冲区空间，避免频繁内存分配
    currentPacketData.reserve(sizeof(PacketData));
}

/**
 * @brief LiDARReceiver析构函数
 * @details 关闭UDP套接字，记录退出日志
 */
#ifdef _WIN32
    #define CLOSE_SOCKET(s) ::closesocket(s)
#else
    #define CLOSE_SOCKET(s) ::close(s)
#endif

LiDARReceiver::~LiDARReceiver() {
    if (sockfd != (socket_t)-1) CLOSE_SOCKET(sockfd);
    Logger::lidarReceiverLogger()->info("UDP接收线程退出，停止监听 {}:{}", ifaceIp, udpPort);
}

void LiDARReceiver::setupRealTimePriority() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    Logger::lidarReceiverLogger()->info("设置LiDAR线程优先级成功 (THREAD_PRIORITY_HIGHEST)");
#else
    if (geteuid() == 0) {
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 5;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            Logger::lidarReceiverLogger()->warn("设置LiDAR线程实时优先级失败: {}", strerror(errno));
        } else {
            Logger::lidarReceiverLogger()->info("设置LiDAR线程实时优先级成功（SCHED_FIFO, 优先级: {})", param.sched_priority);
        }
    } else {
        Logger::lidarReceiverLogger()->warn("非root运行，无法设置LiDAR线程实时优先级!");
    }
#endif
}

socket_t LiDARReceiver::createUDPSocket(int udpPort, const std::string& ifaceIP) {
    socket_t sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == (socket_t)-1) {
        throw AppError(-3, "创建UDP套接字失败");
    }

    int opt = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt)) < 0) {
        CLOSE_SOCKET(sockfd);
        throw AppError(-3, "设置地址重用失败");
    }

    struct sockaddr_in serverAddr;
    std::memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    
    if (inet_pton(AF_INET, ifaceIP.c_str(), &serverAddr.sin_addr) <= 0) {
        CLOSE_SOCKET(sockfd);
        throw AppError(-3, "无效的网卡IP地址：" + ifaceIP);
    }
    serverAddr.sin_port = htons(udpPort);

    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        CLOSE_SOCKET(sockfd);
        throw AppError(-3, "绑定到网卡 " + ifaceIp + ":" + std::to_string(udpPort) + " 失败");
    }

    int optVal = 1024 * 1024 * 20;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&optVal), sizeof(optVal)) < 0) {
        Logger::lidarReceiverLogger()->warn("设置UDP接收缓冲区失败");
    }

#ifdef _WIN32
    DWORD timeout_ms = 1000;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout_ms), sizeof(timeout_ms));
#else
    struct timeval optTimeout;
    optTimeout.tv_sec = 1;
    optTimeout.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&optTimeout), sizeof(optTimeout));
#endif

    return sockfd;
}

/**
 * @brief 校验数据包的合法性
 * @param packet 待校验的数据包指针
 * @return 合法返回true，否则返回false
 * @details 校验数据包头部标识和GPS时间的有效性
 */
bool LiDARReceiver::checkPacket(const PacketData* packet) const {
    // 空指针检查
    if (!packet) return false;
    
    // 头部标识校验
    if (packet->head1 != EXPECTED_HEAD1 || packet->head2 != EXPECTED_HEAD2 ||
        packet->head3 != EXPECTED_HEAD3 || packet->head4 != EXPECTED_HEAD4) {
        return false;
    }
    
    // GPS周数校验
    if (packet->gpsWeek < MIN_GPS_WEEK) return false;
    
    // 周内毫秒数校验
    if (packet->msecInWeek > MAX_MILLSECONDS_IN_WEEK) return false;
    
    return true;
}

/**
 * @brief 接收并处理UDP数据包
 * @return 成功解析出完整数据包返回shared_ptr，否则返回nullptr
 * @details 处理数据包分片、拼接和校验，支持不完整数据包的缓冲处理
 */
std::shared_ptr<PacketData> LiDARReceiver::produce() {
    // 接收UDP数据
    ssize_t recvLen = recvfrom(sockfd,                  // 套接字描述符
                             recvBuffer.data(),        // 接收缓冲区
                             recvBuffer.size(),        // 缓冲区大小
                             0,                        // 标志位
                             (struct sockaddr*)&clientAddr,  // 客户端地址（输出）
                             &clientAddrLen);          // 地址长度（输入/输出）

    // 接收错误处理
    if (recvLen < 0) {
        // 非中断错误（EINTR是正常的信号中断）
        if (errno != EINTR) {
            Logger::lidarReceiverLogger()->warn("UDP接收失败：{}", strerror(errno));
            // 短暂休眠避免CPU空转
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return nullptr;
    } 
    // 空数据包处理
    else if (recvLen == 0) {
        Logger::lidarReceiverLogger()->warn("收到空数据包，已忽略");
        return nullptr;
    }

    // 检查是否包含有效数据包头部
    bool hasHeader = false;
    if (recvLen >= static_cast<ssize_t>(sizeof(PacketDataHead))) {
        // 转换缓冲区数据为数据包头部结构体
        PacketDataHead* header = reinterpret_cast<PacketDataHead*>(recvBuffer.data());
        // 校验头部标识
        hasHeader = (header->head1 == EXPECTED_HEAD1 &&
                    header->head2 == EXPECTED_HEAD2 &&
                    header->head3 == EXPECTED_HEAD3 &&
                    header->head4 == EXPECTED_HEAD4);
    }

    // 收到包含有效头部的数据包
    if (hasHeader) {
        std::shared_ptr<PacketData> completedPacket = nullptr;

        // 如果当前有未处理完的数据包（上一次的分片）
        if (!currentPacketData.empty()) {
            // 检查上一个数据包是否完整
            if (currentPacketData.size() == sizeof(PacketData)) {
                // 转换为PacketData结构体
                PacketData* packet = reinterpret_cast<PacketData*>(currentPacketData.data());
                
                // 校验数据包合法性
                if (checkPacket(packet)) {
                    // 原子计数：完整数据包数
                    full_packets.fetch_add(1, std::memory_order_relaxed);
                    // 创建共享指针并拷贝数据
                    completedPacket = std::make_shared<PacketData>();
                    std::memcpy(completedPacket.get(), packet, sizeof(PacketData));
                } else {
                    // 原子计数：无效数据包数
                    partial_packets.fetch_add(1, std::memory_order_relaxed);
                    Logger::lidarReceiverLogger()->warn("数据包验证失败，已丢弃！");
                }
            } else {
                // 原子计数：不完整数据包数
                partial_packets.fetch_add(1, std::memory_order_relaxed);
                Logger::lidarReceiverLogger()->warn("数据包不完整({}字节/{}字节)，已丢弃！", 
                                                  currentPacketData.size(), sizeof(PacketData));
            }
        }
        
        // 原子计数：总接收数据包数
        total_packets.fetch_add(1, std::memory_order_relaxed);
        
        // 清空缓冲区，准备接收新数据包
        currentPacketData.clear();
        // 将新接收的数据加入缓冲区
        currentPacketData.insert(currentPacketData.end(), recvBuffer.begin(), recvBuffer.begin() + recvLen);
        
        return completedPacket;
    } 
    // 没有有效头部，但当前有未完成的数据包（分片）
    else if (!currentPacketData.empty()) {
        // 拼接分片数据
        currentPacketData.insert(currentPacketData.end(), recvBuffer.begin(), recvBuffer.begin() + recvLen);

        // 防止缓冲区溢出：超过2倍数据包大小则丢弃
        if (currentPacketData.size() > 2 * sizeof(PacketData)) {
            Logger::lidarReceiverLogger()->warn("数据包过大 ({}/{}字节)，已强制丢弃", 
                                              currentPacketData.size(), sizeof(PacketData));
            currentPacketData.clear();
        }
    } 
    // 无效数据分片，直接丢弃
    else {
        Logger::lidarReceiverLogger()->warn("收到无效数据分片，已丢弃");
    }

    return nullptr;
}

/**
 * @brief LiDAR接收线程主循环
 * @details 设置实时优先级，持续接收数据包并将有效数据包放入输出队列
 */
void LiDARReceiver::run() {
    // 设置线程实时优先级
    setupRealTimePriority();
    
    // 主循环：受gRunning原子变量控制
    while (gRunning) {
        // 接收并处理数据包
        auto packet = produce();
        
        // 成功获取到完整数据包
        if (packet) {
            // 入队到输出队列，供解析器处理
            outputQueue.enqueue(packet);
            // 调试日志：输出GPS时间信息
            Logger::lidarReceiverLogger()->debug("数据包完整， gps周{} | 周内秒{}", 
                                              packet->gpsWeek, packet->msecInWeek / 1000);
        }
    }
}

/**
 * @brief LiDARParser构造函数
 * @param inQ 输入队列，接收LiDARReceiver的完整数据包
 * @param outQ 输出队列，输出解析后的点云数据
 * @param running 原子布尔变量，控制线程运行状态
 */
LiDARParser::LiDARParser(moodycamel::ConcurrentQueue<std::shared_ptr<PacketData>>& inQ,
                       moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& outQ,
                       const std::atomic<bool>& running)
    : inputQueue(inQ),    // 输入队列（引用）
      outputQueue(outQ),  // 输出队列（引用）
      gRunning(running)   // 线程运行状态（引用）
{}

/**
 * @brief LiDARParser析构函数
 * @details 记录解析器关闭日志
 */
LiDARParser::~LiDARParser(){
    Logger::lidarReceiverLogger()->info("LiDAR数据解析关闭！");
}

/**
 * @brief 解析单个LiDAR数据包为点云数据
 * @param packet 待解析的数据包共享指针
 * @details 解码脉冲数据为三维点云，处理时间戳，确保点云时间单调递增
 */
void LiDARParser::consume(std::shared_ptr<PacketData> packet) {
    // 记录处理开始时间（性能统计）
    auto process_start = std::chrono::high_resolution_clock::now();
    
    // 预分配点云和时间数组空间（减少内存分配次数）
    std::vector<Eigen::Vector3d> points;  // 三维点坐标
    std::vector<double> times;            // 每个点的时间戳
    points.reserve(64 * 64 * 200);        // 预分配足够空间
    times.reserve(64 * 64 * 200);

    // 遍历所有脉冲数据（200个脉冲/包）
    for (size_t pulseIdx = 0; pulseIdx < 200; pulseIdx++) {
        // 解码单个脉冲数据为点云
        PointCloudData pc = VRT::processDecodeData(packet->pulseData[pulseIdx],  // 脉冲数据
                                                   packet->gpsWeek,             // GPS周数
                                                   packet->msecInWeek,          // 周内毫秒数
                                                   packet->timeRes);            // 时间分辨率

        // 合并点云和时间数据
        points.insert(points.end(), pc.points.begin(), pc.points.end());
        times.insert(times.end(), pc.times.begin(), pc.times.end());
    }

    // 将GPS时间转换为Linux时间戳（毫秒级）
    int64_t timeStamp = TimeUtils::gpsTimeToLinuxTimestamp(packet->gpsWeek, 
                                                          packet->msecInWeek, 
                                                          TimeUtils::TimeUnit::Milliseconds);
    
    // 创建点云数据共享指针（移动语义避免拷贝）
    auto cloud = std::make_shared<PointCloudData>(timeStamp, 
                                                 std::move(points), 
                                                 std::move(times));

    // 检查时间戳是否单调递增（防止乱序）
    if (lastLiDAREnqueuedTime_ >= cloud->timeStamp){
        // 原子计数：非单调递增次数
        lidar_non_mono.fetch_add(1, std::memory_order_relaxed);
        
        // 记录事件信息
        EventBasic ev;
        ev.wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        ev.type = EventType::LIDAR_NON_MONO;       // 事件类型：LiDAR时间非单调
        ev.t0_ms = lastLiDAREnqueuedTime_;         // 上一个时间戳
        ev.t1_ms = cloud->timeStamp;               // 当前时间戳
        ev.t_ms = cloud->timeStamp - lastLiDAREnqueuedTime_;  // 时间差
        
        // 加入事件队列
        lidar_events.push(ev);

        // 记录警告日志
        Logger::lidarParserLogger()->warn("LiDAR数据入队时间不严格递增，当前队列最后元素时间：{} | 待入队元素时间：{} 放弃入队！", 
                                       lastLiDAREnqueuedTime_, cloud->timeStamp);
    } else {
        // 时间戳合法，入队到输出队列
        outputQueue.enqueue(cloud);
        // 更新最后入队时间戳
        lastLiDAREnqueuedTime_ = cloud->timeStamp;
        // 统计每个数据包的点数
        points_per_packet.add(double(cloud->points.size()));
        // 调试日志
        Logger::lidarParserLogger()->debug("LiDAR点云数据解析完成，点数：{}", cloud->points.size());

        // 调试功能：保存解析结果为PCD文件
        std::string pcd_pth = debug_save_pth + "/" + std::to_string(cloud->timeStamp) + ".pcd";
        Save::writeVector3dToPCDManual(cloud->points, pcd_pth);
    }
    
    // 计算并记录处理耗时
    auto process_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(process_end - process_start).count();
    Logger::lidarParserLogger()->debug("LiDAR数据包处理耗时：{} ms", duration);
    
    // TODO 0905 test4 - 测试代码（待移除）
    // outputQueue.enqueue(cloud);
    // lastLiDAREnqueuedTime_ = cloud->timeStamp;
    // points_per_packet.add(double(cloud->points.size()));
    // Logger::lidarParserLogger()->debug("LiDAR点云数据解析完成，点数：{}", cloud->points.size());
}

/**
 * @brief LiDAR解析线程主循环
 * @details 从输入队列获取数据包，解析为点云数据，支持队列为空时的休眠机制
 */
void LiDARParser::run() {
    // 主循环：受gRunning原子变量控制
    while (gRunning) {
        std::shared_ptr<PacketData> packet;
        
        // 尝试从队列取出数据包（非阻塞）
        if (inputQueue.try_dequeue(packet)) {
            // 性能统计：解析耗时
            Telemetry::ScopedTimerMs timer(parse_time_ms);
            // 解析数据包
            consume(packet);
            // 原子计数：总处理数据包数
            total_packets.fetch_add(1, std::memory_order_relaxed);
        } else {
            // 队列为空时短暂休眠，避免CPU空转
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

/**
 * @brief LiDARWriter构造函数
 * @param iQ 输入队列，接收解析后的点云数据
 * @param running 原子布尔变量，控制线程运行状态
 */
LiDARWriter::LiDARWriter(moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& iQ,
                const std::atomic<bool>& running): 
    inQ_(iQ),      // 输入队列（引用）
    gRunning(running)  // 线程运行状态（引用）
{}

/**
 * @brief LiDARWriter析构函数
 */
LiDARWriter::~LiDARWriter(){}

/**
 * @brief LiDAR写入线程主循环
 * @details 从输入队列获取点云数据，写入LAZ压缩文件，按秒级时间戳命名文件
 */
void LiDARWriter::run(){
    // 记录启动日志
    Logger::writeLogger()->info("LiDAR Writer线程启动，开始写入点云数据到LAZ文件，目录：{}", dir_);
    std::string filename = "";
    
    // 主循环：受gRunning原子变量控制
    while(gRunning)
    {
        std::shared_ptr<PointCloudData> pc;
        
        // 尝试从队列取出点云数据（非阻塞）
        if(inQ_.try_dequeue(pc)){
            // 生成文件名：目录 + 秒级时间戳
            filename = dir_ + "/" + std::to_string(pc->timeStamp / 1000);
            // 保存点云到LAZ文件
            savePointCloudToLAZ(pc->points, filename);
            // 原子计数：总写入数据包数
            total_packets.fetch_add(1, std::memory_order_relaxed);
        } else {
            // 队列为空时短暂休眠
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

/**
 * @brief 将点云数据保存为LAZ压缩文件
 * @param points 三维点云数据数组
 * @param outputFilePath 输出文件路径（不含扩展名）
 * @details 使用LASzip库创建LAZ文件，支持坐标缩放、偏移和边界框计算
 */
void LiDARWriter::savePointCloudToLAZ(const std::vector<Eigen::Vector3d>& points,
    const std::string& outputFilePath) 
{
    // 性能统计：写入耗时
    Telemetry::ScopedTimerMs timer(write_time_ms);
    
    // 空点云检查
    if (points.empty()) {
        Logger::writeLogger()->warn("错误：点云为空，无法保存。");
        return;
    }

    // 1. 初始化LASzip写入器
    laszip_POINTER writer;
    if (laszip_create(&writer) != 0) {
        Logger::writeLogger()->error("创建laszip写入器失败");
        return;
    }

    // 2. 获取并配置LAS文件头信息
    laszip_header* header;
    if (laszip_get_header_pointer(writer, &header) != 0) {
        Logger::writeLogger()->error("获取头信息指针失败");
        laszip_destroy(writer);
        return;
    }

    // 设置LAS文件版本和格式
    header->version_major = 1;                  // LAS 1.2版本
    header->version_minor = 2;
    header->point_data_format = 2;              // 点数据格式2（包含GPS时间和RGB）
    header->point_data_record_length = 26;      // 格式2的记录长度为26字节
    
    // 设置系统和软件标识
    strncpy(header->system_identifier, "LiDAR System", 32);       // 系统标识
    strncpy(header->generating_software, "Projection Module", 32); // 生成软件
    
    // 设置坐标缩放因子（毫米级精度）
    header->x_scale_factor = 0.001;
    header->y_scale_factor = 0.001;
    header->z_scale_factor = 0.001;
    
    // 计算点云边界框（用于头信息和坐标偏移）
    double min_x = std::numeric_limits<double>::max();  // 初始化为最大值
    double max_x = std::numeric_limits<double>::lowest();// 初始化为最小值
    double min_y = min_x, max_y = max_x;
    double min_z = min_x, max_z = max_x;

    // 遍历所有点计算边界
    for (const auto& point : points) 
    {
        min_x = std::min(min_x, point.x());
        min_y = std::min(min_y, point.y());
        min_z = std::min(min_z, point.z());
        max_x = std::max(max_x, point.x());
        max_y = std::max(max_y, point.y());
        max_z = std::max(max_z, point.z());
    }

    // 设置坐标偏移（使用最小值作为偏移，减少存储位数）
    header->x_offset = min_x;
    header->y_offset = min_y;
    header->z_offset = min_z;
    
    // 设置边界框信息
    header->min_x = min_x;
    header->max_x = max_x;
    header->min_y = min_y;
    header->max_y = max_y;
    header->min_z = min_z;
    header->max_z = max_z;

    // 设置点总数
    header->number_of_point_records = points.size();
    
    // 设置文件创建时间
    time_t current_time = time(nullptr);
    struct tm* time_info = localtime(&current_time);
    header->file_creation_year = 1900 + time_info->tm_year;  // 年份（1900为基准）
    header->file_creation_day = time_info->tm_yday;          // 年内天数

    // 3. 打开LAZ文件进行写入
    laszip_BOOL compress = 1;  // 启用压缩（LAZ格式）
    std::string filePath = outputFilePath + ".laz";  // 添加文件扩展名
    
    if (laszip_open_writer(writer, filePath.c_str(), compress) != 0) {
        laszip_CHAR* error;
        laszip_get_error(writer, &error);
        Logger::writeLogger()->error("打开LAZ文件失败: {}", std::string(error));
        laszip_destroy(writer);
        return;
    }

    // 4. 获取点数据指针，准备写入
    laszip_point* point;
    if (laszip_get_point_pointer(writer, &point) != 0) {
        Logger::writeLogger()->error("获取点数据指针失败");
        laszip_close_writer(writer);
        laszip_destroy(writer);
        return;
    }

    // 5. 写入所有点数据
    // Logger::writeLogger()->info("开始写入点云数据，共 {} 个点", points.size());

    // 遍历所有点
    for (size_t i = 0; i < points.size(); ++i) {
        // 计算并设置点坐标（应用缩放因子和偏移量）
        point->X = static_cast<laszip_I32>((points[i].x() - header->x_offset) / header->x_scale_factor);
        point->Y = static_cast<laszip_I32>((points[i].y() - header->y_offset) / header->y_scale_factor);
        point->Z = static_cast<laszip_I32>((points[i].z() - header->z_offset) / header->z_scale_factor);
            
        // 写入单个点
        if (laszip_write_point(writer) != 0) {
            Logger::writeLogger()->warn("写入第 {} 个点时发生警告", i);
        }
    }

    Logger::writeLogger()->debug("点云数据写入完成");

    // 6. 关闭文件并释放资源
    if (laszip_close_writer(writer) != 0) {
        laszip_CHAR* error;
        laszip_get_error(writer, &error);
        Logger::writeLogger()->error("关闭LAZ文件失败: {}", std::string(error));
    } else {
        Logger::writeLogger()->info("LAZ文件已成功保存至: {}", filePath);
    }
    
    // 销毁写入器
    laszip_destroy(writer);
}
