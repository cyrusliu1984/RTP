#include "POS.h"
#include "Logger.h"
#include "RayTracing.h"
#include "utils.h"
#include "config_manager.h"

// 系统头文件
#include <cstring>          // 内存操作函数
#include <thread>           // 线程相关
#include <chrono>           // 时间处理
#include <errno.h>          // 错误码定义
#include <cmath>            // 数学函数
#include <iostream>         // 标准输入输出
#include <fstream>          // 文件流
#include <filesystem>       // 文件系统 API

#ifdef _WIN32
    #include <windows.h>
#else
    #include <fcntl.h>          // 文件控制定义
    #include <unistd.h>         // POSIX系统调用
    #include <termios.h>        // 终端I/O控制
    #include <sys/ioctl.h>      // I/O控制操作
    #include <sched.h>          // 调度策略/实时优先级
#endif

// 匿名命名空间 - 仅当前编译单元可见的常量
namespace {
    constexpr int64_t RING_TIMEWINDOW = 20'000; // 20s (ms)
}

POSReceiver::POSReceiver(moodycamel::ConcurrentQueue<uint8_t>& outQ, 
                         moodycamel::ConcurrentQueue<uint8_t>& writerQ, 
                         const std::atomic<bool>& running)
    : outputQueue(outQ),
      writerQueue(writerQ),
      gRunning(running),
      buffer(BUFFER_SIZE)
{
    serialPort = ConfigManager::getInstance().get<std::string>("POS.RECEIVER.COMPORT");
    baudrate = ConfigManager::getInstance().get<int>("POS.RECEIVER.BAUDRATE") / 8;
    
    serialHandle = openSerialPort(serialPort, baudrate);
    
    if (serialHandle != (serial_handle_t)-1) {
        Logger::posReceiverLogger()->info("成功打开串口: {}, 波特率: {}", serialPort, baudrate);
    }
}

POSReceiver::~POSReceiver() {
    if (serialHandle != (serial_handle_t)-1) {
#ifdef _WIN32
        CloseHandle(serialHandle);
#else
        close(serialHandle);
#endif
        serialHandle = (serial_handle_t)-1;
        Logger::posReceiverLogger()->info("串口已关闭: {}", serialPort);
    }
}

void POSReceiver::setupRealTimePriority() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    Logger::posReceiverLogger()->info("设置POS线程优先级成功 (THREAD_PRIORITY_HIGHEST)");
#else
    if (geteuid() == 0) {
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 4;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            Logger::posReceiverLogger()->warn("设置POS线程实时优先级失败: {}", strerror(errno));
        } else {
            Logger::posReceiverLogger()->info("设置POS线程实时优先级成功（SCHED_FIFO, 优先级: {})", param.sched_priority);
        }
    } else {
        Logger::posReceiverLogger()->warn("非root运行，无法设置POS线程实时优先级!");
    }
#endif
}

serial_handle_t POSReceiver::openSerialPort(const std::string& portName, int baudrate) {
#ifdef _WIN32
    std::string winPort = "\\\\.\\" + portName;
    HANDLE hComm = CreateFileA(winPort.c_str(), GENERIC_READ | GENERIC_WRITE,
                               0, NULL, OPEN_EXISTING, 0, NULL);
    if (hComm == INVALID_HANDLE_VALUE) {
        Logger::posReceiverLogger()->error("打开串口失败: {}", portName);
        return (serial_handle_t)-1;
    }

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hComm, &dcb);
    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    SetCommState(hComm, &dcb);

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 1000;
    SetCommTimeouts(hComm, &timeouts);
    return hComm;
#else
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY);  
    if (fd < 0) {
        Logger::posReceiverLogger()->error("打开串口失败: {}，错误: {}", portName, strerror(errno));
        return -1;
    }

    if (!isatty(fd)) {
        Logger::posReceiverLogger()->error("{} 不是终端设备", portName);
        close(fd);
        return -1;
    }

    termios tio{}; 
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) {
        Logger::posReceiverLogger()->error("获取串口属性失败: {}", strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);

    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag |= CLOCAL | CREAD;

    speed_t speed = B115200;
    switch (baudrate) {
        case 9600:    speed = B9600;    break;
        case 19200:   speed = B19200;   break;
        case 38400:   speed = B38400;   break;
        case 57600:   speed = B57600;   break;
        case 115200:  speed = B115200;  break;
        case 230400:  speed = B230400;  break;
        case 460800:  speed = B460800;  break;
        case 921600:  speed = B921600;  break;
        default: break;
    }

    if (cfsetospeed(&tio, speed) != 0 || cfsetispeed(&tio, speed) != 0) {
        Logger::posReceiverLogger()->error("设置波特率失败: {}", strerror(errno));
        close(fd);
        return -1;
    }

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        Logger::posReceiverLogger()->error("设置串口属性失败: {}", strerror(errno));
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
#endif
}

void POSReceiver::run() {
    setupRealTimePriority();
    Logger::posReceiverLogger()->info("POS接收线程启动");

    while (gRunning)
    {
        int bytesRead = 0;
#ifdef _WIN32
        DWORD dwRead = 0;
        if (serialHandle != INVALID_HANDLE_VALUE && ReadFile(serialHandle, buffer.data(), BUFFER_SIZE, &dwRead, NULL)) {
            bytesRead = static_cast<int>(dwRead);
        }
#else
        ssize_t n = read(serialHandle, buffer.data(), BUFFER_SIZE);
        bytesRead = static_cast<int>(n);
#endif

        if (bytesRead > 0) {
            bool okToParser = outputQueue.enqueue_bulk(buffer.data(), bytesRead);
            if (!okToParser) {
                Logger::posReceiverLogger()->warn("队列已满，丢弃 {} 字节数据", bytesRead);
            }
            
            bool okToWriter = writerQueue.enqueue_bulk(buffer.data(), bytesRead);
            if (!okToWriter) {
                Logger::posReceiverLogger()->warn("写文件队列已满，丢弃 {} 字节数据", bytesRead);
            }
        } 
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    
    Logger::posReceiverLogger()->info("POS接收线程退出");
}

/**
 * @brief POSParser构造函数
 * @param inQ 输入队列，接收POSReceiver的原始数据
 * @param outQ1 POS数据环状缓冲区，存储解析后的POSData
 * @param outQ2 Mark数据队列，存储解析后的MarkData
 * @param running 原子布尔变量，控制线程运行状态
 */
POSParser::POSParser(moodycamel::ConcurrentQueue<uint8_t>& inQ,
                   PosRing<std::shared_ptr<POSData>>& outQ1,
                   moodycamel::ConcurrentQueue<std::shared_ptr<MarkData>>& outQ2,
                   const std::atomic<bool>& running)
    : gRunning(running),    // 线程运行状态（引用）
      inputQueue(inQ),      // 原始数据输入队列（引用）
      posRing(outQ1),       // POS数据环状缓冲区（引用）
      markQueue(outQ2)      // Mark数据输出队列（引用）
{
    Logger::posParserLogger()->info("POS数据解析线程启动！");
}

/**
 * @brief POSParser析构函数
 * @details 记录解析器关闭日志
 */
POSParser::~POSParser() {
    Logger::posParserLogger()->info("POS数据解析线程关闭！");
}

/**
 * @brief 解析Mark消息原始数据为结构化MarkData
 * @param data 指向Mark原始数据的指针
 * @param output 输出参数，解析后的MarkData结构体
 * @details 转换GPS时间为Linux时间戳，拷贝位置姿态信息和CRC校验值
 */
void POSParser::parseMarkMessage(const uint8_t* data, MarkData& output){
    // 将原始字节数据转换为MarkRawData结构体
    const MarkRawData* tmpRawData = reinterpret_cast<const MarkRawData*>(data);
    
    // 转换GPS时间（周+秒）为Linux毫秒级时间戳
    output.timeStamp = TimeUtils::gpsTimeToLinuxTimestamp(
        tmpRawData->gpsWeek, 
        static_cast<uint32_t>(tmpRawData->secInWeek * 1000.0), 
        TimeUtils::TimeUnit::Milliseconds
    );

    // 拷贝基础信息
    output.week = tmpRawData->gpsWeek;
    output.seconds = tmpRawData->secInWeek;
    output.latitude = tmpRawData->lat;      // 纬度
    output.longitude = tmpRawData->lon;     // 经度
    output.altitude = tmpRawData->altitude; // 高度
    output.roll = tmpRawData->roll;         // 横滚角
    output.pitch = tmpRawData->pitch;       // 俯仰角
    output.heading = tmpRawData->yaw;       // 航向角（偏航角）

    // 拷贝CRC校验值
    output.CRC = tmpRawData->CRC;
}

/**
 * @brief 解析POS消息原始数据为结构化POSData
 * @param data 指向POS原始数据的指针
 * @param output 输出参数，解析后的POSData结构体
 * @details 转换GPS时间为Linux时间戳，拷贝位置姿态信息和CRC校验值
 */
void POSParser::parsePosMessage(const uint8_t* data, POSData& output){
    // 将原始字节数据转换为POSRawData结构体
    const POSRawData* tmpRawData = reinterpret_cast<const POSRawData*>(data);
    
    // 转换GPS时间（周+毫秒）为Linux毫秒级时间戳
    output.timeStamp = TimeUtils::gpsTimeToLinuxTimestamp(
        tmpRawData->gpsWeek, 
        tmpRawData->msecInWeek, 
        TimeUtils::TimeUnit::Milliseconds
    );
    
    // 调试日志（已注释）
    // Logger::lidarParserLogger()->debug("POS时间戳转换结果：{}周 - 0x{:X}秒", tmpRawData->gpsWeek, tmpRawData->msecInWeek);
    // Logger::lidarParserLogger()->debug("POS经纬高解算结果：{} - {} - {}", tmpRawData->lon, tmpRawData->lat, tmpRawData->altitude);
    
    // 拷贝基础信息
    output.gpsWeek = tmpRawData->gpsWeek;       // GPS周数
    output.gpsMilliseconds = tmpRawData->msecInWeek; // 周内毫秒数
    output.latitude = tmpRawData->lat;          // 纬度
    output.longitude = tmpRawData->lon;         // 经度
    output.altitude = tmpRawData->altitude;     // 高度
    output.roll = tmpRawData->roll;             // 横滚角
    output.pitch = tmpRawData->pitch;           // 俯仰角
    output.heading = tmpRawData->yaw;           // 航向角（偏航角）
    
    // 拷贝CRC校验值
    output.CRC = tmpRawData->CRC;
}

/**
 * @brief POS解析线程主循环
 * @details 从输入队列读取原始数据，解析POS/Mark消息，进行CRC校验和时间戳检查
 *          使用环状缓冲区存储POS数据，支持旧数据自动覆盖
 */
void POSParser::run() {
    std::vector<uint8_t> parseBuffer;       // 解析缓冲区，用于拼接不完整的消息
    std::vector<uint8_t> dequeueBuffer(1024); // 队列读取缓冲区（1024字节）

    // 主循环：受gRunning原子变量控制
    while (gRunning) {
        // 从输入队列批量读取数据（非阻塞）
        size_t n = inputQueue.try_dequeue_bulk(dequeueBuffer.data(), dequeueBuffer.size());

        // 成功读取到数据，追加到解析缓冲区
        if (n > 0) {
            parseBuffer.insert(parseBuffer.end(), dequeueBuffer.begin(), dequeueBuffer.begin() + n);
        } 
        // 队列为空时短暂休眠，降低CPU占用
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // 解析缓冲区中的数据
        size_t i = 0;
        while (i + 3 <= parseBuffer.size()) {
            // 查找消息头：0xAA 0x44 0x12
            if (parseBuffer[i] == 0xAA && parseBuffer[i+1] == 0x44 && parseBuffer[i+2] == 0x12) {
                // 检查是否有足够数据读取MessageId（至少还需要3字节：保留位+2字节MessageId）
                if (i + 6 > parseBuffer.size()) break;

                // 解析MessageId（小端序：低字节在前，高字节在后）
                uint16_t messageId = (parseBuffer[i+4]) | (parseBuffer[i+5] << 8);
                size_t structLen = 0;

                // 根据MessageId确定消息类型和结构体大小
                if (messageId == 1068) {
                    structLen = sizeof(MarkRawData); // HIK Mark消息
                } else if (messageId == 1465) {
                    structLen = sizeof(POSRawData);  // POS主消息
                } else if (messageId == 1067) {
                    structLen = sizeof(MarkRawData); // PH Mark消息
                } else {
                    // 未知MessageId，记录警告并跳过当前字节
                    Logger::posParserLogger()->warn("未知MessageId: {}", messageId);
                    i++;
                    continue;
                }

                // 检查缓冲区是否有完整的消息数据
                if (i + structLen > parseBuffer.size()) break;

                // 处理HIK Mark消息 (MessageId=1068)
                if (messageId == 1068) {
                    MarkData newMarkData;
                    // 解析Mark消息
                    parseMarkMessage(parseBuffer.data() + i, newMarkData);
                    
                    // CRC校验（计算除最后4字节CRC外的数据的CRC32）
                    if (Checksum::CalculateBlockCRC32(sizeof(MarkRawData) - 4, parseBuffer.data() + i) != newMarkData.CRC) {
                        Logger::posParserLogger()->warn("当前CRC检校未通过!");
                    } else {
                        // 检查时间戳是否单调递增
                        if (lastMarkEnqueuedTime_ >= newMarkData.timeStamp) {
                            Logger::posParserLogger()->warn(
                                "Mark数据入队时间不严格递增，当前队列最后元素时间：{} | 待入队元素时间：{} 放弃入队！", 
                                lastMarkEnqueuedTime_, newMarkData.timeStamp
                            );
                        } else {
                            // 入队到Mark数据队列
                            markQueue.enqueue(std::make_shared<MarkData>(newMarkData));
                            // 原子计数：HIK Mark消息数
                            hik_mark_frames.fetch_add(1, std::memory_order_relaxed);
                            // 更新最后入队时间戳
                            lastMarkEnqueuedTime_ = newMarkData.timeStamp;
                            Logger::posParserLogger()->debug("Mark数据解析成功！");
                        }
                    }
                } 
                // 处理POS主消息 (MessageId=1465)
                else if (messageId == 1465) {
                    POSData newPOSData;
                    // 解析POS消息
                    parsePosMessage(parseBuffer.data() + i, newPOSData);
                    
                    // CRC校验（计算除最后4字节CRC外的数据的CRC32）
                    if (Checksum::CalculateBlockCRC32(sizeof(POSRawData) - 4, parseBuffer.data() + i) != newPOSData.CRC) {
                        Logger::posParserLogger()->warn("当前CRC检校未通过!");
                    } else {
                        // 检查时间戳是否单调递增
                        if (lastPosEnqueuedTime_ >= newPOSData.timeStamp) {
                            // 原子计数：POS时间非单调递增次数
                            pos_non_mono.fetch_add(1, std::memory_order_relaxed);
                            
                            // 记录事件信息
                            EventBasic ev;
                            ev.wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch()
                            ).count();
                            ev.type = EventType::POS_NON_MONO;       // 事件类型：POS时间非单调
                            ev.t0_ms = lastMarkEnqueuedTime_;         // 上一个时间戳
                            ev.t1_ms = newPOSData.timeStamp;          // 当前时间戳
                            ev.t_ms = newPOSData.timeStamp - lastMarkEnqueuedTime_; // 时间差
                            
                            // 加入事件队列
                            pos_events.push(ev);
                            
                            // 记录警告日志
                            Logger::posParserLogger()->warn(
                                "POS数据入队时间不严格递增，当前队列最后元素时间：{} | 待入队元素时间：{} 放弃入队！", 
                                lastPosEnqueuedTime_, newPOSData.timeStamp
                            );
                        } else {
                            // 将POS数据推入环状缓冲区（20秒时间窗口）
                            auto x = posRing.push_window(newPOSData.timeStamp, std::make_shared<POSData>(newPOSData), RING_TIMEWINDOW);
                            // 原子计数：POS消息数
                            pos_frames.fetch_add(1, std::memory_order_relaxed);
                            
                            // 检查是否成功入队（缓冲区满则返回false）
                            if (!x) {
                                Logger::posParserLogger()->warn("POS环状缓冲区满，放弃入队时间：{} 的POS数据！", newPOSData.timeStamp);
                            }
                            
                            // 更新最后入队时间戳
                            lastPosEnqueuedTime_ = newPOSData.timeStamp;
                        }
                    }
                } 
                // 处理PH Mark消息 (MessageId=1067)
                else if (messageId == 1067) {
                    // 原子计数：PH Mark消息数（仅计数，不解析内容）
                    ph_mark_frames.fetch_add(1, std::memory_order_relaxed);
                }
                
                // 跳过当前消息的所有字节，处理下一个消息
                i += structLen;
            } 
            // 未找到消息头，跳过当前字节
            else { 
                i++;
            }
        }

        // 移除已处理的字节，保留未处理的数据在缓冲区
        if (i > 0) {
            parseBuffer.erase(parseBuffer.begin(), parseBuffer.begin() + i);
        }
    }
}

/**
 * @brief POSWriter构造函数
 * @param inQ 输入队列，接收POS原始数据
 * @param running 原子布尔变量，控制线程运行状态
 * @details 创建POS数据存储目录，生成带时间戳的文件名，打开二进制文件用于写入
 */
POSWriter::POSWriter(moodycamel::ConcurrentQueue<uint8_t>& inQ, const std::atomic<bool>& running)
    : inputQueue(inQ),
      gRunning(running)
{
    saveDir_ = "../data/POS";
    
    std::string fileName = TimeUtils::get_yymmdd_hhmmss();
    fileName = saveDir_ + "/POS_" + fileName + ".bin";

    std::error_code ec;
    std::filesystem::create_directories(saveDir_, ec);

#ifdef _WIN32
    HANDLE hFile = CreateFileA(fileName.c_str(), FILE_APPEND_DATA,
                               FILE_SHARE_READ, NULL, OPEN_ALWAYS,
                               FILE_ATTRIBUTE_NORMAL, NULL);
    if (hFile == INVALID_HANDLE_VALUE) {
        Logger::posReceiverLogger()->error("创建POS写入数据文件 {} 失败", fileName);
        posLogFd = (file_handle_t)-1;
    } else {
        posLogFd = hFile;
        Logger::posReceiverLogger()->info("创建POS写入数据文件成功: {}", fileName);
    }
#else
    posLogFd = ::open(fileName.c_str(), O_CREAT | O_APPEND | O_WRONLY, 0644);
    if (posLogFd < 0) {
        Logger::posReceiverLogger()->error("创建POS写入数据文件 {} 失败: {}", fileName, strerror(errno));
    } else {
        Logger::posReceiverLogger()->info("创建POS写入数据文件成功: {}", fileName);
    }
#endif
}

POSWriter::~POSWriter() {
    if (posLogFd != (file_handle_t)-1) {
#ifdef _WIN32
        FlushFileBuffers(posLogFd);
        CloseHandle(posLogFd);
#else
        ::fdatasync(posLogFd);
        ::close(posLogFd);
#endif
        posLogFd = (file_handle_t)-1;
        Logger::posReceiverLogger()->info("POS数据写入文件已关闭");
    }
}

bool POSWriter::write_all(file_handle_t fd, const uint8_t* p, size_t n) {
#ifdef _WIN32
    DWORD written = 0;
    if (WriteFile(fd, p, static_cast<DWORD>(n), &written, NULL)) {
        return written == static_cast<DWORD>(n);
    }
    return false;
#else
    while (n > 0) {
        ssize_t written = ::write(fd, p, n);
        if (written < 0) {
            if (errno == EINTR) continue;
            Logger::posReceiverLogger()->error("写入数据失败: {}", strerror(errno));
            return false;
        }
        p += static_cast<size_t>(written);
        n -= static_cast<size_t>(written);
    }
    return true;
#endif
}

void POSWriter::run() {
    Logger::posReceiverLogger()->info("POS原始数据写入线程启动");

    std::vector<uint8_t> dequeueBuffer(1024);

    while (gRunning) {
        size_t dataLen = inputQueue.try_dequeue_bulk(dequeueBuffer.data(), dequeueBuffer.size());

        if (dataLen > 0) {
            if (posLogFd != (file_handle_t)-1) {
                if (!write_all(posLogFd, dequeueBuffer.data(), dataLen)) {
                    Logger::posReceiverLogger()->warn("丢弃 {} 字节POS原始数据（写入失败）", dataLen);
                }
            } else {
                Logger::posReceiverLogger()->warn("POS原始数据文件未打开，丢弃 {} 字节数据", dataLen);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    Logger::posReceiverLogger()->info("POS原始数据写入线程退出");
}
