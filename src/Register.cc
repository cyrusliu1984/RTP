/**
 * @file Register.cpp
 * @brief 点云配准核心模块实现
 * @details 实现LiDAR点云从雷达坐标系到世界坐标系的多线程转换，支持：
 *          1. 雷达到IMU坐标系转换（设备安置角校正）
 *          2. IMU(Body)到世界坐标系(UTM)转换（结合POS姿态和位置数据）
 *          3. ROI区域裁剪（只保留感兴趣区域内的点云）
 *          4. POS数据时间戳匹配与插值
 */
#include <spdlog/fmt/ostr.h>
#include "Register.h"
#include "Logger.h"
#include "thread_pool/BS_thread_pool.hpp"

// 匿名命名空间 - 仅当前编译单元可见的常量/函数
namespace {

}

/**
 * @brief CloudRegister构造函数
 * @param posRing POS数据环状缓冲区（存储最近20秒POS数据）
 * @param lidarQ LiDAR点云输入队列
 * @param colorQ 配准后点云输出到着色模块的队列
 * @param writeQ 配准后点云输出到写入模块的队列
 * @param policy 配准策略（线程数、时间匹配阈值等）
 * @param running 原子布尔变量，控制模块运行状态
 * @details 初始化配置参数，加载ROI区域设置，记录初始化日志
 */
CloudRegister::CloudRegister(PosRing<std::shared_ptr<POSData>>& posRing,
               moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& lidarQ,
               moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& colorQ,
               moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& writeQ,
               const MatchPolicy& policy,
               const std::atomic<bool>& running)
    : pos_(posRing),          // POS数据环状缓冲区（引用）
      lidarQ_(lidarQ),        // LiDAR点云输入队列（引用）
      colorQ_(colorQ),        // 着色模块输入队列（引用）
      writeQ_(writeQ),        // 写入模块输入队列（引用）
      policy_(policy),        // 配准策略配置
      running_(running)       // 运行状态标志（引用）
{
    // 从配置管理器加载ROI相关配置
    roi_enabled_ = ConfigManager::getInstance().get<bool>("FLIGHT.ROI_ENABLED");          // 是否启用ROI裁剪
    roi_target_.lat_min = ConfigManager::getInstance().get<double>("FLIGHT.ROI_AREA.LAT_MIN"); // ROI最小纬度
    roi_target_.lat_max = ConfigManager::getInstance().get<double>("FLIGHT.ROI_AREA.LAT_MAX"); // ROI最大纬度
    roi_target_.lon_min = ConfigManager::getInstance().get<double>("FLIGHT.ROI_AREA.LON_MIN"); // ROI最小经度
    roi_target_.lon_max = ConfigManager::getInstance().get<double>("FLIGHT.ROI_AREA.LON_MAX"); // ROI最大经度
    altitude_ = ConfigManager::getInstance().get<double>("FLIGHT.ALTITUDE");             // 飞行高度
    speed_ = ConfigManager::getInstance().get<double>("FLIGHT.SPEED");                   // 飞行速度

    // 记录ROI配置信息
    if (roi_enabled_) {
        Logger::registerLogger()->info(
            "点云配准启用ROI裁剪，纬度范围：{:.6f} ~ {:.6f}，经度范围：{:.6f} ~ {:.6f}，只下传范围内点云数据",
            roi_target_.lat_min, roi_target_.lat_max, roi_target_.lon_min, roi_target_.lon_max
        );
    } else {
        Logger::registerLogger()->info("点云配准未启用ROI裁剪，下传全部点云数据");
    }   
}

/**
 * @brief 点云配准模块主运行函数
 * @details 启动多线程处理点云配准任务，每个工作线程：
 *          1. 从LiDAR队列获取点云数据
 *          2. 坐标转换（雷达→IMU→世界坐标系）
 *          3. POS时间戳匹配与校验
 *          4. ROI区域裁剪
 *          5. 输出配准后的点云到后续模块
 */
void CloudRegister::run(){
    // 记录模块启动信息，包含配置的工作线程数量
    Logger::registerLogger()->info("点云配准模块启动，工作线程数：{}", policy_.workers);

    // 定义工作线程的处理逻辑（lambda表达式）
    auto registerWorker = [this]() {
        // 记录当前工作线程启动，包含线程ID便于调试
        Logger::registerLogger()->debug("工作线程启动，线程ID：{}", std::this_thread::get_id());

        // 工作线程主循环：持续处理直到收到停止信号
        while (running_) {
            // 存储从队列取出的点云数据
            std::shared_ptr<PointCloudData> cloud;
            
            // 尝试从LiDAR点云队列非阻塞获取数据
            if (!lidarQ_.try_dequeue(cloud)) {
                // 队列为空时短暂休眠（5ms），减少CPU空转
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            
            // 校验点云数据有效性
            if (!cloud || cloud->points.empty() || cloud->times.empty()) {
                Logger::registerLogger()->warn("无效点云（空指针/点集合为空/时间戳为空），已跳过");
                continue;
            }

            // 提取点云基准时间戳（毫秒级）
            int64_t cloud_ts_ms = cloud->timeStamp;
            Logger::registerLogger()->debug(
                "处理点云数据，首个时间戳：{:.3f}秒，点数量：{}",
                cloud_ts_ms/1000.0 + cloud->times.front(),  cloud->points.size()
            );
            
            // 获取当前POS数据快照（线程安全的只读视图）
            const PosRing<std::shared_ptr<POSData>>::Snapshot snap = pos_.snapshot(); 
            
            // 检查POS数据是否为空（读指针>=写指针表示无数据）
            if (snap.r >= snap.w) {
                Logger::registerLogger()->warn("POS数据为空，无法进行点云配准，已跳过该点云");
                continue;
            }
            
            // 获取POS数据的时间范围
            int64_t posStartTs = snap.ts[snap.r & snap.cap_mask];       // 最早POS时间戳
            int64_t posEndTs = snap.ts[(snap.w - 1) & snap.cap_mask];  // 最晚POS时间戳
            Logger::registerLogger()->debug(
                "当前POS数据时间范围：{:.3f} ~ {:.3f}", 
                posStartTs/1000.0, posEndTs/1000.0
            );

            // TODO 0905 test3 - 测试代码（待移除）
            // cloud_ts_ms = posStartTs + 10000;

            // 检查点云时间戳是否在POS数据时间范围内
            // 情况1：点云时间早于POS数据起始时间
            if (cloud_ts_ms < posStartTs) {
                Logger::registerLogger()->warn(
                    "点云时间{:.3f}秒早于POS数据范围，已跳过该点云", 
                    cloud_ts_ms/1000.0
                );
                // 记录时间戳超前事件（用于监控统计）
                recordEarly(
                    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count(),
                    posStartTs, cloud_ts_ms
                );
                continue;
            } 
            // 情况2：点云时间晚于POS数据结束时间（预留1100ms余量）
            else if (cloud_ts_ms > posEndTs - 1100) {
                Logger::registerLogger()->warn(
                    "点云时间{:.3f}秒晚于POS数据范围，已跳过该点云", 
                    cloud_ts_ms/1000.0
                );
                // 记录时间戳滞后事件（用于监控统计）
                recordLate(
                    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count(),
                    posEndTs, cloud_ts_ms
                );
                continue;
            } 
            // 情况3：点云时间在POS数据范围内，进行坐标转换
            else {
                // ---------- 第一步：激光雷达坐标系 → IMU坐标系 ----------
                // 注：经过设备内部安置角配置后，IMU坐标系等价于载体坐标系(Body Frame)
                Telemetry::ScopedTimerMs timer(register_ts_ms); // 性能统计：记录配准总耗时
                
                in_roi_ = false; // 标记：本次点云是否存在ROI内的点

                // 存储转换到IMU坐标系后的点云
                PointCloudData pointsImu;
                pointsImu.timeStamp = cloud->timeStamp;       // 保持基准时间戳不变
                pointsImu.times = cloud->times;                // 保持相对时间戳不变
                pointsImu.points.resize(cloud->points.size()); // 预分配内存
                
                {
                    // 获取雷达到IMU的转换参数（安置角、平移量）
                    LidarToImuParams lidar2imuParam;
                    
                    // 将欧拉角转换为旋转矩阵（yaw-pitch-roll旋转顺序）
                    Eigen::Matrix3d R0 = Projection::eulerToRotm(
                        lidar2imuParam.yaw,    // 偏航角
                        lidar2imuParam.pitch,  // 俯仰角
                        lidar2imuParam.roll    // 横滚角
                    );
                    
                    // 逐点进行坐标转换：P_imu = R0^T * P_lidar + 平移量
                    // 注：R0.transpose() 等价于逆矩阵（正交矩阵）
                    for (size_t i = 0; i < cloud->points.size(); ++i) {
                        pointsImu.points[i] = R0.transpose() * cloud->points[i] + lidar2imuParam.translation;
                    }
                    // Logger::registerLogger()->info("雷达到IMU坐标转换完成，已处理{}个点", cloud->points.size());
                }

                // ---------- 第二步：Body Frame → Navi Frame → ECEF → 世界坐标系（UTM） ----------
                // 存储转换到世界坐标系后的点云
                PointCloudData world;
                world.timeStamp = pointsImu.timeStamp;               // 保持基准时间戳
                world.times.reserve(pointsImu.points.size());        // 预分配内存提升效率
                world.points.reserve(pointsImu.points.size());

                // 逐点匹配POS数据并转换到世界坐标系
                for (size_t i = 0; i < pointsImu.points.size(); ++i) {
                    // 计算当前点的绝对时间戳（基准时间+相对时间）
                    const int64_t point_ts_ms = cloud_ts_ms + cloud->times[i];
                    
                    // 在POS快照中查找小于等于当前点时间的最后一个POS和下一个POS
                    auto [prevIt, nextIt] = pos_.lower_pair(snap, point_ts_ms);
                    
                    // 选择时间差更小的POS数据（最近邻匹配）
                    const int64_t nearstIt = (std::abs(snap.ts[(prevIt & snap.cap_mask)] - point_ts_ms) 
                                           <= std::abs(snap.ts[(nextIt & snap.cap_mask)] - point_ts_ms)) 
                                           ? prevIt : nextIt;
                    
                    // 检查时间匹配误差是否超过阈值
                    if (std::abs(snap.ts[(nearstIt & snap.cap_mask)] - point_ts_ms) > policy_.max_nn_error_ms) {
                        // 记录匹配失败事件（用于监控统计）
                        recordFail(
                            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count(),
                            snap.ts[(nearstIt & snap.cap_mask)], point_ts_ms
                        );
                        
                        // 记录错误日志
                        Logger::registerLogger()->error(
                            "点时间{:.3f}秒与最近POS时间{:.3f}秒差值{:.3f}秒，超过阈值{}毫秒，已跳过该点",
                            point_ts_ms / 1000.0, snap.ts[(nearstIt & snap.cap_mask)] / 1000.0,
                            std::abs(snap.ts[(nearstIt & snap.cap_mask)] - point_ts_ms) / 1000.0,
                            policy_.max_nn_error_ms
                        );
                        Logger::registerLogger()->debug(
                            "当前POS数据时间范围：{:.3f} ~ {:.3f}", 
                            posStartTs/1000.0, posEndTs/1000.0
                        );
                        Logger::registerLogger()->debug(
                            "上一个时间戳 {:.3f}, 下一个时间戳{:.3f}", 
                            snap.ts[((nearstIt-1) & snap.cap_mask)] / 1000.0, 
                            snap.ts[((nearstIt+1) & snap.cap_mask)] / 1000.0
                        );
                        continue; // 跳过当前点
                    }
                    
                    // 获取匹配到的POS数据
                    std::shared_ptr<POSData> posPtr = snap.val[(nearstIt & snap.cap_mask)];
                    if (!posPtr) {
                        Logger::registerLogger()->warn("时间{}毫秒对应的POS数据为空，已跳过该点", point_ts_ms);
                        continue;
                    }

                    // ---------- ROI区域判断 ----------
                    // 计算当前位置的动态ROI区域
                    roi_current_ = Proj::computeROI(posPtr->latitude, posPtr->longitude, altitude_, speed_);
                    // 检查当前ROI与目标ROI是否相交
                    if (Proj::is_intersect(roi_current_, roi_target_)) {
                        in_roi_ = true; // 标记本次点云存在ROI内的点
                    } 
                    
                    // ---------- 坐标转换：IMU→世界坐标系 ----------
                    // 1. 将POS的经纬度转换为UTM坐标（东向、北向）
                    auto utm = Projection::projection(posPtr->longitude, posPtr->latitude); 

                    // 2. 构建IMU姿态旋转矩阵（Navi→Body）
                    Eigen::Matrix3d R_imu = Projection::eulerToRotm(
                        -posPtr->heading,  // 偏航角（取负：Body→Navi）
                        posPtr->pitch,     // 俯仰角
                        posPtr->roll       // 横滚角
                    );

                    // 3. 定义世界坐标系原点（UTM东向、北向 + 海拔高度）
                    const Eigen::Vector3d origin_xyz(utm.first, utm.second, posPtr->altitude);
                    
                    // 4. 计算世界坐标系下的点坐标：P_world = R_imu * P_imu + 原点坐标
                    const Eigen::Vector3d Pw = R_imu * pointsImu.points[i] + origin_xyz;

                    // 5. 将转换后的点添加到结果点云
                    world.points.push_back(Pw);
                    world.times.push_back(pointsImu.times[i]);
                }

                // 检查转换后是否有有效点云
                if (world.points.empty()) {
                    Logger::registerLogger()->warn("IMU到世界坐标系转换后无有效点，已跳过后续处理");
                    continue;
                }

                // 原子计数：处理完成的数据包数（用于监控统计）
                total_packets.fetch_add(1, std::memory_order_relaxed);
                
                // 记录配准耗时和处理点数
                Logger::registerLogger()->debug(
                    "点云处理完成，耗时：{}毫秒，点数量：{}", 
                    register_ts_ms.snapshot().mean, cloud->points.size()
                );

                // ---------- 第三步：输出配准后的点云到后续模块 ----------
                // 如果启用ROI且本次点云无ROI内的点，则丢弃
                if (roi_enabled_ && !in_roi_) {
                    Logger::registerLogger()->info(
                        "点云时间{:.3f}秒所有点均不在ROI内，已丢弃该点云", 
                        cloud_ts_ms/1000.0
                    );
                    continue;
                } 

                // 创建共享指针（使用move语义避免拷贝）
                auto world_ptr = std::make_shared<PointCloudData>(std::move(world));
                
                // 投递到着色模块队列
                colorQ_.enqueue(world_ptr);
                // 投递到写入模块队列
                writeQ_.enqueue(world_ptr);
            }
        }
        
        // 记录工作线程退出
        Logger::registerLogger()->debug("工作线程退出，线程ID：{}", std::this_thread::get_id());
    };

    // ---------- 启动工作线程池 ----------
    std::vector<std::thread> threads;
    threads.reserve(policy_.workers); // 预分配线程容器空间
    
    // 启动指定数量的工作线程
    for (size_t i = 0; i < policy_.workers; ++i) {
        threads.emplace_back(registerWorker);
        Logger::registerLogger()->info("已启动第 {}/{} 个工作线程", i+1, policy_.workers);
    }

    // 阻塞等待所有工作线程完成
    for (auto &th : threads) {
        if (th.joinable()) {
            th.join();
        }
    }

    // 模块退出日志
    Logger::registerLogger()->info("点云配准模块run()方法执行完毕");
}
