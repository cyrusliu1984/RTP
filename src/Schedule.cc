/**
 * @file Schedule.cpp
 * @brief 系统调度器核心实现
 * @details 负责：
 *          1. 系统所有模块的线程创建、启动和停止管理
 *          2. 全系统运行状态监控与性能指标统计
 *          3. 各模块异常捕获与系统紧急停止
 *          4. 关键指标速率计算与状态告警
 */
#include "Schedule.h"
#include "config_manager.h"
#include <exception>

// 匿名命名空间 - 全局匹配策略配置（点云配准模块使用）
namespace {
    MatchPolicy gMatchPolicy; // Register/CloudRegister 配准策略配置
}

/**
 * @brief 调度器构造函数
 * @param running 全局运行状态标志（原子变量）
 * @details 初始化系统所有功能模块，创建线程对象，绑定全局运行状态
 */
Schedule::Schedule(std::atomic<bool>& running)
    : gRunning_(running)                                  // 绑定全局运行状态标志
    // LiDAR模块初始化
    , lidarReceiver_(lidarReceiveQueue_, gRunning_)       // LiDAR数据接收模块
    , lidarParser_(lidarReceiveQueue_, lidarParseQueue_, gRunning_) // LiDAR数据解析模块
    , lidarWriter_(writeQueue_, gRunning_)                // LiDAR数据写入模块
    // POS模块初始化
    , posReceiver_(posBufferQueue_, posWriterQueue_, gRunning_) // POS数据接收模块
    , posParser_(posBufferQueue_, posRing_, markQueue_, gRunning_) // POS数据解析模块
    , posWriter_(posWriterQueue_, gRunning_)              // POS数据写入模块
    // 点云配准模块初始化
    , registrar_(posRing_, lidarParseQueue_, colorQueue_, writeQueue_, gMatchPolicy, gRunning_)
    // 其他功能模块初始化
    , dataTransmitter_(gRunning_)                         // 数据传输模块（网络发送）
{}

/**
 * @brief 调度器析构函数
 * @details 处理系统退出时的状态上报，记录调度器停止日志
 */
Schedule::~Schedule(){
    // 如果发生致命错误，更新状态并发送状态消息
    if (fatalError_) {
        statusStr_ = "状态：出错";
        SendStatusMessage(statusStr_);
    }
    // 记录调度器停止日志
    Logger::scheduleLogger()->info("调度器已停止");
}

/**
 * @brief 启动系统所有模块
 * @details 按依赖顺序启动各功能模块的工作线程，确保数据流程的正确性
 */
void Schedule::startAll(){
    Logger::scheduleLogger()->info("调度器已启动");
    
    // 启动顺序：先启动基础数据接收模块，再启动处理模块
    startPosReceive();      // POS数据接收
    startPosParse();        // POS数据解析
    startPosWriter();       // POS数据写入
    startLidarReceive();    // LiDAR数据接收
    startLidarParse();      // LiDAR数据解析
    startLidarWriter();     // LiDAR数据写入
    startRegister();        // 点云配准
    startDataTransmitter(); // 数据传输
    startMonitor();         // 系统监控（最后启动）
}

/**
 * @brief 停止系统所有模块
 * @details 按逆序停止所有工作线程，确保资源正确释放
 */
void Schedule::stopAll(){
    // 停止顺序：先停止上层处理模块，再停止基础接收模块
    JoinIfJoinable(tMonitor_);         // 监控线程
    JoinIfJoinable(tRegister_);        // 点云配准线程
    JoinIfJoinable(tLidarWriter_);     // LiDAR写入线程
    JoinIfJoinable(tLidarParse_);      // LiDAR解析线程
    JoinIfJoinable(tLidarRecv_);       // LiDAR接收线程
    JoinIfJoinable(tPosWriter_);       // POS写入线程
    JoinIfJoinable(tPosParse_);        // POS解析线程
    JoinIfJoinable(tPosRecv_);         // POS接收线程
    JoinIfJoinable(tDataTransmitter_); // 数据传输线程
    
    Logger::scheduleLogger()->info("调度器内线程均停止工作");
}

// ---- 分段启动实现（每个模块的线程启动函数）----
/**
 * @brief 启动POS数据接收线程
 */
void Schedule::startPosReceive(){ 
    if (!tPosRecv_.joinable())   
        tPosRecv_ = std::thread([this]{ RunTrampoline_PosReceive(); }); 
}

/**
 * @brief 启动POS数据解析线程
 */
void Schedule::startPosParse(){   
    if (!tPosParse_.joinable())  
        tPosParse_ = std::thread([this]{ RunTrampoline_PosParse(); }); 
}

/**
 * @brief 启动POS数据写入线程
 */
void Schedule::startPosWriter(){  
    if (!tPosWriter_.joinable()) 
        tPosWriter_ = std::thread([this]{ RunTrampoline_PosWriter(); }); 
}

/**
 * @brief 启动LiDAR数据接收线程
 */
void Schedule::startLidarReceive(){
    if (!tLidarRecv_.joinable()) 
        tLidarRecv_ = std::thread([this]{ RunTrampoline_LidarReceive(); }); 
}

/**
 * @brief 启动LiDAR数据解析线程
 */
void Schedule::startLidarParse(){  
    if (!tLidarParse_.joinable())
        tLidarParse_ = std::thread([this]{ RunTrampoline_LidarParse(); }); 
}

/**
 * @brief 启动LiDAR数据写入线程
 */
void Schedule::startLidarWriter(){ 
    if (!tLidarWriter_.joinable())
        tLidarWriter_ = std::thread([this]{ RunTrampoline_LidarWriter(); }); 
}

/**
 * @brief 启动点云配准线程
 */
void Schedule::startRegister(){    
    if (!tRegister_.joinable())  
        tRegister_ = std::thread([this]{ RunTrampoline_Register(); }); 
}

/**
 * @brief 启动数据传输线程
 */
void Schedule::startDataTransmitter(){ 
    if (!tDataTransmitter_.joinable()) 
        tDataTransmitter_ = std::thread([this]{ RunTrampoline_DataTransmitter(); }); 
}

/**
 * @brief 启动系统监控线程
 */
void Schedule::startMonitor(){ 
    if (!tMonitor_.joinable()) 
        tMonitor_ = std::thread([this]{ runMonitor(); }); 
}

/**
 * @brief 计算数据处理速率（每秒处理数量）
 * @param r 速率统计结构体
 * @param now_total 当前累计处理总数
 * @param now_ms 当前时间戳（毫秒）
 * @return 当前计算出的速率（个/秒）
 * @details 基于两次采样的时间差和数量差计算实时速率，处理时间回退边界情况
 */
double Schedule::calcRate(RateSnap& r, uint64_t now_total, uint64_t now_ms){
    // 首次采样，初始化基准值
    if (r.last_ms == 0) { 
        r.last = now_total; 
        r.last_ms = now_ms; 
        return 0.0; 
    }
    
    // 计算时间差（处理时间回退情况，取0）
    auto dt = (now_ms >= r.last_ms) ? now_ms - r.last_ms : 0ULL; 
    if (dt == 0) return r.rate; // 时间无变化，返回上次速率
    
    // 计算数量差（处理数量回退情况，取0）
    auto dv = (now_total >= r.last) ? now_total - r.last : 0ULL;
    
    // 计算速率：(数量差 * 1000) / 时间差 = 个/秒
    r.rate = (1000.0 * static_cast<double>(dv)) / static_cast<double>(dt);
    
    // 更新基准值
    r.last_ms = now_ms; 
    r.last = now_total; 
    return r.rate;
}

/**
 * @brief 系统监控线程主函数
 * @details 周期性采集并输出全系统各模块的运行状态和性能指标：
 *          1. 各模块处理速率、累计数量
 *          2. 处理耗时统计（均值、标准差）
 *          3. 队列状态监控
 *          4. 异常告警统计
 *          5. 系统运行时间
 */
void Schedule::runMonitor(){
    using std::chrono::milliseconds;
    Logger::scheduleLogger()->info("===== 监控线程启动 =====");
    
    // 模块分隔符（用于格式化输出）
    const std::string MODULE_SEP = "----------------------------------------";
    // 记录监控线程启动时间
    const auto start_time = std::chrono::steady_clock::now();
    
    // 监控主循环：持续运行直到系统停止
    while (gRunning_.load(std::memory_order_acquire)) {
        // 获取当前时间戳
        const auto now = std::chrono::steady_clock::now();
        const uint64_t now_ms = (uint64_t)std::chrono::duration_cast<milliseconds>(
                                now.time_since_epoch()).count();
        
        // 格式化当前时间（年月日 时分秒:毫秒）
        auto now_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm tm = *std::localtime(&now_t);
        char time_buf[32];
        std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &tm);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch() % std::chrono::seconds(1)
        ).count();
        std::string timestamp = fmt::format("{}:{:03d}", time_buf, ms);
        
        // 计算系统运行时间（时分秒.毫秒）
        auto run_duration = now - start_time;
        auto run_hours = std::chrono::duration_cast<std::chrono::hours>(run_duration);
        run_duration -= run_hours;
        auto run_mins = std::chrono::duration_cast<std::chrono::minutes>(run_duration);
        run_duration -= run_mins;
        auto run_secs = std::chrono::duration_cast<std::chrono::seconds>(run_duration);
        run_duration -= run_secs;
        auto run_ms = std::chrono::duration_cast<std::chrono::milliseconds>(run_duration);
        
        // 构建监控状态信息字符串
        statusStr_ = fmt::format("===== 监控快照 [{}] =====\n", timestamp);
        statusStr_ += fmt::format("{}\n", MODULE_SEP);
        statusStr_ += fmt::format("[运行时间]: {:02d}:{:02d}:{:02d}.{:03d}\n",
            run_hours.count(), run_mins.count(), run_secs.count(), run_ms.count());
        
        // ---------- 1. LiDAR接收模块监控 ----------
        try {
            uint64_t total   = lidarReceiver_.total_packets.load();          // 总数据包数
            const uint64_t full    = lidarReceiver_.full_packets.load();     // 完整数据包数
            const uint64_t partial = lidarReceiver_.partial_packets.load();  // 不完整数据包数
            const double full_ratio = total? (double)full / (double)(full+partial) : 0.0; // 完整率

            // 计算各类数据包处理速率
            double total_rate   = calcRate(lidar_recv_total_rate_,   total,   now_ms);
            double full_rate    = calcRate(lidar_recv_full_rate_,    full,    now_ms);
            double partial_rate = calcRate(lidar_recv_partial_rate_, partial, now_ms);

            // 拼接监控信息
            statusStr_ += fmt::format("{}\n", MODULE_SEP);
            statusStr_ += fmt::format("[LiDAR接收] 总数据包:      {} (速率: {:.1f}/s)\n",
                total, total_rate);
            statusStr_ += fmt::format("[LiDAR接收] 正在处理数据包: {}\n", (full > 0)? 1 : 0);
            statusStr_ += fmt::format("[LiDAR接收] 完整数据包:    {} (速率: {:.1f}/s)\n",
                full, full_rate);
            statusStr_ += fmt::format("[LiDAR接收] 不完整数据包:  {} (速率: {:.1f}/s)\n",
                partial, partial_rate);
            statusStr_ += fmt::format("[LiDAR接收] 完整率:        {:.2f}%\n", full_ratio * 100);
        } catch (...) {
            // 捕获所有异常，避免监控线程崩溃
        }

        // ---------- 2. LiDAR解析模块监控 ----------
        try {
            uint64_t total_packets = lidarParser_.total_packets.load();  // 累计处理数据包数
            double parse_rate = calcRate(lidar_parse_total_rate_, total_packets, now_ms); // 解析速率

            // 获取解析耗时和每包点数的统计信息
            auto s = lidarParser_.parse_time_ms.snapshot();    // 解析耗时统计（均值、标准差）
            auto p = lidarParser_.points_per_packet.snapshot();// 每包点数统计

            // 拼接监控信息
            statusStr_ += fmt::format("{}\n", MODULE_SEP);
            statusStr_ += fmt::format("[LiDAR解析] 处理数据包:    {} (速率: {:.1f}/s)\n",
                total_packets, parse_rate);
            statusStr_ += fmt::format("[LiDAR解析] 平均耗时:      {:.3f}ms (样本数: {}, 标准差: {:.3f})\n",
                 s.mean, s.n, s.stddev);
            statusStr_ += fmt::format("[LiDAR解析] 平均点数:      {:.1f} (样本数: {}, 标准差: {:.1f})\n",
                 p.mean, p.n, p.stddev);
        } catch (...) {}

        // ---------- 3. POS解析模块监控 ----------
        try {
            const uint64_t pos_total  = posParser_.pos_frames.load();          // POS帧总数
            const uint64_t hik_mark_total = posParser_.hik_mark_frames.load(); // Hik-MARK帧总数
            const uint64_t ph_mark_total = posParser_.ph_mark_frames.load();   // Pha-MARK帧总数
            
            // 计算各类帧的处理速率
            double pos_rate  = calcRate(pos_frame_rate_,  pos_total,  now_ms);
            double hik_mark_rate = calcRate(hik_mark_frame_rate_, hik_mark_total, now_ms);
            double ph_mark_rate = calcRate(ph_mark_frame_rate_, ph_mark_total, now_ms);

            // 拼接监控信息
            statusStr_ += fmt::format("{}\n", MODULE_SEP);
            statusStr_ += fmt::format("[POS接收]   POS帧:         {} (速率: {:.1f}/s)\n",
                 pos_total, pos_rate);
            statusStr_ += fmt::format("[POS接收]   Hik-MARK帧:    {} (速率: {:.1f}/s)\n",
                 hik_mark_total, hik_mark_rate);
            statusStr_ += fmt::format("[POS接收]   Pha-MARK帧:    {}（速率: {:.1f}/s)\n",
                 ph_mark_total, ph_mark_rate);
        } catch (...) {}

        // ---------- 4. 点云配准模块监控 ----------
        try {
            uint64_t total_packets = registrar_.total_packets.load();  // 累计处理数据包数
            double reg_rate = calcRate(reg_total_rate_, total_packets, now_ms); // 配准速率

            // 获取配准耗时统计信息
            auto s = registrar_.register_ts_ms.snapshot();
            
            // 拼接监控信息
            statusStr_ += fmt::format("{}\n", MODULE_SEP);
            statusStr_ += fmt::format("[点云配准] 处理数据包:    {} (速率: {:.1f}/s)\n",
                     total_packets, reg_rate);
            statusStr_ += fmt::format("[点云配准] 平均耗时:      {:.3f}ms (样本数: {}, 标准差: {:.3f})\n",
                     s.mean, s.n, s.stddev);
        } catch (...) {}
        
        // ---------- 5. LiDAR写入模块监控 ----------
        try {
            uint64_t total_packets = lidarWriter_.total_packets.load();  // 累计写入数据包数
            double wrt_rate = calcRate(wrt_total_rate_, total_packets, now_ms); // 写入速率
            
            // 获取写入耗时统计信息
            auto ws = lidarWriter_.write_time_ms.snapshot();

            // 拼接监控信息
            statusStr_ += fmt::format("{}\n", MODULE_SEP);
            statusStr_ += fmt::format("[点云写入] 处理数据包:    {} (速率: {:.1f}/s)\n",
                    total_packets, wrt_rate);
            statusStr_ += fmt::format("[点云写入] 平均耗时:      {:.3f}ms (样本数: {}, 标准差: {:.3f})\n",
                    ws.mean, ws.n, ws.stddev);
        } catch (...) {}

        // ---------- 6. 数据传输模块监控 ----------
        try {
            uint64_t tx_enqueued    = dataTransmitter_.tx_enqueued_.load();    // 累计入队次数
            uint64_t tx_attempts    = dataTransmitter_.tx_attempts_.load();    // 累计尝试传输次数
            uint64_t tx_success     = dataTransmitter_.tx_success_.load();     // 累计成功传输次数
            uint64_t tx_failed      = dataTransmitter_.tx_failed_.load();      // 累计失败传输次数
            uint64_t tx_bytes       = dataTransmitter_.tx_bytes_.load();       // 累计传输字节数
            uint64_t tx_q_depth     = dataTransmitter_.queueDepth();           // 当前队列深度
            
            // 计算传输速率
            double tx_attempt_rate = calcRate(tx_attempt_rate_, tx_attempts, now_ms);
            double tx_success_rate = calcRate(tx_success_rate_, tx_success, now_ms);
            double tx_byte_rate    = calcRate(tx_bytes_rate_, tx_bytes, now_ms);
            double tx_mbps = (tx_byte_rate > 0.0)? (tx_byte_rate / (1024.0 * 1024.0)) : 0.0; // 转换为MB/s

            // 拼接监控信息
            statusStr_ += fmt::format("{}\n", MODULE_SEP);
            statusStr_ += fmt::format("[传输]     累计传输次数:  {}\n", tx_enqueued);
            statusStr_ += fmt::format("[传输]     等待传输次数:  {}\n", tx_q_depth);
            statusStr_ += fmt::format("[传输]     尝试传输次数:  {} (速率: {:.1f}/s)\n", tx_attempts, tx_attempt_rate);
            statusStr_ += fmt::format("[传输]     成功传输次数:  {} (速率: {:.1f}/s)\n", tx_success, tx_success_rate);
            statusStr_ += fmt::format("[传输]     失败传输次数:  {}\n", tx_failed);
            statusStr_ += fmt::format("[传输]     累计传输数据:  {:.2f} MB\n", tx_bytes / (1024.0 * 1024.0));
            statusStr_ += fmt::format("[传输]     实时传输速率:  {:.2f} MB/s\n", tx_mbps);
        } catch (...) {}


        
        // ---------- 8. 队列状态监控 ----------
        try{
            // 获取各队列当前大小
            auto [tmin, tmax] = posRing_.time_range();  // POS环形队列时间范围
            double tmin_sec = tmin / 1000.0;
            double tmax_sec = tmax / 1000.0;
            
            // 拼接队列状态信息
            statusStr_ += fmt::format("{}\n", MODULE_SEP);
            statusStr_ += fmt::format("[队列状态] 雷达数据接收队列:    {}个元素\n",  lidarReceiveQueue_.size_approx());
            statusStr_ += fmt::format("[队列状态] 雷达数据解析队列:    {}个元素\n",  lidarParseQueue_.size_approx());
            statusStr_ += fmt::format("[队列状态] POS环形队列:       {}个元素 (容量: {})\n",
                    posRing_.size(), posRing_.capacity());
            statusStr_ += fmt::format("[队列状态] POS时间窗:      [{:.3f}, {:.3f}]秒\n",
                    tmin_sec, tmax_sec);
            statusStr_ += fmt::format("[队列状态] Mark队列:     {}个元素\n",  markQueue_.size_approx());
            statusStr_ += fmt::format("[队列状态] 待保存点云队列:    {}个元素\n", writeQueue_.size_approx());
            statusStr_ += fmt::format("[队列状态] 待着色队列:    {}个元素\n", colorQueue_.size_approx());
        } catch (...) {}

        // ---------- 9. 异常告警信息 ----------
        try{
            // 获取各类异常计数
            uint64_t non_mono_lidar_count = lidarParser_.lidar_non_mono.load(std::memory_order_relaxed); // LiDAR时间非递增
            uint64_t non_mono_pos_count = posParser_.pos_non_mono.load(std::memory_order_relaxed);       // POS时间非递增
            uint64_t lidar_reg_early = registrar_.reg_early.load(std::memory_order_relaxed);             // LiDAR时间超前
            uint64_t lidar_reg_late  = registrar_.reg_late.load(std::memory_order_relaxed);              // LiDAR时间滞后
            uint64_t lidar_reg_fail  = registrar_.reg_fail.load(std::memory_order_relaxed);              // 配准失败

            // 检查是否有新的异常发生
            if (prev_non_mono_lidar_count_ != non_mono_lidar_count ||
                prev_non_mono_pos_count_ != non_mono_pos_count ||
                prev_lidar_reg_early_count_ != lidar_reg_early ||
                prev_lidar_reg_late_count_ != lidar_reg_late ||
                prev_lidar_reg_fail_count_ != lidar_reg_fail) {
                warnError_ = true;  // 有新异常，设置警告标志
            } else {
                warnError_ = false; // 无新异常，清除警告标志
            }

            // 更新异常计数基准值
            prev_non_mono_lidar_count_ = non_mono_lidar_count;
            prev_non_mono_pos_count_ = non_mono_pos_count;
            prev_lidar_reg_early_count_ = lidar_reg_early;
            prev_lidar_reg_late_count_ = lidar_reg_late;
            prev_lidar_reg_fail_count_ = lidar_reg_fail;

            // 拼接告警信息
            statusStr_ += fmt::format("{}\n", MODULE_SEP);
            statusStr_ += fmt::format("[告警信息]-[LiDAR解析] 入队数据不满足递增情况:           {}例\n", non_mono_lidar_count);
            statusStr_ += fmt::format("[告警信息]-[POS解析]   入队数据不满足递增情况:           {}例\n", non_mono_pos_count);
            statusStr_ += fmt::format("[告警信息]-[点云配准]   点云配准时雷达时间超前POS时间窗:   {}例\n", lidar_reg_early);
            statusStr_ += fmt::format("[告警信息]-[点云配准]   点云配准时雷达时间滞后POS时间窗:   {}例\n", lidar_reg_late);
            statusStr_ += fmt::format("[告警信息]-[点云配准]   点云配准失败:                   {}例\n", lidar_reg_fail);
            statusStr_ += fmt::format("{}\n", MODULE_SEP);
            
            // 添加系统状态总结
            if (warnError_) {
                statusStr_ += "状态：警告（请检查告警信息）\n";
            } else {
                statusStr_ += "状态：正常\n";
            }
            
            statusStr_ += fmt::format("{}\n", MODULE_SEP);
        } catch (...) {}
        
        // 发送状态消息（可根据需要启用）
        // SendStatusMessage(statusStr_);
        
        // 输出监控信息到日志
        Logger::scheduleLogger()->info(statusStr_);
        
        // 监控周期休眠（默认配置的监控间隔）
        std::this_thread::sleep_for(std::chrono::milliseconds(monitor_period_ms_));
    }
    
    Logger::scheduleLogger()->info("===== 监控线程退出 =====");
}

// ---- Trampoline 实现（线程入口包装函数，用于异常捕获）----
/**
 * @brief POS接收线程包装函数
 * @details 捕获POS接收模块的所有异常，记录错误日志并停止系统
 */
void Schedule::RunTrampoline_PosReceive(){
    try { posReceiver_.run(); }
    catch (const std::exception& e){ 
        Logger::scheduleLogger()->error("POS接收崩溃: {}", e.what()); 
        fatalError_ = true; 
        gRunning_.store(false); 
    }
    catch (...) { 
        Logger::scheduleLogger()->error("POS接收崩溃: 未知错误"); 
        gRunning_.store(false);
    } 
}

/**
 * @brief POS解析线程包装函数
 * @details 捕获POS解析模块的所有异常，记录错误日志并停止系统
 */
void Schedule::RunTrampoline_PosParse(){
    try { posParser_.run(); }
    catch (const std::exception& e){ 
        Logger::scheduleLogger()->error("POS解析崩溃: {}", e.what()); 
        fatalError_ = true; 
        gRunning_.store(false); 
    }
    catch (...) { 
        Logger::scheduleLogger()->error("POS解析崩溃: 未知错误"); 
        gRunning_.store(false);
    } 
}

/**
 * @brief POS写入线程包装函数
 * @details 捕获POS写入模块的所有异常，记录错误日志并停止系统
 */
void Schedule::RunTrampoline_PosWriter(){
    try { posWriter_.run(); }
    catch (const std::exception& e){ 
        Logger::scheduleLogger()->error("POS写入崩溃: {}", e.what()); 
        fatalError_ = true; 
        gRunning_.store(false); 
    }
    catch (...) { 
        Logger::scheduleLogger()->error("POS写入崩溃: 未知错误"); 
        gRunning_.store(false);
    } 
}

/**
 * @brief LiDAR接收线程包装函数
 * @details 捕获LiDAR接收模块的所有异常，记录错误日志并停止系统
 */
void Schedule::RunTrampoline_LidarReceive(){
    try { lidarReceiver_.run(); }
    catch (const std::exception& e){ 
        Logger::scheduleLogger()->error("LiDAR接收崩溃: {}", e.what()); 
        fatalError_ = true; 
        gRunning_.store(false); 
    }
    catch (...) { 
        Logger::scheduleLogger()->error("LiDAR接收崩溃: 未知错误"); 
        gRunning_.store(false);
    } 
}

/**
 * @brief LiDAR解析线程包装函数
 * @details 捕获LiDAR解析模块的所有异常，记录错误日志并停止系统
 */
void Schedule::RunTrampoline_LidarParse(){
    try { lidarParser_.run(); }
    catch (const std::exception& e){ 
        Logger::scheduleLogger()->error("LiDAR解析崩溃: {}", e.what()); 
        fatalError_ = true; 
        gRunning_.store(false); 
    }
    catch (...) { 
        Logger::scheduleLogger()->error("LiDAR解析崩溃: 未知错误"); 
        gRunning_.store(false);
    } 
}

/**
 * @brief LiDAR写入线程包装函数
 * @details 捕获LiDAR写入模块的所有异常，记录错误日志并停止系统
 */
void Schedule::RunTrampoline_LidarWriter(){
    try { lidarWriter_.run(); }
    catch (const std::exception& e){ 
        Logger::scheduleLogger()->error("写入崩溃: {}", e.what()); 
        fatalError_ = true; 
        gRunning_.store(false); 
    }
    catch (...) { 
        Logger::scheduleLogger()->error("写入崩溃: 未知错误"); 
        gRunning_.store(false);
    } 
}

/**
 * @brief 点云配准线程包装函数
 * @details 捕获点云配准模块的所有异常，记录错误日志并停止系统
 */
void Schedule::RunTrampoline_Register(){
    try { registrar_.run(); }
    catch (const std::exception& e){ 
        Logger::scheduleLogger()->error("配准/注册崩溃: {}", e.what()); 
        fatalError_ = true; 
        gRunning_.store(false); 
    }
    catch (...) { 
        Logger::scheduleLogger()->error("配准/注册崩溃: 未知错误"); 
        gRunning_.store(false);
    } 
}

/**
 * @brief 数据传输线程包装函数
 * @details 捕获数据传输模块的所有异常，记录错误日志并停止系统
 */
void Schedule::RunTrampoline_DataTransmitter(){
    try { dataTransmitter_.run(); }
    catch (const std::exception& e){ 
        Logger::scheduleLogger()->error("数据传输崩溃: {}", e.what()); 
        fatalError_ = true; 
        gRunning_.store(false); 
    }
    catch (...) { 
        Logger::scheduleLogger()->error("数据传输崩溃: 未知错误"); 
        gRunning_.store(false);
    } 
}



/**
 * @brief 线程安全的join辅助函数
 * @param t 要join的线程对象
 * @details 检查线程是否可join，避免join已结束的线程导致异常
 */
void Schedule::JoinIfJoinable(std::thread& t){ 
    if (t.joinable()) { 
        try { t.join(); } 
        catch (...) {} 
    } 
}
