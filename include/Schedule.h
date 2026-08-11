#pragma once

#include "type.h"
#include "PosRing.h" // 仅作接口使用，不合并实现
#include "POS.h"
#include "LiDAR.h"
#include "Logger.h"
#include "Register.h"
#include "moodycamel/concurrentqueue.h"
#include "Transmit.h"

#include <atomic>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <functional>
#include <chrono>
#include <cstdint>
#include <cmath>

// -------------------- 调度器 --------------------
class Schedule {
public:
    explicit Schedule(std::atomic<bool>& running);
    ~Schedule();

    // 一键启停
    void startAll();
    void stopAll();

    // 分段启动（与现有接口保持一致）
    void startPosReceive();
    void startPosParse();
    void startPosWriter();
    void startLidarReceive();
    void startLidarParse();
    void startLidarWriter();
    void startRegister();
    void startDataTransmitter();

    // 监控线程
    void startMonitor();

    // 非拷贝/非移动
    Schedule(const Schedule&) = delete;
    Schedule& operator=(const Schedule&) = delete;
    Schedule(Schedule&&) = delete;
    Schedule& operator=(Schedule&&) = delete;

private:
    // === 监控实现（由原 Monitor.cpp/.h 收拢而来） ===
    void runMonitor();

    // 速率计算缓存
    struct RateSnap { uint64_t last=0; uint64_t last_ms=0; double rate=0; };
    static double calcRate(RateSnap& r, uint64_t now_total, uint64_t now_ms);

    // Trampolines（捕获异常，避免崩溃）
    void RunTrampoline_PosReceive();
    void RunTrampoline_PosParse();
    void RunTrampoline_PosWriter();
    void RunTrampoline_LidarReceive();
    void RunTrampoline_LidarParse();
    void RunTrampoline_LidarWriter();
    void RunTrampoline_Register();
    void RunTrampoline_DataTransmitter();

    static void JoinIfJoinable(std::thread& t);

private:
    // === 全局运行标志 ===
    std::atomic<bool>& gRunning_;

    // === 队列与环 ===
    // LiDAR：接收 -> 解析 -> 写入
    moodycamel::ConcurrentQueue<std::shared_ptr<PacketData>> lidarReceiveQueue_;
    moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>> lidarParseQueue_;
    moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>> writeQueue_;
    moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>> colorQueue_;

    // POS：原始字节 -> 解析 -> 1.环 2.mark队列
    moodycamel::ConcurrentQueue<uint8_t> posBufferQueue_;
    moodycamel::ConcurrentQueue<uint8_t> posWriterQueue_; 
    PosRing<std::shared_ptr<POSData>> posRing_; 
    moodycamel::ConcurrentQueue<std::shared_ptr<MarkData>> markQueue_;
    


    // === 模块实例 ===
    LiDARReceiver  lidarReceiver_;
    LiDARParser    lidarParser_;
    LiDARWriter    lidarWriter_;
    POSReceiver    posReceiver_;
    POSParser      posParser_;
    POSWriter      posWriter_;
    CloudRegister  registrar_;
    DataTransmitter dataTransmitter_;


    // === 工作线程 ===
    std::thread tPosRecv_;
    std::thread tPosParse_;
    std::thread tPosWriter_;
    std::thread tLidarRecv_;
    std::thread tLidarParse_;
    std::thread tLidarWriter_;
    std::thread tRegister_;
    std::thread tDataTransmitter_;
    std::thread tMonitor_;

    // === 监控内部状态 ===
    int monitor_period_ms_ = 5000; // 每5秒输出一次
    std::string statusStr_; 
    bool fatalError_ = false; // 出现致命错误，停止运行
    bool warnError_ = false;  // 出现警告错误，继续运行
    RateSnap lidar_recv_total_rate_, lidar_recv_full_rate_, lidar_recv_partial_rate_;  // LiDARReceiver
    RateSnap lidar_parse_total_rate_; // LiDARParser
    RateSnap pos_frame_rate_, hik_mark_frame_rate_, ph_mark_frame_rate_; // POSParser
    RateSnap reg_total_rate_; // LiDARRegister
    RateSnap wrt_total_rate_; // LIDARWriter
    RateSnap tx_attempt_rate_, tx_success_rate_, tx_bytes_rate_; // DataTransmitter

    uint64_t prev_non_mono_lidar_count_ = 0;
    uint64_t prev_non_mono_pos_count_ = 0;
    uint64_t prev_lidar_reg_early_count_ = 0;
    uint64_t prev_lidar_reg_late_count_ = 0;
    uint64_t prev_lidar_reg_fail_count_ = 0;
};



