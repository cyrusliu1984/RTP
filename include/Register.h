#pragma once
#include "type.h"
#include "utils.h"
#include <PosRing.h>
#include "projection.h"
#include "moodycamel/concurrentqueue.h"
#include "config_manager.h"

#include <vector>
#include <cstdint>
#include <atomic>
#include <thread>
#include <algorithm>


struct MatchPolicy {
    int64_t max_nn_error_ms = 3; // 超过则标记/可丢弃（此处仍输出）
    bool    prefer_next_when_tie = true;
    int     wait_next_ms = 50;     // 批任务内是否短等候“next”可见（0 = 不等）
    size_t  workers = 1; //std::max(1u, std::thread::hardware_concurrency());
};

class CloudRegister {
public:
    CloudRegister(PosRing<std::shared_ptr<POSData>>& posRing,
                moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& lidarQ,
                moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& colorQ,
                moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& writeQ,
                const MatchPolicy& policy,
                const std::atomic<bool>& running);

    void run(); // 阻塞式主循环

    std::atomic<uint64_t> total_packets{0}; // 点云配组数量 (Schedule Monitor)
    Telemetry::Welford register_ts_ms; // 点云配准耗时 (Schedule Monitor)

    std::atomic<uint64_t> reg_early{0}; // 雷达时间超前累计次数 （一般是刚启动的情况，雷达时间小于POS时间窗下界）
    std::atomic<uint64_t> reg_late{0};  // 雷达时间滞后累计次数 （一般是POS数据缺失，雷达时间大于POS时间窗上界）
    std::atomic<uint64_t> reg_fail{0};  // 配准失败累计次数 （POS时间窗内无匹配）
    EventRing<64>         reg_events; // 最近16条样例

private:
    PosRing<std::shared_ptr<POSData>>& pos_;
    moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& lidarQ_;
    moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& colorQ_;
    moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& writeQ_;
    MatchPolicy policy_;
    const std::atomic<bool>& running_;

    // roi相关
    bool roi_enabled_ = false;
    bool in_roi_ = false;
    ROI roi_target_;
    ROI roi_current_;
    double altitude_;
    double speed_;

    inline void recordEarly(int64_t wall_ms, int64_t win_min, int64_t cloud_ts){
        reg_early.fetch_add(1, std::memory_order_relaxed);
        reg_events.push(EventBasic{wall_ms, EventType::REG_EARLY, win_min, cloud_ts, win_min- cloud_ts, 0});
    }
    inline void recordLate(int64_t wall_ms, int64_t win_max, int64_t cloud_ts){
        reg_late.fetch_add(1, std::memory_order_relaxed);
        reg_events.push(EventBasic{wall_ms, EventType::REG_LATE, win_max, cloud_ts, win_max- cloud_ts, 0});
    }
    inline void recordFail(int64_t wall_ms, int64_t match_ts, int64_t cloud_ts){
        reg_fail.fetch_add(1, std::memory_order_relaxed);
        reg_events.push(EventBasic{wall_ms, EventType::REG_TIMEOUT, match_ts, cloud_ts, match_ts - cloud_ts, 0});
    }

};
