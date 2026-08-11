#pragma once
#include <cstdint>   // 用于 uint32_t 等类型
#include <cmath>     // 用于 floor/round
#include <chrono>
#include <mutex>     // 用于 std::mutex
#include <atomic>    // 用于 std::atomic
#include <vector>    // 用于 std::vector
#include <string>    // 用于 std::string
#include <Eigen/Dense>
#include <iostream>

#include "type.h"    // 包含自定义类型定义
#include "projection.h"

// 时间工具命名空间：声明常量、枚举和函数
namespace TimeUtils {  
    // GPS与Unix时间戳的核心常量（声明+定义，constexpr可在头文件直接定义）
    constexpr int64_t GPS_EPOCH = 315964800000LL; // GPS起始时间对应的Linux时间戳（1980-01-06 00:00:00）
    constexpr int64_t GPS_LEAP_SECONDS = 18;      // 截至2025年7月的GPS闰秒数

    // 时间单位枚举（声明+定义，枚举可在头文件直接定义）
    enum class TimeUnit {
        Seconds,      // 秒
        Milliseconds  // 毫秒
    };

    /**
     * @brief GPS时间转Unix时间戳
     * @param gpsWeek GPS周数
     * @param timeInWeek 周内时间（单位由timeUnit指定）
     * @param timeUnit 时间单位（Seconds/Milliseconds）
     * @return 转换后的Unix时间戳（int64_t 毫秒）
     */
    int64_t gpsTimeToLinuxTimestamp(int gpsWeek, uint32_t timeInWeek, TimeUnit timeUnit);

    /**
     * @brief Unix时间戳转GPS时间（周数+周内时间）
     * @param linuxTimestamp 待转换的Unix时间戳（int64_t 毫秒）
     * @param gpsWeek [输出] 转换后的GPS周数
     * @param timeInWeek [输出] 转换后的周内时间（单位由timeUnit指定）
     * @param timeUnit 目标时间单位（Seconds/Milliseconds）
     */
    void linuxTimestampToGpsTime(int64_t linuxTimestamp, int& gpsWeek, uint32_t& timeInWeek, TimeUnit timeUnit);

    std::string get_yymmdd_hhmmss();
}


// 校验和工具命名空间：声明常量和函数
namespace Checksum {
    // CRC32多项式常量（constexpr可在头文件直接定义）
    static constexpr uint32_t CRC32_POLYNOMIAL = 0xEDB88320L;

    /**
     * @brief 获取CRC32查表（内部使用，静态函数避免外部调用）
     * @return CRC32查表的指针（全局唯一，第一次调用时生成）
     */
    inline const uint32_t* get_crc32_table();

    /**
     * @brief 计算数据块的CRC32校验和（与发送方保持一致）
     * @param ulCount 数据长度（字节数）
     * @param ucBuffer 数据缓冲区指针（非空）
     * @return 计算出的CRC32校验和
     */
    uint32_t CalculateBlockCRC32(uint32_t ulCount, const uint8_t* ucBuffer);
}


// -------------------- 统计工具 --------------------
namespace Telemetry {
    struct Welford {
        void add(double x) {
            std::lock_guard<std::mutex> lk(mu_);
            ++n_; double delta = x - mean_; mean_ += delta / n_; m2_ += delta * (x - mean_);
        }
        struct Snap { uint64_t n=0; double mean=0, var=0, stddev=0; };
        Snap snapshot() const {
            std::lock_guard<std::mutex> lk(mu_);
            Snap s; s.n = n_; s.mean = mean_; s.var = (n_>1? m2_/(n_-1):0); s.stddev = (s.var>0? std::sqrt(s.var):0); return s;
        }
    private:
        mutable std::mutex mu_;
        uint64_t n_ = 0; double mean_ = 0.0, m2_ = 0.0;
    };

    struct ScopedTimerMs {
        explicit ScopedTimerMs(Welford& w): w_(w), t0_(std::chrono::steady_clock::now()) {}
        ~ScopedTimerMs(){
            using namespace std::chrono;
            auto dt_ms = duration_cast<microseconds>(steady_clock::now() - t0_).count() / 1000.0; //double ms
            // std::cout << "ScopedTimerMs dt_ms=" << dt_ms << "ms\n";
            w_.add(dt_ms);
        }
    private:
        Welford& w_;
        std::chrono::steady_clock::time_point t0_;
    };

    struct WarningCounters {
        std::atomic<uint64_t> pos_ts_non_monotonic{0};
        std::atomic<uint64_t> lidar_ts_non_monotonic{0};
        std::atomic<uint64_t> merger_time_mismatch{0};
    };
} // namespace schedule_telemetry

namespace Save {
    void writeVector3dToPCDManual(const std::vector<Eigen::Vector3d>& curSecData, const std::string& filename);
}

// -------------------- Projection --------------------
namespace Proj {
    ROI computeROI(double lat0_deg, double lon0_deg, double H=3000.0, double FOV_deg=60.0);

    bool is_intersect(const ROI& a, const ROI& b);
}