// pos_ring.h
#pragma once
#include <atomic>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <cassert>
#include <limits>

#include "Logger.h"

// 单写多读无锁环形缓冲区（时间戳+值对）
template<typename T>
class PosRing {
public:
    struct Snapshot {
        const int64_t* ts;   // 指向内部时间戳数组（只读）
        const T*       val;  // 指向内部值数组（只读）
        size_t         cap_mask;
        uint64_t       r;    // 逻辑起
        uint64_t       w;    // 逻辑止（半开）
    };

    explicit PosRing(size_t capacity_pow2 = 1 << 20)
        : cap_(round_up_pow2(capacity_pow2)),
          ts_(cap_), val_(cap_), read_idx_(0), write_idx_(0) {}

    // 单写线程调用；要求 ts 单调递增；满时返回 false（由上层决定扩容/丢弃/回收后重试）
    bool push(int64_t ts_ms, const T& v) {
        const auto w = write_idx_.load(std::memory_order_relaxed);
        const auto r = read_idx_.load(std::memory_order_acquire);
        if (w - r >= cap_) return false; // 满
        const size_t i = (size_t)(w & (cap_ - 1));
        ts_[i]  = ts_ms;
        val_[i] = v; // T 可为轻量结构；若大对象可存指针/索引
        write_idx_.store(w + 1, std::memory_order_release);
        return true;
    }

    // A. 强约束，严格按照时间窗
    // 写入前先清窗口外，再写。若仍因容量不够失败，可选择“丢最老”以保证可写。
    template<class U>
    bool push_window(int64_t ts_ms, U&& v, int64_t window_ms,
                     bool drop_oldest_on_full=true) {
        reclaim_until(ts_ms - window_ms); // 先清窗外
        T tmp(std::forward<U>(v)); // 可重试副本
        if (push(ts_ms, tmp)) return true; // 成功

        // 兜底：容量不足或回收不及时
        if (!drop_oldest_on_full) return false;  // 1. 
        auto r = read_idx_.load(std::memory_order_relaxed); 
        auto w = write_idx_.load(std::memory_order_acquire);
        if (w > r) read_idx_.store(r + 1, std::memory_order_relaxed); // 丢最老 
        return push(ts_ms, std::move(tmp)); // 再试
    }

    // B. 省CPU：事件驱动 + 节流
    struct GCController {
        int64_t window_ms = 20'000; // 时间窗
        int64_t gc_step_ms = 200;
        int64_t next_gc_ms = std::numeric_limits<int64_t>::min();
    };

    template<class U>
    bool push_with_gc(int64_t ts_ms, U&& v, GCController& gc) {
        T tmp(std::forward<U>(v)); // 可重试副本
        // 先尝试写入
        if (!push(ts_ms, tmp)) {
            reclaim_until(ts_ms - gc.window_ms); // 先清窗外
            if (!push(ts_ms, tmp)) return false; // 再试
        }

        // 定时清理
        if (ts_ms >= gc.next_gc_ms) {
            reclaim_until(ts_ms - gc.window_ms);
            gc.next_gc_ms = ts_ms + gc.gc_step_ms;
        }
        return true;
    }


    // 读者快照（多读无锁）
    Snapshot snapshot() const {
        Snapshot s;
        s.cap_mask = cap_ - 1;
        s.r = read_idx_.load(std::memory_order_acquire);
        s.w = write_idx_.load(std::memory_order_acquire);
        s.ts = ts_.data();
        s.val = val_.data();
        return s;
    }

    // 在快照上找 pair(prev_idx, next_idx)，若不存在返回 -1
    std::pair<int64_t, int64_t> lower_pair(const Snapshot& s, int64_t t_ms) const {
        if (s.w <= s.r) return { -1, -1 };
        const uint64_t N = s.w - s.r;

        // 二分在虚拟连续区间 [0, N)
        uint64_t lo = 0, hi = N;
        while (lo < hi) {
            uint64_t mid = (lo + hi) >> 1;
            int64_t v = s.ts[(size_t)((s.r + mid) & s.cap_mask)];
            if (v < t_ms) lo = mid + 1;
            else          hi = mid;
        }
        int64_t nextI = -1, prevI = -1;
        if (lo < N)      nextI = (int64_t)((s.r + lo) & s.cap_mask);
        if (lo > 0)      prevI = (int64_t)((s.r + lo - 1) & s.cap_mask);
        return { prevI, nextI };
    }

    // 回收：前移 read_idx 直到 ts >= watermark_ms
    void reclaim_until(int64_t watermark_ms) {
        auto r = read_idx_.load(std::memory_order_relaxed);
        // Logger::sendLogger()->debug("POS环状缓冲区回收至时间戳为 >= {} 毫秒的元素", watermark_ms);
        // Logger::sendLogger()->debug("当前缓冲区最后一个元素时间戳== {} 毫秒", ts_[(size_t)(r & (cap_-1))]);
        // Logger::sendLogger()->debug("当前缓冲区读索引 r == {}, 写索引 w == {}, 元素个数 == {}", r, write_idx_.load(std::memory_order_acquire), size());
        const auto w = write_idx_.load(std::memory_order_acquire);
        while (r < w) {
            const size_t i = (size_t)(r & (cap_ - 1));
            if (ts_[i] >= watermark_ms) break;
            ++r;
        }
        read_idx_.store(r, std::memory_order_release);
    }

    // 可选：当前元素个数
    uint64_t size() const {
        const auto r = read_idx_.load(std::memory_order_acquire);
        const auto w = write_idx_.load(std::memory_order_acquire);
        return w - r;
    }

    // 可选：返回环形缓冲区容量（固定值）
    size_t capacity() const {
        return cap_;
    }

    // 可选：返回当前缓冲区的时间范围 [最早时间戳， 最新时间戳]
    std::pair<int64_t, int64_t> time_range() const {
        const auto r = read_idx_.load(std::memory_order_acquire);
        const auto w = write_idx_.load(std::memory_order_acquire);
        if (w <= r) return {0, 0};
        const int64_t tmin = ts_[(size_t)(r & (cap_ - 1))];
        const int64_t tmax = ts_[(size_t)((w - 1) & (cap_ - 1))];
        return {tmin, tmax};
    }

private:
    static size_t round_up_pow2(size_t x) {
        size_t p = 1; while (p < x) p <<= 1; return p;
    }

    const size_t cap_;
    std::vector<int64_t> ts_;
    std::vector<T>       val_;
    std::atomic<uint64_t> read_idx_;
    std::atomic<uint64_t> write_idx_;
};