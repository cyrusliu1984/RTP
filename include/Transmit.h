#pragma once
#include <atomic>
#include <cstdint>
#include <queue>
#include <map>
#include <string>
#include <vector>
#include <filesystem>

#include "type.h"

// ===== 自由函数：类外可直接调用 =====
bool SendStatusMessage(const std::string& message);                         // 用缺省端点
bool SendStatusMessageTo(const std::string& ip, uint16_t port,
                         const std::string& message);                       // 指定端点
bool SendLazFileTo(const std::string& filepath, const std::string& ip,
                   uint16_t port);                                         // 指定端点

class DataTransmitter {
public:
    // monitorDir 示例: "../data/LAZ"
    DataTransmitter(const std::atomic<bool>& running);

    ~DataTransmitter();
    void run();

    // 供其它模块查询当前缺省端点（可选）
    static std::string DefaultIp();
    static uint16_t    DefaultPort();

    static bool isLazPath(const std::filesystem::path& p);

    // Schedule Monitor 访问
    std::atomic<uint64_t> tx_enqueued_{0};
    std::atomic<uint64_t> tx_queue_depth_{0};
    std::atomic<uint64_t> tx_attempts_{0};
    std::atomic<uint64_t> tx_success_{0};
    std::atomic<uint64_t> tx_failed_{0};
    std::atomic<uint64_t> tx_bytes_{0};
    inline uint64_t queueDepth() const {
        int64_t d = tx_queue_depth_.load(std::memory_order_relaxed);
        return d > 0 ? static_cast<uint64_t>(d) : 0ULL;
    }
    

private:
    const std::atomic<bool>& gRunning_;
    std::string monitorDir_;
    std::queue<std::string> fileQueue_;
    std::map<std::string, FileState> fileStates_;

    static bool pathExistsReadableNonEmpty(const std::string& path);

    // === 缺省端点（线程安全） ===
    static std::atomic<uint32_t> s_ip_be_; // IP 的网络序 32-bit
    static std::atomic<uint16_t> s_port_;  // 端口
    static void setDefaultEndpoint(const std::string& ip, uint16_t port);
};