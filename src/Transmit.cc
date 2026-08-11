/**
 * @file Transmit.cpp
 * @brief LAZ文件TCP传输模块核心实现（跨平台适配：Windows & Linux）
 */
#include "Transmit.h"
#include "Logger.h"

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using socket_t = SOCKET;
    #define IS_VALID_SOCKET(s) ((s) != INVALID_SOCKET)
    #define CLOSE_SOCKET(s) ::closesocket(s)
    #define SOCKET_LAST_ERROR WSAGetLastError()
#else
    #include <arpa/inet.h>
    #include <netinet/in.h>
    #include <sys/socket.h>
    #include <sys/stat.h>
    #include <unistd.h>
    #include <cerrno>
    using socket_t = int;
    #define IS_VALID_SOCKET(s) ((s) >= 0)
    #define CLOSE_SOCKET(s) ::close(s)
    #define SOCKET_LAST_ERROR errno
#endif

#include <cstring>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <filesystem>
#include <queue>
#include <unordered_map>
#include <atomic>
#include <thread>

namespace fs = std::filesystem;

// 全局配置（目标IP、端口、监控目录）
namespace{
    std::string dstIp = "192.168.5.120";     // 默认传输目标IP
    uint16_t    tcpPort = 8888;              // 默认传输目标端口
    std::string monitorDir = "../data/LAZ";  // 监控的LAZ文件目录
}

// 自动初始化 WinSock 结构体助手
#ifdef _WIN32
struct WinsockInit {
    WinsockInit() {
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
    }
    ~WinsockInit() {
        WSACleanup();
    }
};
static WinsockInit g_winsockInit;
#endif

// ======== 静态成员定义 ========
std::atomic<uint32_t> DataTransmitter::s_ip_be_{0};   // 目标IP（网络字节序）
std::atomic<uint16_t> DataTransmitter::s_port_{0};    // 目标端口

static std::string beIpToString(uint32_t ip_be) {
    char buf[INET_ADDRSTRLEN] = {0};
    in_addr addr{};
    addr.s_addr = ip_be;
    if (::inet_ntop(AF_INET, &addr, buf, sizeof(buf))) return std::string(buf);
    return {};
}

static uint32_t stringIpToBE(const std::string& ip) {
    in_addr addr{};
    if (::inet_pton(AF_INET, ip.c_str(), &addr) != 1) return 0;
    return addr.s_addr;
}

DataTransmitter::DataTransmitter(const std::atomic<bool>& running)
    : tx_enqueued_(0), tx_queue_depth_(0), tx_attempts_(0), tx_success_(0), tx_failed_(0), tx_bytes_(0)
    , gRunning_(running), monitorDir_(monitorDir)
{
    setDefaultEndpoint(dstIp, tcpPort);
    Logger::transmitLogger()->info("数据传输模块已启动，传输目标 {}:{}, 监听目录 {}",
                                   dstIp, tcpPort, monitorDir_);
}

DataTransmitter::~DataTransmitter() {
    Logger::transmitLogger()->info("数据传输模块已关闭");
}

void DataTransmitter::setDefaultEndpoint(const std::string& ip, uint16_t port) {
    const uint32_t ip_be = stringIpToBE(ip);
    if (ip_be == 0) {
        Logger::transmitLogger()->warn("无效的目的 IP: {}", ip);
        return;
    }
    s_ip_be_.store(ip_be, std::memory_order_release);
    s_port_.store(port, std::memory_order_release);
}

std::string DataTransmitter::DefaultIp() {
    return beIpToString(s_ip_be_.load(std::memory_order_acquire));
}

uint16_t DataTransmitter::DefaultPort() {
    return s_port_.load(std::memory_order_acquire);
}

bool DataTransmitter::isLazPath(const fs::path& p) {
    if (!p.has_extension()) return false;
    auto ext = p.extension().string();
    if (ext.size() != 4) return false;
    return (ext == ".laz" || ext == ".LAZ" || ext == ".Laz" || ext == ".lAz" ||
            ext == ".laZ" || ext == ".lAZ" || ext == ".LaZ" || ext == ".LAz");
}

bool DataTransmitter::pathExistsReadableNonEmpty(const std::string& path) {
    std::error_code ec;
    if (!fs::exists(path, ec) || !fs::is_regular_file(path, ec)) return false;
    auto sz = fs::file_size(path, ec);
    if (ec || sz == 0) return false;
    return true;
}

void DataTransmitter::run() {
    // 确保监控目录存在
    std::error_code ec;
    fs::create_directories(monitorDir_, ec);

    // 扫描并跟进目录文件（跨平台通用轮询机制）
    while (gRunning_.load(std::memory_order_acquire)) {
        // ===== 1. 处理传输队列 =====
        while (!fileQueue_.empty()) {
            std::string path = std::move(fileQueue_.front());
            fileQueue_.pop();
            tx_queue_depth_.fetch_sub(1, std::memory_order_relaxed);
            tx_attempts_.fetch_add(1, std::memory_order_relaxed);

            if (!pathExistsReadableNonEmpty(path)) {
                Logger::transmitLogger()->warn("跳过不可读/空文件: {}", path);
                auto it = fileStates_.find(path);
                if (it != fileStates_.end()) it->second.queued = false;
                continue;
            }

            const auto ip = DefaultIp();
            const auto port = DefaultPort();
            
            uint64_t fsize = 0;
            try {
                fsize = static_cast<uint64_t>(fs::file_size(path));
            } catch (...) {
                fsize = 0;
            }
            
            bool ok = SendLazFileTo(path, ip, port);

            auto it = fileStates_.find(path);
            if (it != fileStates_.end()) {
                it->second.queued = false;
                it->second.transmitted = ok ? true : it->second.transmitted;
            }

            if (ok) {
                tx_success_.fetch_add(1, std::memory_order_relaxed);
                if (fsize > 0) {
                    tx_bytes_.fetch_add(fsize, std::memory_order_relaxed);
                }
            } else {
                tx_failed_.fetch_add(1, std::memory_order_relaxed);
            }
        }

        // ===== 2. 跨平台目录扫描与稳定性检测 =====
        try {
            if (fs::exists(monitorDir_)) {
                std::unordered_map<std::string, bool> currentScanFiles;
                for (const auto& entry : fs::directory_iterator(monitorDir_)) {
                    if (entry.is_regular_file() && isLazPath(entry.path())) {
                        std::string fullPath = entry.path().string();
                        currentScanFiles[fullPath] = true;

                        auto mtime = entry.last_write_time().time_since_epoch().count();
                        auto fsize = entry.file_size();

                        auto it = fileStates_.find(fullPath);
                        if (it == fileStates_.end()) {
                            // 新文件跟踪
                            fileStates_[fullPath] = FileState{
                                .lastModified = mtime,
                                .lastSize = fsize,
                                .queued = false,
                                .transmitted = false
                            };
                            Logger::transmitLogger()->info("新文件，开始跟踪: {}", fullPath);
                        } else {
                            // 判断文件稳定性（修改时间和大小停止变化）
                            auto& st = it->second;
                            bool stable = (!st.queued && !st.transmitted &&
                                           st.lastModified == mtime &&
                                           st.lastSize == fsize);
                            st.lastModified = mtime;
                            st.lastSize = fsize;

                            if (stable) {
                                fileQueue_.push(fullPath);
                                st.queued = true;
                                tx_enqueued_.fetch_add(1, std::memory_order_relaxed);
                                tx_queue_depth_.fetch_add(1, std::memory_order_relaxed);
                                Logger::transmitLogger()->info("文件稳定，加入传输队列: {}", fullPath);
                            }
                        }
                    }
                }

                // 清除已删除文件的跟踪状态
                for (auto it = fileStates_.begin(); it != fileStates_.end(); ) {
                    if (currentScanFiles.find(it->first) == currentScanFiles.end()) {
                        Logger::transmitLogger()->info("文件已删除，移除跟踪: {}", it->first);
                        it = fileStates_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
        } catch (const std::exception& e) {
            Logger::transmitLogger()->warn("目录扫描失败: {}", e.what());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ========== 自由函数：状态消息发送 ==========
bool SendStatusMessage(const std::string& message) {
    const std::string ip = DataTransmitter::DefaultIp();
    const uint16_t    port = DataTransmitter::DefaultPort();
    if (ip.empty() || port == 0) {
        Logger::transmitLogger()->error("缺省端点未设置，无法发送状态消息");
        return false;
    }
    return SendStatusMessageTo(ip, port, message);
}

bool SendStatusMessageTo(const std::string& ip, uint16_t port,
                         const std::string& message) {
    socket_t fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (!IS_VALID_SOCKET(fd)) {
        Logger::transmitLogger()->error("socket 创建失败");
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        Logger::transmitLogger()->error("无效 IP: {}", ip);
        CLOSE_SOCKET(fd);
        return false;
    }

    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        Logger::transmitLogger()->error("连接 {}:{} 失败", ip, port);
        CLOSE_SOCKET(fd);
        return false;
    }

    const char type = 'S';
    uint32_t len = htonl(static_cast<uint32_t>(message.size()));

    if (::send(fd, &type, 1, 0) != 1) {
        Logger::transmitLogger()->error("发送状态类型失败");
        CLOSE_SOCKET(fd); return false;
    }
    if (::send(fd, reinterpret_cast<const char*>(&len), sizeof(len), 0) != static_cast<int>(sizeof(len))) {
        Logger::transmitLogger()->error("发送消息长度失败");
        CLOSE_SOCKET(fd); return false;
    }
    if (::send(fd, message.data(), static_cast<int>(message.size()), 0) != static_cast<int>(message.size())) {
        Logger::transmitLogger()->error("发送消息内容失败");
        CLOSE_SOCKET(fd); return false;
    }

    CLOSE_SOCKET(fd);
    return true;
}

// ========== 自由函数：LAZ文件发送 ==========
bool SendLazFileTo(const std::string& filepath, const std::string& ip,
                   uint16_t port) {
    if (!DataTransmitter::isLazPath(fs::path(filepath))) {
        Logger::transmitLogger()->error("仅支持 .laz 文件: {}", filepath);
        return false;
    }

    std::ifstream file(filepath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        Logger::transmitLogger()->error("无法打开文件: {}", filepath);
        return false;
    }
    const std::streamsize fsz = file.tellg();
    if (fsz <= 0) {
        Logger::transmitLogger()->error("空文件或大小无效: {}", filepath);
        return false;
    }
    file.seekg(0, std::ios::beg);

    socket_t fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (!IS_VALID_SOCKET(fd)) {
        Logger::transmitLogger()->error("socket 创建失败");
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        Logger::transmitLogger()->error("无效 IP: {}", ip);
        CLOSE_SOCKET(fd);
        return false;
    }
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        Logger::transmitLogger()->error("连接 {}:{} 失败", ip, port);
        CLOSE_SOCKET(fd);
        return false;
    }

    const std::string base = fs::path(filepath).filename().string();
    uint32_t name_len = htonl(static_cast<uint32_t>(base.size()));

    const char type = 'F';
    if (::send(fd, &type, 1, 0) != 1) {
        Logger::transmitLogger()->error("发送文件类型失败");
        CLOSE_SOCKET(fd); return false;
    }
    if (::send(fd, reinterpret_cast<const char*>(&name_len), sizeof(name_len), 0) != static_cast<int>(sizeof(name_len))) {
        Logger::transmitLogger()->error("发送文件名长度失败");
        CLOSE_SOCKET(fd); return false;
    }
    if (::send(fd, base.data(), static_cast<int>(base.size()), 0) != static_cast<int>(base.size())) {
        Logger::transmitLogger()->error("发送文件名失败");
        CLOSE_SOCKET(fd); return false;
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    std::vector<char> buf(64 * 1024);
    std::size_t total = 0;

    while (file) {
        file.read(buf.data(), buf.size());
        std::streamsize n = file.gcount();
        if (n <= 0) break;

        const char* p = buf.data();
        std::streamsize left = n;
        while (left > 0) {
            int sent = ::send(fd, p, static_cast<int>(left), 0);
            if (sent < 0) {
                Logger::transmitLogger()->error("发送文件内容失败");
                CLOSE_SOCKET(fd);
                return false;
            }
            p += sent;
            left -= sent;
            total += static_cast<std::size_t>(sent);
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double sec = std::chrono::duration<double>(t1 - t0).count();
    double mb = static_cast<double>(fsz) / (1024.0 * 1024.0);
    double rate = (sec > 0.0) ? (mb / sec) : 0.0;

    Logger::transmitLogger()->info("文件发送完成: {} ({} MB, {:.2f} MB/s)",
                                   base, mb, rate);

    CLOSE_SOCKET(fd);
    return true;
}
