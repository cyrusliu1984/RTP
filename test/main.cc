/**
 * @file main.cpp
 * @brief 航拍数据处理系统主程序入口
 * @details 负责程序启动初始化、单实例保证、信号处理、数据归档、配置加载
 *          及调度器启动，是整个系统的核心入口，协调雷达采集、LAZ传输等模块运行
 */
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <cstring>
#include <csignal>   // 信号处理

#include <atomic>
#include <chrono>
#include <thread>
#include <cerrno> 

#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h> // geteuid, getpid, sleep, access
    #include <fcntl.h>  // open
    #include <sys/file.h> // flock 文件锁
    #include <sys/stat.h> // chmod
    #include <sys/types.h> // pid_t
#endif

#include "Logger.h"          // 日志模块
#include "config_manager.h"  // 配置管理模块
#include "Schedule.h"        // 调度器模块
#include "utils.h"           // 工具函数
#include "Transmit.h"        // 数据传输模块

namespace fs = std::filesystem;

// 全局运行标志（原子变量，线程安全）
std::atomic<bool> g_running(true);

#ifndef _WIN32
// PID文件路径（用于单实例保证，仅Linux）
static const char* PIDFILE = "/var/run/rtpcp.pid";
#else
static HANDLE g_hMutex = NULL;
#endif

/**
 * @brief 信号处理函数
 * @param signum 接收到的信号编号
 * @details 处理SIGINT(Ctrl+C)和SIGTERM(终止信号)，设置全局退出标志
 */
void signalHandler(int signum) {
    if (signum == SIGINT || signum == SIGTERM) {
        std::cout << "\n收到终止信号，准备退出..." << std::endl;
        // 设置退出标志（release语义，确保其他线程可见）
        g_running.store(false, std::memory_order_release);
    }
}

#ifndef _WIN32
/**
 * @brief 等待进程退出 (Linux)
 */
bool wait_pid_exit(pid_t pid, int timeout_ms) {
    using namespace std::chrono;
    auto deadline = steady_clock::now() + milliseconds(timeout_ms);
    
    while (steady_clock::now() < deadline) {
        if(kill(pid, 0) == -1 && errno == ESRCH) {
            return true;
        }
        std::this_thread::sleep_for(milliseconds(50));
    }
    return (kill(pid, 0) == -1 && errno == ESRCH);
}

/**
 * @brief 优雅终止进程 (Linux)
 */
bool kill_gracefully(pid_t pid, int term_wait_ms = 1500, int kill_wait_ms = 800) {
    if (pid <= 0) return true;

    if (kill(pid, SIGTERM) == 0) {
        if (wait_pid_exit(pid, term_wait_ms)) return true;
    }

    kill(pid, SIGKILL);
    return wait_pid_exit(pid, kill_wait_ms);
}
#endif

/**
 * @brief 单实例保证：获取锁或终止旧实例
 * @return 成功获取锁返回>0（或描述符），失败返回-1
 */
int acquire_single_instance_or_kill_previous() {
#ifdef _WIN32
    g_hMutex = CreateMutexA(NULL, TRUE, "Global\\RTPCP_SINGLE_INSTANCE_MUTEX");
    if (GetLastError() == ERROR_ALREADY_EXISTS) {
        std::cerr << "检测到已有 RTPCP 实例在运行，直接退出。\n";
        if (g_hMutex) CloseHandle(g_hMutex);
        g_hMutex = NULL;
        return -1;
    }
    return 1;
#else
    int fd = open(PIDFILE, O_RDWR | O_CREAT, 0644);
    if (fd < 0){
        std::cerr << "无法打开 PID 文件" << PIDFILE << " : " << std::strerror(errno) << "\n";
        return -1;
    }

    if (flock(fd, LOCK_EX | LOCK_NB) == 0) {
        ftruncate(fd, 0);
        std::string me = std::to_string(getpid()) + "\n";
        write(fd, me.c_str(), me.size());
        return fd;
    }

    if (errno != EWOULDBLOCK) {
        std::cerr << "给PID文件加锁失败: " << std::strerror(errno) << "\n";
        close(fd);
        return -1;
    }

    lseek(fd, 0, SEEK_SET);
    char buf[64] = {0};
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    pid_t oldpid = 0;
    if (n > 0) {
        try{
            oldpid = static_cast<pid_t>(std::stol(buf));
        } catch(...){
            oldpid = 0;
        }
    }

    if (oldpid > 0){
        std::cerr << "检测到已有实例 (PID=" << oldpid << "), 尝试结束...\n";
        if (!kill_gracefully(oldpid, 6500, 1000)) {
            std::cerr << "无法结束旧实例 (PID=" << oldpid << "). \n";
            close(fd);
            return -1;
        }
    } else {
        std::cerr << "PID 文件存在但内容异常，继续尝试夺回锁...\n";
    }

    for(int i = 0; i < 30; i++){
        if(flock(fd, LOCK_EX | LOCK_NB) == 0) {
            ftruncate(fd, 0);
            lseek(fd, 0, SEEK_SET);
            std::string me = std::to_string(getpid()) + "\n";
            write(fd, me.c_str(), me.size());
            return fd;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    std::cerr << "仍然无法获得单实例锁. \n";
    close(fd);
    return -1;
#endif
}

/**
 * @brief 释放单实例资源
 */
void release_single_instance(int fd) {
#ifdef _WIN32
    if (g_hMutex) {
        ReleaseMutex(g_hMutex);
        CloseHandle(g_hMutex);
        g_hMutex = NULL;
    }
#else
    if (fd >= 0) {
        close(fd);
    }
#endif
}

/**
 * @brief 执行系统命令
 */
bool run_cmd(const std::string& cmd) {
    int ret = system(cmd.c_str());
    if (ret != 0) {
        Logger::mainLogger()->error("执行命令失败: {} (返回码 {})", cmd, ret);
        return false;
    }
    return true;
}

/**
 * @brief 清空目录（保留目录本身）
 */
bool clear_dir_keep_self(const fs::path& dir){
    std::error_code ec;
    if (!fs::exists(dir)) {
        fs::create_directories(dir, ec);
        if (ec) {
            Logger::mainLogger()->error("创建目录失败: {} ({})", dir.string(), ec.message());
            return false;
        }
        return true;
    }
    
    if (!fs::is_directory(dir)) {
        Logger::mainLogger()->error("路径不是目录: {}", dir.string());
        return false;
    }

    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        if (ec) {
            Logger::mainLogger()->error("遍历目录失败: {} ({})", dir.string(), ec.message());
            return false;
        }
        fs::remove_all(entry.path(), ec);
        if (ec) {
            Logger::mainLogger()->error("删除文件/目录失败: {} ({})", entry.path().string(), ec.message());
            return false;
        }
    }

    return true;
}

/**
 * @brief 数据归档（轮转）
 */
bool archieve(const fs::path& base_data_dir,
                          const std::string& archive_dir_name = "archive") {
    namespace fs = std::filesystem;
    const std::vector<std::string> src_dirs = {"LAZ", "POS", "lidarFrame"};

    fs::path data_dir = fs::weakly_canonical(base_data_dir);
    fs::path archive_dir = data_dir / archive_dir_name;

    std::error_code ec;
    fs::create_directories(archive_dir, ec);
    if (ec) {
        Logger::mainLogger()->error("创建归档目录失败: {} ({})", archive_dir.string(), ec.message());
        return false;
    }

    std::string timestamp = TimeUtils::get_yymmdd_hhmmss();
    fs::path timestamp_dir = archive_dir / timestamp;
    
    fs::create_directories(timestamp_dir, ec);
    if (ec) {
        Logger::mainLogger()->error("创建时间戳目录失败: {} ({})", timestamp_dir.string(), ec.message());
        return false;
    }

    for (const auto& dir : src_dirs) {
        fs::path src_path = data_dir / dir;
        fs::create_directories(src_path, ec);
        if (ec) {
            Logger::mainLogger()->error("创建数据目录失败: {} ({})", src_path.string(), ec.message());
            return false;
        }

        fs::path dest_path = timestamp_dir / dir;
        
        if (fs::exists(dest_path)) {
            fs::remove_all(dest_path, ec);
            if (ec) {
                Logger::mainLogger()->error("删除已存在的目标目录失败: {} ({})", dest_path.string(), ec.message());
                return false;
            }
        }

        fs::rename(src_path, dest_path, ec);
        if (ec) {
            Logger::mainLogger()->error("移动目录失败: {} -> {} ({})", 
                                      src_path.string(), dest_path.string(), ec.message());
            return false;
        }

        fs::create_directories(src_path, ec);
        if (ec) {
            Logger::mainLogger()->error("重建源目录失败: {} ({})", src_path.string(), ec.message());
            return false;
        }
    }

    Logger::mainLogger()->info("归档完成并已轮转: {}", timestamp_dir.string());
    return true;
}

/**
 * @brief 程序主函数
 */
int main() {
#ifndef _WIN32
    // 1. Linux 下检查是否有 root 权限
    if (geteuid() != 0) {
        std::cerr << "请使用sudo权限运行此程序。" << std::endl;
        return 1;
    }
#endif

    // 2. 单实例保证：获取锁或终止旧实例
    int lock_fd = acquire_single_instance_or_kill_previous();
    if (lock_fd < 0) return 2;
    
    // 3. 注册信号处理（SIGINT/SIGTERM）
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 4. 初始化日志系统
    Logger::init();
    Logger::mainLogger()->info("程序启动");

    // 5. 归档旧数据（数据轮转）
    if (!archieve("../data")) {
        Logger::mainLogger()->error("归档旧数据失败，程序退出");
        release_single_instance(lock_fd);
        return 2;
    }

    // 6. 加载配置文件
    const std::string cfgPth = "../config/default.yaml";
    if (!ConfigManager::getInstance().load(cfgPth)) {
        Logger::mainLogger()->critical("无法加载配置文件: {}", cfgPth);
        release_single_instance(lock_fd);
        return 3;
    }
    Logger::mainLogger()->info("配置文件加载成功: {}", cfgPth);

    {
        // 7. 启动调度器和全链路模块
        try{
            Schedule sched(g_running);

            sched.startAll();

            while (g_running.load(std::memory_order_relaxed)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }

            sched.stopAll();
        } catch (...) {
            Logger::mainLogger()->critical("调度器出错，程序退出");
            std::string statusStr_ = "状态：出错";
            SendStatusMessage(statusStr_);
            release_single_instance(lock_fd);
            return 4;
        }
    }

    // 8. 正常退出：释放资源
    release_single_instance(lock_fd);
    Logger::mainLogger()->info("程序正常退出");
    return 0;
}

