#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <iomanip> 
#include "Logger.h"

namespace Logger{
    // 静态变量定义
    static std::shared_ptr<spdlog::logger> sMainLogger;  // 主函数
    static std::shared_ptr<spdlog::logger> sScheduleLogger;  // 调度类
    static std::shared_ptr<spdlog::logger> sLidarReceiverLogger;  // 雷达数据接收
    static std::shared_ptr<spdlog::logger> sPosReceiverLogger;  // Pos数据接收
    static std::shared_ptr<spdlog::logger> sLidarParserLogger;  // 雷达数据解析类
    static std::shared_ptr<spdlog::logger> sPosParserLogger;  // Pos数据解析
    static std::shared_ptr<spdlog::logger> sRegisterLogger;  // 点云融合
    static std::shared_ptr<spdlog::logger> sTransmitLogger;  // 数传->地面
    static std::shared_ptr<spdlog::logger> sWriteLogger; // Laz写入Logger
    static std::shared_ptr<spdlog::logger> sColorationLogger; // 着色Logger
    static std::shared_ptr<spdlog::logger> sConfigLogger;  // 配置Logger
    static std::shared_ptr<spdlog::logger> sMonitorLogger;  // 运行状态监控Logger

    std::string createLogDir(){
        auto now = std::chrono::system_clock::now();
        auto nowTimeT = std::chrono::system_clock::to_time_t(now);
        std::tm localTm;
#ifdef _WIN32
        localtime_s(&localTm, &nowTimeT);
#else
        localtime_r(&nowTimeT, &localTm);
#endif

        std::stringstream ss;
        ss << "../logs/"
           << std::put_time(&localTm, "%Y%m%d_%H%M%S");
        
        std::string logDir = ss.str();
        try {
            std::filesystem::create_directories(logDir);
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "无法创建日志目录: " << e.what() << std::endl;
            throw;  // 传递异常，让调用者处理
        }
        return logDir + "/";
    }

    void init() {
        try {
            // 创建控制台输出器（多线程安全）
            auto console_sink = std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>();

            // 创建日志目录并获取路径
            std::string logDir = createLogDir();
            
            // 单个文件最大尺寸（10MB）和最大文件数
            const size_t maxFileSize = 10 * 1024 * 1024;
            const size_t maxFiles = 10;
            
            //  helper函数：创建带有控制台和文件输出的logger
            auto createLogger = [&](const std::string& name) {
                // 为每个logger创建独立的日志文件
                auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                    logDir + name + ".log",  // 文件名格式: 目录/logger名称.log
                    maxFileSize,
                    maxFiles);
                
                // 同时输出到控制台和文件
                spdlog::sinks_init_list sinks{console_sink, file_sink};
                auto logger = std::make_shared<spdlog::logger>(name, sinks);
                spdlog::register_logger(logger);
                return logger;
            };

            // 初始化各模块日志器（使用新变量名）
            sMainLogger = createLogger("main");
            sScheduleLogger = createLogger("schedule");
            sLidarReceiverLogger = createLogger("lidarReceiver");
            sPosReceiverLogger = createLogger("posReceiver");
            sLidarParserLogger = createLogger("lidarParser");
            sPosParserLogger = createLogger("posParser");
            sRegisterLogger = createLogger("register");
            sTransmitLogger = createLogger("transmit");
            sWriteLogger = createLogger("write");
            sColorationLogger = createLogger("coloration");
            sConfigLogger = createLogger("config");
            sMonitorLogger = createLogger("monitor");

            // 设置全局日志级别
            spdlog::set_level(spdlog::level::debug);
            spdlog::set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] [%l] [%n]%$ %v");  // %^开始颜色，%$结束颜色
            spdlog::flush_on(spdlog::level::debug); // 设置info级别以上自动flush

            sMainLogger->info("日志系统成功初始化！日志文件路径: {}", logDir);
        } catch (const spdlog::spdlog_ex& ex) {
            std::cerr << "日志初始化系统失败：" << ex.what() << std::endl;
            throw;
        }
    }

    // 日志器获取函数
    std::shared_ptr<spdlog::logger> mainLogger() {
        return sMainLogger;
    }

    std::shared_ptr<spdlog::logger> scheduleLogger() {
        return sScheduleLogger;
    }

    std::shared_ptr<spdlog::logger> lidarReceiverLogger() {
        return sLidarReceiverLogger;
    }

    std::shared_ptr<spdlog::logger> posReceiverLogger() {
        return sPosReceiverLogger;
    }

    std::shared_ptr<spdlog::logger> lidarParserLogger() {
        return sLidarParserLogger;
    }

    std::shared_ptr<spdlog::logger> posParserLogger() {
        return sPosParserLogger;
    }

    std::shared_ptr<spdlog::logger> registerLogger() {
        return sRegisterLogger;
    }

    std::shared_ptr<spdlog::logger> transmitLogger() {
        return sTransmitLogger;
    }

    std::shared_ptr<spdlog::logger> writeLogger() {
        return sWriteLogger;
    }

    std::shared_ptr<spdlog::logger> colorationLogger() {
        return sColorationLogger;
    }

    std::shared_ptr<spdlog::logger> configLogger() {
        return sConfigLogger;
    }

    std::shared_ptr<spdlog::logger> monitorLogger() {
        return sMonitorLogger;
    }
    
}