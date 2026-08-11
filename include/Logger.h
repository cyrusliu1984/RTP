#pragma once
#include "spdlog/spdlog.h"

#include <memory>
#include <string>

namespace Logger {

    /**
     * @brief 初始化日志系统
     * @details 创建日志目录、注册所有模块的日志器、设置日志格式和级别
     * @note 必须在程序启动时调用，且仅需调用一次
     * @throw 若创建目录或初始化日志器失败，会抛出std::exception
     */
    void init();

    /**
     * @brief 创建日志目录（包含当前时间戳）
     * @return 日志目录的完整路径（以'/'结尾）
     * @throw 若目录创建失败，会抛出std::filesystem::filesystem_error
     */
    std::string createLogDir();

    // 各模块日志器的获取接口
    /** @brief 获取主函数日志器 */
    std::shared_ptr<spdlog::logger> mainLogger();
    /** @brief 获取调度类日志器 */
    std::shared_ptr<spdlog::logger> scheduleLogger();
    /** @brief 获取雷达数据接收日志器 */
    std::shared_ptr<spdlog::logger> lidarReceiverLogger();
    /** @brief 获取Pos数据接收日志器 */
    std::shared_ptr<spdlog::logger> posReceiverLogger();
    /** @brief 获取雷达数据解析类日志器 */
    std::shared_ptr<spdlog::logger> lidarParserLogger();
    /** @brief 获取Pos数据解析日志器 */
    std::shared_ptr<spdlog::logger> posParserLogger();
    /** @brief 获取点云融合日志器 */
    std::shared_ptr<spdlog::logger> registerLogger();
    /** @brief 获取数传（到地面）日志器 */
    std::shared_ptr<spdlog::logger> transmitLogger();
    /** @brief 获取Laz写入日志器 */
    std::shared_ptr<spdlog::logger> writeLogger();
    /** @brief 获取着色模块日志器 */
    std::shared_ptr<spdlog::logger> colorationLogger();
    /** @brief 获取配置模块日志器 */
    std::shared_ptr<spdlog::logger> configLogger();
    /** @brief 获取运行状态监控模块日志器 */
    std::shared_ptr<spdlog::logger> monitorLogger();

} // namespace Logger
    