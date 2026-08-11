#include "config_manager.h"
#include <Logger.h>

#include <iostream>
#include <sstream>

// 加载配置文件
bool ConfigManager::load(const std::string& filePath){
    // 清空现有配置
    config_.clear();
    rootNode_ = YAML::Node();
    currentFilePath_ = filePath;

    try{
        // load yaml file
        rootNode_ = YAML::LoadFile(filePath);
        if(!rootNode_.IsDefined()){
            Logger::configLogger()->critical("加载YAML文件失败: {}", filePath);
            return false;
        }

        parseNode(rootNode_);
        Logger::configLogger()->info("配置文件加载成功: {} (共 {} 项配置)", 
                                    filePath, config_.size());
        return true;
    } catch(const YAML::BadFile& e) {
        Logger::configLogger()->critical("无法打开配置文件: {} ({})", filePath, e.what());
    } catch(const YAML::ParserException& e){
        Logger::configLogger()->critical("配置文件解析错误: {} (行 {}): {}", 
                                        filePath, e.mark.line + 1, e.what());
    } catch (const std::exception& e) {
        Logger::configLogger()->critical("加载配置文件失败: {}", e.what());
    }

    return false;
}

// 检查配置项是否存在
bool ConfigManager::exists(const std::string& key) const {
    return config_.find(key) != config_.end();
}

// 获取嵌套配置节点
YAML::Node ConfigManager::getNode(const std::string& key) const{
    auto it = config_.find(key);
    if (it != config_.end()){
        return it->second;
    }
    Logger::configLogger()->debug("配置项 {} 不存在", key);
    return YAML::Node(); // 返回空节点
}

// 重新加载配置文件
bool ConfigManager::reload() {
    if (currentFilePath_.empty()) {
        Logger::configLogger()->error("错误: 未加载任何配置文件，无法重新加载");
        return false;
    }
    Logger::configLogger()->info("重新加载配置文件: {}", currentFilePath_);
    return load(currentFilePath_);
}

// 获取当前加载的配置文件路径
std::string ConfigManager::getFilePath() const {
    return currentFilePath_;
}

// 递归解析YAML节点，将嵌套结构扁平化为"key.subkey"形式存储
void ConfigManager::parseNode(const YAML::Node& node, const std::string& parentKey) {
    if (!node.IsDefined()) return;

    // 处理映射节点（键值对）
    if (node.IsMap()) {
        for (const auto& entry : node) {
            std::string key = entry.first.as<std::string>();
            std::string fullKey = parentKey.empty() ? key : parentKey + "." + key;

            // 若子节点是映射或序列，继续递归解析；否则直接存储
            if (entry.second.IsMap() || entry.second.IsSequence()) {
                parseNode(entry.second, fullKey);
            } else {
                config_[fullKey] = entry.second;
                Logger::configLogger()->debug("解析配置项: {} - {}", fullKey, entry.second.as<std::string>());
            }
        }
    }
    // 处理序列节点（数组）
    else if (node.IsSequence()) {
        for (size_t i = 0; i < node.size(); ++i) {
            std::stringstream ss;
            ss << parentKey << "[" << i << "]";
            parseNode(node[i], ss.str());
        }
    }
    // 处理基本类型节点
    else if (node.IsScalar()) {
        if (!parentKey.empty()) {
            config_[parentKey] = node;
            Logger::configLogger()->debug("解析配置项: {} - {}", parentKey, node.as<std::string>());
        }
    }
}
