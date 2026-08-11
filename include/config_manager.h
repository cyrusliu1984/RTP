#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>


class ConfigManager {
public:
    // 单例模式：全局唯一实例
    static ConfigManager& getInstance() {
        static ConfigManager instance;
        return instance;
    }

    // 禁止拷贝和赋值
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    // 读取配置文件
    bool load(const std::string& filePath);

    // 检查配置项是否存在
    bool exists(const std::string& key) const;

    // 获取配置值，支持多种数据类型
    template<typename T>
    T get(const std::string& key) const {
        if(!exists(key)) return T();

        try {
        // const 类型的unordered_map不能直接使用 operator[]，
        // 需要使用 at() 方法 因为 at() 方法会在 key 不存在时抛出异常
            return config_.at(key).as<T>(); 
        } catch (const YAML::BadConversion&){
            return T(); // 转化失败也返回默认值
        }
    }

    // 获取嵌套配置节点
    YAML::Node getNode(const std::string& key) const;
    // 重新加载配置文件
    bool reload();

    // 获取当前加载的配置文件路径
    std::string getFilePath() const;


private:
    // 私有构造函数 
    //ConfigManager 是单例模式，构造函数被声明为 private（私有），目的是禁止外部直接创建对象
    //（只能通过 getInstance() 获取实例）
    ConfigManager() = default;

    // 解析YAML节点到配置映射
    void parseNode(const YAML::Node& node, const std::string& parentKey="");

    YAML::Node rootNode_; // YAML根节点
    std::unordered_map<std::string, YAML::Node> config_; // 扁平化配置存储
    std::string currentFilePath_; // // 当前加载的配置文件路径
};


#endif // CONFIG_MANAGER_H