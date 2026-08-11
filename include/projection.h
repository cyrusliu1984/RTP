#pragma once
// #define _CRT_SECURE_NO_WARNINGS
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <stdexcept>
#include <string>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <laszip/laszip_api.h>

#include <type.h>

// #include "PosDataProcess.h"
// #include "LiDARDataProcess.h"

// struct LidarToImuParams {
//     double yaw = -90.0;                   // 偏航角(度)
//     double pitch = 90.0;                   // 俯仰角(度)
//     double roll = 0.0;                   // 滚转角(度)
//     Eigen::Vector3d translation = Eigen::Vector3d(0.283, -0.004, -0.022);  // 平移向量(m)
// };

// 1126 chouzhou waichang ji zai
struct LidarToImuParams {
    double yaw = 0.0;                   // 偏航角(度)
    double pitch = 0.0;                   // 俯仰角(度)
    double roll = 0.0;                   // 滚转角(度)
    Eigen::Vector3d translation = Eigen::Vector3d(0.283, -0.004, -0.022);  // 平移向量(m)
};

struct CameraToImuParams {
    double yaw = 90.0;                   // 偏航角(度)
    double pitch = 0.0;                  // 俯仰角(度)
    double roll = 180.0;                 // 滚转角(度)
    Eigen::Vector3d translation = Eigen::Vector3d(0.304, 0.119, -0.113);  // 平移向量(m)
};

class Projection {
public:
    // 常量定义 (WGS84 - UTM48N参数)
    static constexpr double WGS84_A = 6378137.0;           // 长半轴 (WGS84)
    static constexpr double WGS84_F = 1.0 / 298.257223563; // 扁率 (WGS84)
    static constexpr double WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; 
    static constexpr double UTM_K0 = 0.9996;               // 尺度因子
    static constexpr int UTM_ZONE = 48;                    // UTM区号 (48N)   
    static constexpr bool NORTHERN_HEMISPHERE = true;      // 是否为北半球

    
    static void transformToImu(const std::shared_ptr<PointCloudData>& input,
        const LidarToImuParams& params,
        PointCloudData& output);


    static void transformToWorld(std::vector<Eigen::Vector3d>& input,
        std::vector<Eigen::Vector3d>& world_points, POSData pos);

    
    static void savePointCloudToPCD(const std::shared_ptr<PointCloudData>& data,
        const std::string& filepath);

        
    static void savePointCloudToLAZ(const std::shared_ptr<PointCloudData>& data,
    const std::string& filepath);

    
    /**
    * @brief 将经纬度转换为UTM坐标
    * @param lon 经度(度)
    * @param lat 纬度(度)
    * @return 东向和北向坐标(米)的对组
    */
    inline static std::pair<double, double> projection(double lon, double lat)
    {
        double easting = 0.0, northing = 0.0;
        lonlatToUtm(lon, lat, easting, northing);
        return std::make_pair(easting, northing);
    }

    
    static std::vector<std::pair<double, double>> projectionBatch(
        const std::vector<double>& lon,
        const std::vector<double>& lat);

    static Eigen::Matrix3d eulerToRotm(double yaw, double pitch, double roll);

    /**
     * @brief 角度转弧度
     * @param deg 角度值
     * @return 弧度值
     */
    static inline double deg2rad(double deg) 
    {
        return deg * M_PI / 180.0;
    }

    /**
     * @brief 弧度转角度
     * @param rad 弧度值
     * @return 角度值
     */
    static inline double rad2deg(double rad) 
    {
        
        return rad * 180.0 / M_PI;
    }


private:
    /**
     * @brief 计算UTM中央经线
     * @param zone UTM区号
     * @return 中央经线(度)
     */
    static inline double centralMeridian(int zone) 
    {
        return (zone - 1) * 6 - 180 + 3; // 中央经线公式
    }

    
    static bool lonlatToUtm(double lon, double lat, double& easting, double& northing);

    
    /**
     * @brief 欧拉角转旋转矩阵(ZYX顺序)
     * @param yaw 偏航角(度)
     * @param pitch 俯仰角(度)
     * @param roll 滚转角(度)
     * @return 旋转矩阵
     */
    
};