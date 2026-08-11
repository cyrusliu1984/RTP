#include "projection.h"


/**
* @brief 将雷达点云转换到IMU坐标系
* @param input 输入点云
* @param params 标定参数
* @param output 输出点云(IMU坐标系)
*/
void Projection::transformToImu(const std::shared_ptr<PointCloudData>& input,
    const LidarToImuParams& params,
    PointCloudData& output) 
{
    // 创建旋转矩阵
    Eigen::Matrix3d R0 = eulerToRotm(params.yaw, params.pitch, params.roll);

    output.times = input->times;
    output.points.resize(input->points.size());

    // 应用旋转和平移
    for (size_t i = 0; i < input->points.size(); ++i) 
    {
        output.points[i] = R0 * input->points[i] + params.translation;
    }
}



/**
* @brief 将点从IMU坐标系转换到世界坐标系(UTM)
* @param input 输入点云(IMU坐标系)
* @param world_points 输出点云(世界坐标系)
* @param pos POS数据(位置和姿态)
*/
void Projection::transformToWorld(std::vector<Eigen::Vector3d>& input,
    std::vector<Eigen::Vector3d>& world_points, POSData pos) {
    std::cout << "解算点云到地理投影坐标系..." << std::endl;

    // 将每个点依次转换
    for (size_t i = 0; i < input.size(); ++i) {
        // 获取点和对应的POS数据
        const Eigen::Vector3d& P_lidar = input[i];
        double yaw = pos.heading;
        double pitch = pos.pitch;
        double roll = pos.roll;

        // 将经纬度转换为UTM坐标
        auto utm = projection(pos.longitude, pos.latitude);

        // 构建IMU旋转矩阵（负角度）
        Eigen::Matrix3d R_imu = eulerToRotm(-yaw, -roll, -pitch);

        // 原点坐标（UTM + 高度）
        Eigen::Vector3d origin_xyz(utm.first, utm.second, pos.altitude);

        // 应用变换
        world_points[i] = R_imu.transpose() * P_lidar + origin_xyz;
    }

    std::cout << "坐标系转换完成" << std::endl;
}


/**
* @brief 保存点云数据到PCD文件
* @param data 点云数据
* @param filepath 文件路径
*/
void Projection::savePointCloudToPCD(const std::shared_ptr<PointCloudData>& data,
    const std::string& filepath) 
{
    std::cout <<"--------------------------------------------------------------"<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 填充点云数据
    cloud->width = data->points.size();
    cloud->height = 1;
    cloud->points.resize(data->points.size());

    for (size_t i = 0; i < data->points.size(); ++i) 
    {
        cloud->points[i].x = data->points[i](0);
        cloud->points[i].y = data->points[i](1);
        cloud->points[i].z = data->points[i](2);
        cloud->points[i].intensity = data->times[i];
    }

    // 保存点云
    pcl::io::savePCDFileASCII(filepath + ".pcd", *cloud);
}



void Projection::savePointCloudToLAZ(const std::shared_ptr<PointCloudData>& cloud,
    const std::string& outputFilePath) 
{
    if (!cloud || cloud->points.empty()) {
        std::cerr << "Error: Point cloud is empty or invalid." << std::endl;
        return;
    }

    // 1. 初始化LASzip写入器
    laszip_POINTER writer;
    if (laszip_create(&writer) != 0) {
        std::cerr << "Error creating laszip writer" << std::endl;
        return;
    }

    // 2. 初始化头信息
    laszip_header* header;
    if (laszip_get_header_pointer(writer, &header) != 0) {
        std::cerr << "Error getting header pointer" << std::endl;
        laszip_destroy(writer);
        return;
    }

    // 设置LAS 1.2版本
    header->version_major = 1;
    header->version_minor = 2;
    header->point_data_format = 2;  // 格式2 (含GPS时间和RGB)
    header->point_data_record_length = 26;
    
    // 设置系统标识
    strncpy(header->system_identifier, "LiDAR System", 32);
    strncpy(header->generating_software, "Projection Module", 32);
    
    // 设置坐标精度
    header->x_scale_factor = 0.001;
    header->y_scale_factor = 0.001;
    header->z_scale_factor = 0.001;
    
    // 计算边界框
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x;
    double min_z = min_x, max_z = max_x;

    for (const auto& point : cloud->points) 
    {
        min_x = std::min(min_x, point.x());
        min_y = std::min(min_y, point.y());
        min_z = std::min(min_z, point.z());
        max_x = std::max(max_x, point.x());
        max_y = std::max(max_y, point.y());
        max_z = std::max(max_z, point.z());
    }

    header->x_offset = min_x;
    header->y_offset = min_y;
    header->z_offset = min_z;
    
    header->min_x = min_x;
    header->max_x = max_x;
    header->min_y = min_y;
    header->max_y = max_y;
    header->min_z = min_z;
    header->max_z = max_z;

    header->number_of_point_records = cloud->points.size(); // 点计数
    
    // 设置文件创建时间
    time_t current_time = time(nullptr);
    struct tm* time_info = localtime(&current_time);
    header->file_creation_year = 1900 + time_info->tm_year;
    header->file_creation_day = time_info->tm_yday;


    // 3. 打开LAZ文件
    laszip_BOOL compress = 1;  // 启用压缩
    if (laszip_open_writer(writer, (outputFilePath + ".laz").c_str(), compress) != 0) {
        laszip_CHAR* error;
        laszip_get_error(writer, &error);
        std::cerr << "Error opening LAZ file: " << error << std::endl;
        laszip_destroy(writer);
        return;
    }

    // 4. 准备点数据
    laszip_point* point;
    if (laszip_get_point_pointer(writer, &point) != 0) {
        std::cerr << "Error getting point pointer" << std::endl;
        laszip_close_writer(writer);
        laszip_destroy(writer);
        return;
    }



    // 遍历点云中的每个点并进行赋色
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        // 设置点的坐标（使用缩放因子和偏移量）
        point->X = static_cast<laszip_I32>((cloud->points[i](0) - header->x_offset) / header->x_scale_factor);
        point->Y = static_cast<laszip_I32>((cloud->points[i](1) - header->y_offset) / header->y_scale_factor);
        point->Z = static_cast<laszip_I32>((cloud->points[i](2) - header->z_offset) / header->z_scale_factor);
            

        // point->gps_time = cloud->time[i];
        
        // 写入点
        laszip_write_point(writer);
    }

     // 6. 关闭文件
    if (laszip_close_writer(writer) != 0) {
        laszip_CHAR* error;
        laszip_get_error(writer, &error);
        std::cerr << "Error closing file: " << error << std::endl;
    }
    
    laszip_destroy(writer);
}




/**
* @brief 经纬度转UTM核心算法
* @param lon 经度(度)
* @param lat 纬度(度)
* @param easting 输出:东向坐标(m)
* @param northing 输出:北向坐标(m)
* @return 是否成功
*/
bool Projection::lonlatToUtm(double lon, double lat, double& easting, double& northing) {
    // 转换角度到弧度
    double lat_rad = deg2rad(lat);
    double lon_rad = deg2rad(lon);

    // 计算中央经线
    double lon0 = centralMeridian(UTM_ZONE);
    double lon0_rad = deg2rad(lon0);

    // 计算参数
    double e_sq = WGS84_F * (2 - WGS84_F);
    double e_prime_sq = e_sq / (1 - e_sq);
    double N = WGS84_A / sqrt(1 - e_sq * pow(sin(lat_rad), 2));
    double T = pow(tan(lat_rad), 2);
    double C = e_prime_sq * pow(cos(lat_rad), 2);
    double A = (lon_rad - lon0_rad) * cos(lat_rad);

    // 计算M (子午线弧长)
    double M = WGS84_A * ((1 - e_sq / 4 - 3 * pow(e_sq, 2) / 64 - 5 * pow(e_sq, 3) / 256) * lat_rad
        - (3 * e_sq / 8 + 3 * pow(e_sq, 2) / 32 + 45 * pow(e_sq, 3) / 1024) * sin(2 * lat_rad)
        + (15 * pow(e_sq, 2) / 256 + 45 * pow(e_sq, 3) / 1024) * sin(4 * lat_rad)
        - (35 * pow(e_sq, 3) / 3072) * sin(6 * lat_rad));

    // 计算东坐标和北坐标
    easting = UTM_K0 * N * (A + (1 - T + C) * pow(A, 3) / 6
        + (5 - 18 * T + T * T + 72 * C - 58 * e_prime_sq) * pow(A, 5) / 120) + 500000.0;

    northing = UTM_K0 * (M + N * tan(lat_rad) * (pow(A, 2) / 2
        + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24
        + (61 - 58 * T + T * T + 600 * C - 330 * e_prime_sq) * pow(A, 6) / 720));

    // 南半球处理
    if (!NORTHERN_HEMISPHERE) {
        northing += 10000000.0;
    }

    return true;
}


// /**
// * @brief 将经纬度转换为UTM坐标
// * @param lon 经度(度)
// * @param lat 纬度(度)
// * @return 东向和北向坐标(米)的对组
// */
// inline std::pair<double, double> Projection::projection(double lon, double lat) {
//     double easting = 0.0, northing = 0.0;
//     lonlatToUtm(lon, lat, easting, northing);
//     return std::make_pair(easting, northing);
// }


/**
* @brief 批量转换经纬度到UTM坐标
* @param lon 经度数组
* @param lat 纬度数组
* @return UTM坐标数组
*/
std::vector<std::pair<double, double>> Projection::projectionBatch(
    const std::vector<double>& lon,
    const std::vector<double>& lat) {

    std::vector<std::pair<double, double>> result;
    result.reserve(lon.size());

    for (size_t i = 0; i < lon.size(); ++i) {
        result.push_back(projection(lon[i], lat[i]));
    }

    return result;
}


/**
* @brief 欧拉角转旋转矩阵(ZYX顺序)
* @param yaw 偏航角(度)
* @param pitch 俯仰角(度)
* @param roll 滚转角(度)
* @return 旋转矩阵
*/
Eigen::Matrix3d Projection::eulerToRotm(double yaw, double pitch, double roll) {
    yaw = -yaw;
    pitch = -pitch;
    roll = -roll;
    // 转坐标轴（相当于每个点乘以R的转置）
    // 也就是等价于 yaw = -yaw, pitch = -pitch, roll = -roll
    // 转换为弧度
    double yaw_rad = deg2rad(yaw);
    double pitch_rad = deg2rad(pitch);
    double roll_rad = deg2rad(roll);

    // 绕Z轴旋转 (偏航)
    Eigen::Matrix3d Rz;
    Rz << cos(yaw_rad), sin(yaw_rad), 0,
        -sin(yaw_rad), cos(yaw_rad), 0,
        0, 0, 1;

    // 绕Y轴旋转 (俯仰)
    Eigen::Matrix3d Ry;
    Ry << cos(pitch_rad), 0, -sin(pitch_rad),
        0, 1, 0,
        sin(pitch_rad), 0, cos(pitch_rad);

    // 绕X轴旋转 (滚转)
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
        0, cos(roll_rad), sin(roll_rad),
        0, -sin(roll_rad), cos(roll_rad);

    // 内旋转顺序: Z->Y->X
    return Rz * Ry * Rx;
}



// Eigen::Matrix3d Projection::eulerToRotm(double yaw, double pitch, double roll) {
//     // 转换为弧度
//     double yaw_rad = deg2rad(yaw);
//     double pitch_rad = deg2rad(pitch);
//     double roll_rad = deg2rad(roll);

//     Eigen::Matrix3d Rz;
//     Rz << cos(yaw_rad), -sin(yaw_rad), 0,  // 第1行第2列改为负
//       sin(yaw_rad),  cos(yaw_rad), 0,  // 第2行第1列改为正
//       0, 0, 1;

//     // 绕Y轴（俯仰）

//     Eigen::Matrix3d Ry;
//     Ry << cos(pitch_rad), 0, sin(pitch_rad),  // 第1行第3列改为正
//       0, 1, 0,
//       -sin(pitch_rad), 0, cos(pitch_rad); // 第3行第1列改为负

//     // 绕X轴（滚转）保持不变（因原代码符号正确）

//     Eigen::Matrix3d Rx;
//     Rx << 1, 0, 0,
//       0, cos(roll_rad), -sin(roll_rad), // 改为负！ <<<<<< 这里也需修正
//       0, sin(roll_rad), cos(roll_rad);

//     // 内旋转顺序: Z->Y->X
//     return Rz * Ry * Rx;
// }