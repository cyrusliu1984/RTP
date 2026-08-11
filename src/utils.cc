#include "utils.h"   // 包含自身头文件
#include <string>
#include <cmath>         // 用于 floor()、round() 函数
#include <fstream>

#include "Logger.h"

// 实现GPS时间转Unix时间戳（返回毫秒级时间戳，类型为int64_t）
int64_t TimeUtils::gpsTimeToLinuxTimestamp(int gpsWeek, uint32_t timeInWeek, TimeUnit timeUnit) {
    // 根据时间单位转换周内时间为毫秒（避免浮点数精度损失）
    int64_t millisecondsInWeek;
    switch (timeUnit) {
        case TimeUnit::Seconds:
            millisecondsInWeek = static_cast<int64_t>(timeInWeek) * 1000;  // 秒转毫秒
            break;
        case TimeUnit::Milliseconds:
            millisecondsInWeek = static_cast<int64_t>(timeInWeek);         // 直接使用毫秒
            break;
        default:
            // 默认按毫秒处理（保持兼容性）
            millisecondsInWeek = static_cast<int64_t>(timeInWeek);
            break;
    }

    // 计算GPS总毫秒数（周数×一周毫秒数 + 周内毫秒数）
    // 一周 = 7*24*60*60 = 604800秒 = 604800*1000 = 604800000毫秒
    int64_t totalGpsMilliseconds = static_cast<int64_t>(gpsWeek) * 604800000LL + millisecondsInWeek;
    
    // GPS转UTC：减去闰秒（转换为毫秒）
    // 注意：GPS_LEAP_SECONDS需定义为当前GPS闰秒数（如2023年为18秒）
    int64_t utcMilliseconds = totalGpsMilliseconds - (GPS_LEAP_SECONDS * 1000LL);
    
    // UTC转Unix时间戳：GPS起始时间的Unix时间戳（毫秒） + UTC毫秒数
    // GPS_EPOCH需定义为1980-01-06 00:00:00的Unix时间戳（毫秒），即315964800000LL
    return GPS_EPOCH + utcMilliseconds;
}

// 实现Unix时间戳转GPS时间（使用int64_t避免浮点精度问题）
void TimeUtils::linuxTimestampToGpsTime(int64_t linuxTimestamp, int& gpsWeek, uint32_t& timeInWeek, TimeUnit timeUnit) {
    // 假设输入的linuxTimestamp是毫秒级Unix时间戳
    // 转换为秒级（如果输入是秒级，可去掉这一步）
    int64_t linuxTsSeconds = linuxTimestamp / 1000LL;
    
    // Unix时间戳转UTC秒数（减去GPS起始时间戳的秒数）
    // GPS_EPOCH应定义为1980-01-06 00:00:00的Unix时间戳（秒）：315964800LL
    int64_t utcSeconds = linuxTsSeconds - GPS_EPOCH;
    
    // UTC转GPS：加上闰秒（GPS比UTC快）
    int64_t totalGpsSeconds = utcSeconds + GPS_LEAP_SECONDS;
    
    // 一周的总秒数：7*24*60*60 = 604800
    const int64_t SECONDS_PER_WEEK = 604800LL;
    
    // 计算GPS周数（总秒数 ÷ 一周秒数，向下取整）
    gpsWeek = static_cast<int>(totalGpsSeconds / SECONDS_PER_WEEK);
    
    // 计算周内秒数
    int64_t secondsInWeek = totalGpsSeconds % SECONDS_PER_WEEK;
    // 处理负数情况（当totalGpsSeconds为负时，取模可能为负）
    if (secondsInWeek < 0) {
        secondsInWeek += SECONDS_PER_WEEK;
        gpsWeek -= 1;
    }
    
    // 根据目标时间单位转换周内时间
    switch (timeUnit) {
        case TimeUnit::Seconds:
            // 秒级直接转换（确保在uint32_t范围内）
            timeInWeek = static_cast<uint32_t>(secondsInWeek);
            break;
        case TimeUnit::Milliseconds: {
            // 转换为毫秒级（先计算毫秒级Unix时间戳的余数）
            int64_t msRemainder = linuxTimestamp % 1000LL;
            timeInWeek = static_cast<uint32_t>(secondsInWeek * 1000LL + msRemainder);
            break;
        }
        default:
            // 默认转秒级
            timeInWeek = static_cast<uint32_t>(secondsInWeek);
            break;
    }
}
    

std::string TimeUtils::get_yymmdd_hhmmss() {
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm); // 线程安全版本
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%04d%02d%02d_%02d%02d%02d",
                    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                    tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(buffer);
}



// 实现CRC32查表生成（静态变量确保全局唯一，线程安全？注：C++11后局部静态初始化线程安全）
const uint32_t* Checksum::get_crc32_table() {
    static uint32_t table[256];    // 静态查表（仅初始化一次）
    static bool initialized = false; // 初始化标志

    if (!initialized) {
        // 生成CRC32查表
        for (int i = 0; i < 256; ++i) {
            uint32_t ulCRC = i;
            for (int j = 0; j < 8; ++j) {
                if (ulCRC & 1) {
                    ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
                } else {
                    ulCRC >>= 1;
                }
            }
            table[i] = ulCRC;
        }
        initialized = true;
    }
    return table;
}

// 实现CRC32校验和计算
uint32_t Checksum::CalculateBlockCRC32(uint32_t ulCount, const uint8_t* ucBuffer) {
    // 先获取CRC32查表
    const uint32_t* crc32_table = get_crc32_table();

    // 初始化CRC值（与发送方保持一致：初始值为0）
    uint32_t ulCRC = 0;
    while (ulCount-- != 0) {
        uint32_t ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;  // 高24位右移保留
        uint32_t ulTemp2 = crc32_table[(ulCRC ^ *ucBuffer++) & 0xFF]; // 查表计算低8位
        ulCRC = ulTemp1 ^ ulTemp2;  // 合并结果
    }
    return ulCRC;
}

void Save::writeVector3dToPCDManual(const std::vector<Eigen::Vector3d>& curSecData, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        Logger::lidarParserLogger()->warn("Error: Could not open file: {}", filename);
        return;
    }

    // 写入PCD文件头
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH " << curSecData.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << curSecData.size() << "\n";
    file << "DATA ascii\n";

    // 写入点数据
    for (const auto& point : curSecData) {
        file << point.x() << " " << point.y() << " " << point.z() << "\n";
    }

    file.close();
    Logger::lidarParserLogger()->debug("Save {} points to {}", curSecData.size(), filename);
}

ROI Proj::computeROI(double lat0_deg, double lon0_deg, double H, double FOV_deg) {
    // 将角度转换为弧度
    double FOV_rad = FOV_deg * M_PI / 180.0;
    double lat0_rad = lat0_deg * M_PI / 180.0;
    // double lon0_rad = lon0_deg * M_PI / 180.0; 卯酉圈曲率半径 子午圈曲率半径 和 经度无关，只与纬度有关

    // 雷达投影半径（m）
    double R = H * std::tan(FOV_rad / 2.0);

    // 每度纬度的米数（近似值，使用子午圈曲率半径）
    double a = Projection::WGS84_A;
    double f = Projection::WGS84_F;
    double e2 = 2 * f - f * f; // 第一偏心率

    double sinLat = std::sin(lat0_rad);
    double M = a * (1 - e2) / std::pow(1 - e2 * sinLat * sinLat, 1.5); // 子午圈曲率半径 d{s_(lat)} = M * d{lat}
    double N = a / std::sqrt(1 - e2 * sinLat * sinLat); // 卯酉圈曲率半径 d{s_(lon)} = N * cos(lat) * d{lon} 

    double meters_per_deg_lat = M * (M_PI / 180.0); // 每度纬度的米数
    double meters_per_deg_lon = N * std::cos(lat0_rad) * (M_PI / 180.0); // 每度经度的米数

    double dLat = R / meters_per_deg_lat; // 纬度变化量（度）
    double dLon = R / meters_per_deg_lon; // 经度变化量（度）

    ROI roi;
    roi.lat_min = lat0_deg - dLat;
    roi.lat_max = lat0_deg + dLat;
    roi.lon_min = lon0_deg - dLon;
    roi.lon_max = lon0_deg + dLon;
    return roi;
}

bool Proj::is_intersect(const ROI& a, const ROI& b) {
    // 如果一个矩形在另一个矩形的左侧或右侧，则不相交
    if (a.lon_max < b.lon_min || b.lon_max < a.lon_min) {
        return false;
    }
    // 如果一个矩形在另一个矩形的上方或下方，则不相交
    if (a.lat_max < b.lat_min || b.lat_max < a.lat_min) {
        return false;
    }
    // 否则，矩形相交
    return true;
}