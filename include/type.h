#pragma once
#include <atomic>
#include <vector>
#include <array>
#include <string>
#include <cstdint>
#include <mutex>
#include <memory>
#include <exception>
#include <type_traits>
#include <cmath>

#include <Eigen/Dense>
#include "moodycamel/concurrentqueue.h"


// ====================================== Lidar Related ======================================
#pragma pack(push, 1) // 确保结构体按1字节对齐
// 单个脉冲结构数据
struct PulseData {
    uint16_t head; // 脉冲帧头
    uint16_t razorIdx; // 脉冲序号
    uint16_t disStart; // 距离门起始
    uint16_t disEnd; // 距离门终止
    uint32_t razorTick; // 微秒计时
    uint32_t razorCnt; // 码盘计数
    std::array<uint8_t, 7168> echoData; // 7168 -> 64 x 64 数据
};

// 数据包结构 （包含200个脉冲数据）
struct PacketData {
    uint16_t head1; // 包头1
    uint16_t head2; // 包头2
    uint16_t head3; // 包头3
    uint16_t head4; // 包头4
    uint16_t gpsWeek; // GPS周数
    uint32_t msecInWeek; // GPS秒数 (毫秒)
    uint16_t timeRes; // 时间分辨率
    std::array<PulseData, 200> pulseData; // 200个脉冲数据
}; 

struct PacketDataHead {
    uint16_t head1;
    uint16_t head2;
    uint16_t head3;
    uint16_t head4;
};
#pragma pack(pop)  // 恢复默认对齐

struct PointCloudData{
    int64_t timeStamp = 0;
    std::vector<Eigen::Vector3d> points;
    std::vector<double> times;  // 单位：毫秒

    PointCloudData() = default;
    PointCloudData(int64_t ts, std::vector<Eigen::Vector3d>&& pts, std::vector<double>&& t): timeStamp(ts), points(std::move(pts)), times(std::move(t)) {}
};

// ====================================== POS Related ======================================
#pragma pack(push, 1) // 确保结构体按1字节对齐
struct POSRawData{ // 28 + 120 + 4 = 152 bytes (158bytes) // 28 + 126 +4 = 158
    uint8_t head1; // 0xAA 
    uint8_t head2; // 0x44
    uint8_t head3; // 0x12
    uint8_t packetLen; // 
    uint16_t messageId;
    uint8_t unknown1[7];
    uint8_t gpsTimeStatus;
    uint16_t gpsWeek;
    uint32_t msecInWeek;
    uint8_t unknown2[8];  // 前28位
    uint32_t insStatus; 
    uint8_t unknown3[4];
    double lat;
    double lon;
    //大地高（以参考椭球面为标准） = 海拔高 + 高程异常
    double altitude;  // 海拔高
    float heightAnomaly; // 高程异常
    uint8_t unknown4[24]; // 前88位
    double roll;
    double pitch;
    double yaw;
    uint8_t unknown5[42]; // 前154位

    uint32_t CRC; 
};

struct MarkRawData{ // 28 + 88 + 4
    uint8_t head1;
    uint8_t head2;
    uint8_t head3;
    uint8_t packetLen;
    uint16_t messageId; // 第6位
    uint8_t unknown1[22]; // 前28位
    uint32_t gpsWeek;
    double secInWeek;
    double lat;
    double lon;
    double altitude; //TODO 检查这个是大地高 还是 海拔高！！
    uint8_t unknown2[24]; 
    double roll; // 
    double pitch;
    double yaw; //
    uint8_t unknown3[4]; 

    uint32_t CRC;
};

struct POSData{
    int64_t timeStamp;

    uint16_t gpsWeek;         // GPS周
    uint32_t gpsMilliseconds; // GPS周内毫秒
    double latitude;          // 纬度 (度)
    double longitude;         // 经度 (度)
    double altitude;          // 高度 (米)
    double roll;              // 横滚角 (度)
    double pitch;             // 俯仰角 (度)
    double heading;           // 航向角 (度)

    uint32_t CRC;
};

struct MarkData {
    int64_t timeStamp;
    
    uint32_t week;            
    double seconds;           
    double latitude;          
    double longitude;  
    double altitude;       
    double roll;               
    double pitch;              
    double heading;
    
    uint32_t CRC;
};
#pragma pack(pop)  // 恢复默认对齐

// ====================================== Camera Related ======================================
struct CamImage : MarkData{
    int w, h; // 图像宽高

    // 内参，畸变参数
    // 快速剪裁使用：地面正方形（存d=0时候的四个角点射线）
    // 图像位置/句柄
};

// ====================================== Writer Related ======================================
struct FileState {  // 文件状态跟踪结构
    time_t lastModified;
    size_t lastSize;
    bool queued;
    bool transmitted;   // 标记文件是否已成功传输
};

// ====================================== Coloration Related ======================================
struct LidarPointRaw {
    double x{0}, y{0}, z{0}; // 若 points_are_local=false, 这里放世界坐标
    double x_local{0}, y_local{0}; // 若 true ，已是环心局部坐标
};

struct LidarRingRaw {
    uint64_t ring_id{0};
    int64_t t_ms{0};  // 圈中心/平均时刻
    double cx{0}, cy{0};  // 圈心地面坐标（用于 world->local)
    double yaw_rad{0};  // 航向角（x'方向）
    bool points_are_local{false};
    std::vector<LidarPointRaw> pts;
};

struct ColoredPoint {
    double x_local{0}, y_local{0}, z{0};
    uint8_t r{0}, g{0}, b{0};
    float conf{0.f};
};

struct ColoredRing {
    uint64_t ring_idx{0};
    int64_t t_ms{0};
    std::vector<ColoredPoint> left, right;
    bool left_done{false}, right_done{false};
};


// ====================================== PlateForm Related ======================================
struct ColorationParams {
    double H = 3000;   // 航高
    double v_air = 50; //航速(m/s)
    
    const double l_fov = 60;
    const double c_fov = 28;

    double Rl; // = H * std::tan(l_fov / 2 * M_PI / 180.0);
    double Rc; // = H * std::tan(c_fov / 2 * M_PI / 180.0);
    
    // 窗口(ms)
    int64_t t_in_ms, t_best_ms, t_out_ms, delta_ms;
    // 评分()
    double w_time{1.0}, w_cover{0.2}, w_geom{0.2};

    // 右半定格判断
    bool use_local_stable{true};
    int64_t pass_target_margin_ms{350};

    // 缓存保留
    int64_t keep_cam_ms{70000};

    ColorationParams() = default;

    ColorationParams(double Height, double velocity):H(Height), v_air(velocity){
        Rl = H * std::tan(l_fov / 2 * M_PI / 180.0);
        Rc = H * std::tan(c_fov / 2 * M_PI / 180.0);

        auto ms = [](double s) { return (int64_t)llround(s*1000.0); };
        t_in_ms = ms((Rl - Rc)/v_air);
        t_best_ms = ms(Rl/v_air);
        t_out_ms = ms((Rl + Rc)/v_air);
        delta_ms = t_best_ms;
    } 
};



static_assert(sizeof(POSRawData) == 158, "POSRawData 字节长度应该是158！");
static_assert(sizeof(MarkRawData) == 120, "MarkRawData 字节长度应该是120！");

static_assert(std::is_pod<POSRawData>::value, "POSRawData 应该是POD类型");
static_assert(std::is_pod<MarkRawData>::value, "MarkRawData 不应该是POD类型");
static_assert(std::is_pod<POSData>::value, "POSData 不应该是POD类型");
static_assert(std::is_pod<MarkData>::value, "MarkData 不应该是POD类型");

class AppError : public std::exception {
private:
    int code;
    std::string msg;
public:
    AppError(int c, const std::string& m) : code(c), msg(m) {}
    const char* what() const noexcept override { return msg.c_str(); }
    int getCode() const { return code; }
};


// -------------------- 异常事件监控 --------------------
enum class EventType : uint8_t {
    POS_NON_MONO,
    LIDAR_NON_MONO,
    REG_EARLY,
    REG_LATE,
    REG_TIMEOUT
};

struct EventBasic {
    int64_t wall_ms; // system_clock ms (人类可读)
    EventType type;
    int64_t t0_ms; // 上一次 | 窗口下界 | 期待的时间
    int64_t t1_ms; // 本次 | 窗口上界 | 实际时间
    int64_t t_ms;  // 非递增量 | 超时间隔
    uint32_t id;  // 可选： 帧号/包号（用时间戳来代替）
};

// 环形缓冲区（单生产者/多消费者均可）
template<size_t K>
class EventRing {
public:
    // 覆盖式的o(1)
    void push(const EventBasic& e) {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_[head_ % K] = e; // 完整写payload
        ++head_;
    }

    std::vector<EventBasic> snapshot() const {
        std::lock_guard<std::mutex> lk(mtx_);
        const uint32_t h = head_;
        const uint32_t n = std::min<uint32_t>(h, K);
        std::vector<EventBasic> out;
        out.reserve(n);
        for (uint32_t i = 0; i < n; ++i) {
            out.push_back(buf_[(h - 1 - i) % K]);  // 从新到旧
        }
        return out;
    }

private:
    mutable std::mutex mtx_;
    std::array<EventBasic, K> buf_{};
    uint32_t head_{0}; // 仅仅在mtx_保护下访问
};

struct ROI {
    double lat_min;
    double lat_max;
    double lon_min;
    double lon_max;
};