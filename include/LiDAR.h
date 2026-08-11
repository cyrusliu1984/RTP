#pragma once
#include <atomic>
#include <vector>
#include <cstdint>
#include <array>
#include <string>
#include <Eigen/Dense>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    using socket_t = SOCKET;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    using socket_t = int;
#endif

#include "type.h"
#include "moodycamel/concurrentqueue.h"
#include "utils.h"

#include <laszip/laszip_api.h>

// Lidar接收类
class LiDARReceiver {
private:
    const int BUFFER_SIZE = 150000; // 缓冲区大小

    std::string ifaceIp;
    int udpPort;
    socket_t sockfd = (socket_t)-1;
    const std::atomic<bool>& gRunning;
    std::vector<uint8_t> currentPacketData;
    std::vector<uint8_t> recvBuffer;
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    moodycamel::ConcurrentQueue<std::shared_ptr<PacketData>>& outputQueue;
    

    void setupRealTimePriority();
    socket_t createUDPSocket(int udpPort, const std::string& ifaceIP);
    bool checkPacket(const PacketData* packet) const;

public:
    LiDARReceiver(moodycamel::ConcurrentQueue<std::shared_ptr<PacketData>>& outQ, 
                 const std::atomic<bool>& running);
    ~LiDARReceiver();
    std::shared_ptr<PacketData> produce();
    void run();

    // 用于在Scheule类内监控信息
    std::atomic<uint64_t> total_packets{0};
    std::atomic<uint64_t> full_packets{0};
    std::atomic<uint64_t> partial_packets{0};
};

// Lidar解析类
class LiDARParser {
private:
    moodycamel::ConcurrentQueue<std::shared_ptr<PacketData>>& inputQueue;
    moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& outputQueue;
    const std::atomic<bool>& gRunning;
    int64_t lastLiDAREnqueuedTime_ = 0;

    std::string debug_save_pth = "../data/lidarFrame";

public:
    Telemetry::Welford parse_time_ms;  // 统计解析1s数据用时
    Telemetry::Welford points_per_packet; // 统计解析1s有效点数
    std::atomic<uint64_t> total_packets{0};

    std::atomic<uint64_t>   lidar_non_mono{0}; // 累计次数
    EventRing<16>           lidar_events; // 最近16条样例
    
    LiDARParser(moodycamel::ConcurrentQueue<std::shared_ptr<PacketData>>& inQ,
               moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& outQ,
               const std::atomic<bool>& running);
    ~LiDARParser();
    void consume(std::shared_ptr<PacketData> packet);
    void run();
};


// Lidar Writer
class LiDARWriter {
private:
    moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& inQ_;
    std::string dir_ = "../data/LAZ";
    const std::atomic<bool>& gRunning;
    void savePointCloudToLAZ(const std::vector<Eigen::Vector3d>& points,
                             const std::string& outputFilePath);
public:
    LiDARWriter(moodycamel::ConcurrentQueue<std::shared_ptr<PointCloudData>>& iQ,
                const std::atomic<bool>& running);
    ~LiDARWriter();
    void run();

    std::atomic<uint64_t> total_packets{0}; // 统计写入点云数量 (Schedule Monitor)
    Telemetry::Welford write_time_ms;  // 统计写入点云用时 (Schedule Monitor)
};

