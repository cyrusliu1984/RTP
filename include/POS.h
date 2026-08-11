#pragma once
#include "type.h"
#include "PosRing.h"

#include <atomic>
#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include <iomanip> 
#include <ctime>
#include <sstream>

#ifdef _WIN32
    #include <windows.h>
    using serial_handle_t = HANDLE;
    using file_handle_t = HANDLE;
#else
    using serial_handle_t = int;
    using file_handle_t = int;
#endif

#include "moodycamel/concurrentqueue.h"

// POS接收类
class POSReceiver {
private:
    int BUFFER_SIZE = 1024 * 20; // 缓冲区大小 bytes
    std::string serialPort;
    int baudrate;

    serial_handle_t serialHandle = (serial_handle_t)-1;
    moodycamel::ConcurrentQueue<uint8_t>& outputQueue;
    moodycamel::ConcurrentQueue<uint8_t>& writerQueue;   // 新增：输出队列：给POSWriter
    const std::atomic<bool>& gRunning;
    std::vector<uint8_t> buffer; // 串口读入缓存
    // std::vector<uint8_t> outputBuffer;  // 复用的输出缓存，避免每次新建

    void setupRealTimePriority();
    serial_handle_t openSerialPort(const std::string& serialPort, int baudrate);

public:
    POSReceiver(moodycamel::ConcurrentQueue<uint8_t>& outQ, 
                moodycamel::ConcurrentQueue<uint8_t>& writerQ,
                const std::atomic<bool>& running);
    ~POSReceiver();
    void run();
};

// POS解析类
class POSParser {
private:
    const std::atomic<bool>& gRunning;
    moodycamel::ConcurrentQueue<uint8_t>& inputQueue;
    PosRing<std::shared_ptr<POSData>>& posRing;
    moodycamel::ConcurrentQueue<std::shared_ptr<MarkData>>& markQueue;
    // PosRing<std::shared_ptr<MarkData>> markRing;
    // ConcurrentSreturn std::vector<uint8_t>();ortedMap<std::shared_ptr<POSData>>& posDataMap;
    // ConcurrentSortedMap<std::shared_ptr<MarkData>>& markDataMap;
    int64_t lastPosEnqueuedTime_ = 0;
    int64_t lastMarkEnqueuedTime_ = 0;
    // moodycamel::ConcurrentQueue<std::shared_ptr<POSData>>& outputPosQueue;
    // moodycamel::ConcurrentQueue<std::shared_ptr<MarkData>>& outputMarkQueue;

    void parseMarkMessage(const uint8_t* data, MarkData& output);
    void parsePosMessage(const uint8_t* data, POSData& output);
    
public:
    POSParser(moodycamel::ConcurrentQueue<uint8_t>& inQ,
             PosRing<std::shared_ptr<POSData>>& outQ1,
             moodycamel::ConcurrentQueue<std::shared_ptr<MarkData>>& outQ2,
             const std::atomic<bool>& running);
    ~POSParser();
    void run();    

    std::atomic<uint64_t> pos_frames{0};  // POS帧数 (Schedule Monitor)
    std::atomic<uint64_t> hik_mark_frames{0}; // Mark帧数 (Schedule Monitor)  mark_frames
    std::atomic<uint64_t> ph_mark_frames{0};  

    std::atomic<uint64_t> pos_non_mono{0}; // 累计次数
    EventRing<16>        pos_events; // 最近16条样例
};

class POSWriter {
private:
    moodycamel::ConcurrentQueue<uint8_t>& inputQueue; 
    const std::atomic<bool>& gRunning; 
    file_handle_t posLogFd = (file_handle_t)-1;
    std::string saveDir_;

    bool write_all(file_handle_t fd, const uint8_t* p, size_t n);

public:
    POSWriter(moodycamel::ConcurrentQueue<uint8_t>& inQ, const std::atomic<bool>& running);
    ~POSWriter();
    void run();
};