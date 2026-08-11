# RTPCP — 机载激光雷达实时处理模块 (Windows & Linux 跨平台版)

> 这是项目的完整说明文档，包含环境依赖、vcpkg 包安装、Windows/Linux 跨平台编译步骤、运行示例以及代码模块说明。

---

## 一、项目概述

RTPCP 是一个用于接收、解码、投影/配准并传输雷达/激光点云数据的 C++17 跨平台工程。
核心包含：
- **雷达数据解码与光路追踪 (VRT)**：接收高频 UDP 数据并解码为点云。
- **点云实时配准**：利用 POS 时间同步与插值，完成传感器坐标系 -> IMU 姿态坐标系 -> WGS84 大地/UTM/ENU 坐标系转换。
- **LAZ 压缩存储**：基于 LASzip 实时将点云写入 `.laz` 压缩文件。
- **文件监控与 TCP 数据传输**：自动检测新生成的 LAZ 文件并在大小稳定后通过 TCP 协议传输至目标服务器。
- **跨平台支持**：全新重构，完全**移除海康威视 MVS SDK 依赖**，全面兼容 **Windows (MSVC)** 与 **Linux (GCC/Clang)**。

---

## 二、目录结构

```text
RTP/
├── CMakeLists.txt     # 跨平台 CMake 构建脚本
├── config/            # YAML 配置文件 (config.yaml, default.yaml)
├── doc/               # 项目文档与笔记
├── include/           # 头文件目录
├── src/               # 模块实现源文件 (跨平台适配)
├── test/main.cc       # 程序主入口 (单实例控制/归档/调度)
├── lib/               # 静态库目录 (跨平台适配)
└── readme.md          # 本说明文档
```

模块对应关系：
- `src/RayTracing.cc` / `include/RayTracing.h` — 雷达信号解码与 VRT 光线追踪。
- `src/Register.cc` / `include/Register.h` — 多线程点云配准 (WGS84 / UTM 投影与 POS 插值)。
- `src/Transmit.cc` / `include/Transmit.h` — LAZ 文件稳定检测与 TCP 数据传输 (Winsock / POSIX Socket 适配)。
- `src/POS.cc` / `include/POS.h` — POS 串口接收 (Win32 API / Termios 适配)、二进制解析与 PosRing 环形缓冲区。
- `src/LiDAR.cc` / `include/LiDAR.h` — UDP 雷达数据接收、解析与 LAZ 写入。
- `src/Logger.cc` / `include/Logger.h` — spdlog 多日志器封装。
- `src/utils.cc` / `include/utils.h` — 时间转换、CRC32 校验与通用工具函数。

---

## 三、依赖库与 vcpkg 安装命令

项目依赖的核心第三方库包括：
1. **PCL** (Point Cloud Library)
2. **OpenCV**
3. **Eigen3**
4. **yaml-cpp**
5. **LASzip**
6. **spdlog**

---

### 1. Windows 平台 vcpkg 安装命令

在 Windows 下推荐使用 Microsoft 官方的 [vcpkg](https://github.com/microsoft/vcpkg) 管理依赖：

```powershell
# 1. 克隆 vcpkg (若未安装)
git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
C:\vcpkg\bootstrap-vcpkg.bat

# 2. 安装 RTPCP 所需的 x64-windows 依赖包
C:\vcpkg\vcpkg install pcl opencv eigen3 yaml-cpp laszip spdlog --triplet x64-windows
```

---

### 2. Linux 平台 (Ubuntu/Debian) vcpkg / 包管理器安装命令

在 Linux 下可以通过系统 `apt` 直接安装，也可以通过 `vcpkg` 安装：

#### 方案 A：使用 vcpkg (推荐)
```bash
# 安装 vcpkg 依赖包
./vcpkg/vcpkg install pcl opencv eigen3 yaml-cpp laszip spdlog --triplet x64-linux
```

#### 方案 B：使用系统 apt 原生安装
```bash
sudo apt update
sudo apt install build-essential cmake libpcl-dev libopencv-dev libeigen3-dev \
                 libyaml-cpp-dev liblaszip-dev libspdlog-dev
```

---

## 四、编译步骤 (CMake)

### 1. Windows 平台编译 (MSVC / Visual Studio)

打开 `Developer Command Prompt for VS` 或 PowerShell：

```powershell
# 进入项目根目录
cd RTP

# 创建并进入 build 目录，指定 vcpkg 工具链文件
mkdir build
cd build

# 配置 CMake (请替换 C:/vcpkg 为你的实际 vcpkg 安装路径)
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"

# 执行编译
cmake --build . --config Release

# 可执行文件将生成于 bin/Release/RTPCP.exe
```

---

### 2. Linux 平台编译 (GCC / Clang)

在终端中执行：

```bash
cd RTP

mkdir -p build
cd build

# 使用系统库构建：
cmake .. -DCMAKE_BUILD_TYPE=Release

# 或使用 vcpkg 构建：
# cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$HOME/vcpkg/scripts/buildsystems/vcpkg.cmake"

make -j$(nproc)

# 可执行文件生成于 bin/RTPCP
```

---

## 五、运行与配置说明

### 1. 运行方式

- **Windows**:
  进入工程根目录，在 PowerShell 或 CMD 中运行：
  ```cmd
  .\bin\RTPCP.exe
  ```

- **Linux**:
  进入工程根目录，运行：
  ```bash
  sudo ./bin/RTPCP
  ```

---

### 2. 配置文件 (`config/default.yaml`)

配置文件位于 `config/` 目录下，主要控制选项：
- **`LIDAR.RECEIVER.IP` / `PORT`**：LiDAR 网卡 IP 与 UDP 端口。
- **`POS.RECEIVER.COMPORT` / `BAUDRATE`**：POS 串口名称（Windows 如 `COM3`，Linux 如 `/dev/ttyUSB0`）与波特率。
- **`TRANSMIT`**：目标接收服务器的 IP (`192.168.5.120`) 与端口 (`8888`)。

---

## 六、跨平台适配技术总结

1. **移除相机与 MVS 依赖**：彻底剥离 Hikvision MVS Camera SDK 链接逻辑，清理日志器与调度线程。
2. **Socket 屏蔽层**：自动识别 Windows (`Winsock2` / `ws2_32.lib`) 与 Linux (`POSIX Sockets`)，统一使用 `socket_t` 接口。
3. **串口兼容**：Windows 环境自动转换为 Win32 Comm API (`CreateFileA` / `DCB` / `ReadFile`)，Linux 下保留 POSIX `termios` 模式。
4. **单实例锁**：Windows 环境使用 Windows 命名互斥量 (`CreateMutexA`) 防止重复运行；Linux 下维持 `/var/run/rtpcp.pid` 与 `flock` 文件锁。
5. **线程优先级**：Windows 使用 `SetThreadPriority`；Linux 使用 `pthread_setschedparam` 与 `SCHED_FIFO`。
