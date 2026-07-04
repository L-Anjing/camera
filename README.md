<div align="center">

# 🎥 Camera Bridge

**ROS2 package for Azure Kinect · Intel RealSense · Insight9**  
_YOLO object detection accelerated by TensorRT_

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=flat&logo=ros)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?style=flat&logo=ubuntu)](https://ubuntu.com/)
[![CUDA](https://img.shields.io/badge/CUDA-11.x-green?style=flat&logo=nvidia)](https://developer.nvidia.com/cuda-toolkit)
[![TensorRT](https://img.shields.io/badge/TensorRT-8.6-76B900?style=flat&logo=nvidia)](https://developer.nvidia.com/tensorrt)
[![License](https://img.shields.io/badge/License-MIT-yellow?style=flat)]()

</div>

---

## 📋 Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Build](#build)
- [Usage](#usage)

---

## ✨ Features

- **Multi-Camera Support**: Azure Kinect DK, Intel RealSense D435/D455, and Insight9
- **Real-time Object Detection**: YOLO integration with TensorRT acceleration
- **ROS Integration**: Publishes sensor data and detection results via ROS topics
- **Serial Communication**: Supports serial port communication for external devices
- **USB ROS2 Pipeline**: USB 摄像头由 `camera_stream` 发布，`usb_detect` 只订阅图像 topic 做检测

---

## 📦 Prerequisites

### System Requirements
- Ubuntu 22.04
- NVIDIA GPU with CUDA support
- ROS2 Humble

### Required Dependencies

Refer to `requirement.md`. All dependencies must be properly installed before building:

| Dependency | Version / Notes |
|------------|----------------|
| **CUDA** | 11.x or higher |
| **cuDNN** | Compatible with CUDA version |
| **TensorRT** | 8.6.1.6 — extract to `~/TensorRT-8.6.1.6` or set `TensorRT_ROOT` |
| **OpenCV** | 4.2 — `apt install libopencv-dev` |
| **PCL** | `apt install libpcl-dev` |
| **VTK 7.1** | `apt install libvtk7-dev libvtk7-qt-dev` |
| **ROS2 Humble** | Desktop or base install |
| **Azure Kinect SDK** | For K4A camera support |
| **Intel RealSense SDK** | For RealSense camera support |
| **yaml-cpp** | `sudo apt install libyaml-cpp-dev` |

Additional dependencies will be reported during build if missing.

---

## 🚀 Installation

### 1. Create ROS2 Workspace
```bash
mkdir -p ~/camera_ws/src
```

### 2. Clone Repository
```bash
cd ~/camera_ws/src
git clone https://github.com/L-Anjing/camera camera_bridge
```

### 3. Install Dependencies
Refer to `requirement.md` and install each dependency according to the official documentation or Notion notes.

> **⚠️ Note:** CUDA, cuDNN and TensorRT must be **version-compatible** — check the official compatibility matrix.

---

## ⚙️ Configuration

### TensorRT Path Setup
The default TensorRT path is `$ENV{HOME}/TensorRT-8.6.1.6`.  
Edit `CMakeLists.txt`:

```cmake
# 手动指定 TensorRT 路径 (基于你之前的安装路径)
set(TensorRT_ROOT "/opt/TensorRT-8.6.1.6")
set(TENSORRT_INCLUDE_DIR "${TensorRT_ROOT}/include")
set(TENSORRT_LIB_DIR "${TensorRT_ROOT}/lib")
```

### Camera Configuration
Edit configuration files in `config/`:

| File | Purpose |
|------|---------|
| `K4AConfig.yaml` | Azure Kinect settings |
| `RsConfig.yaml` | RealSense settings |
| `Insight9Config.yaml` | Insight9 camera settings (refer to K4AConfig.yaml format) |
| `UsbRosConfig.yaml` | USB ROS2 单路订阅配置（默认订阅 `/cam_left/image_raw`） |

---

## 🔨 Build

```bash
cd ~/camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_bridge
```

---

## ▶️ Usage

### Source Environment

```bash
source /opt/ros/humble/setup.bash
cd ~/camera_ws
source install/setup.bash
```

### Run ROS2 Nodes

| Command | Description |
|---------|-------------|
| `ros2 run camera_bridge k4a_detect` | Azure Kinect detection |
| `ros2 run camera_bridge rs_viewer` | RealSense viewer |
| `ros2 run camera_bridge insight9_detect` | Insight9 detection |
| `ros2 run camera_bridge k4a_capture_images` | Capture images |
| `ros2 run camera_bridge serial_node` | Serial communication |
| `ros2 run camera_bridge usb_detect` | USB camera detection from ROS2 topic |

### Run with Launch File

```bash
# K4A camera + serial communication
ros2 launch camera_bridge k4a_and_serial.launch.py

# Insight9 camera detection
ros2 launch camera_bridge insight9_detect.launch.py

# Insight9 camera + serial communication
ros2 launch camera_bridge insight9_and_serial.launch.py

# USB camera detection only
ros2 launch camera_bridge usb_detect.launch.py

# USB camera + serial communication
ros2 launch camera_bridge usb_and_serial.launch.py

# Orbbec camera + serial communication
ros2 launch camera_bridge orbbec_and_serial.launch.py

# Orbbec watchdog launcher
./start_orbbec.sh
```

---

## 🔗 ROS2 Topic Topology

实际工程建议按下面的方式组织 topic：

| 节点 | 输入 | 输出 |
|------|------|------|
| `camera_stream/camera.launch.py` | `/dev/camera_left`, `/dev/camera_right` | `/cam_left/image_raw`, `/cam_right/image_raw` |
| `camera_bridge/usb_detect` | `/cam_left/image_raw` | `/target_info` |
| `color_detect/color_detect_node` | `/cam_left/image_raw`, `/cam_right/image_raw` | `/color_detect/state` |

`usb_detect` 只使用 `config/UsbRosConfig.yaml`，也就是只订阅一路 ROS2 图像 topic。
相机发布端的设备路径、格式、分辨率、帧率统一由 `camera_stream/config/camera_stream.yaml` 管理。

### USB 相机约定

- 相机发布端统一使用 `camera_stream`
- 默认分辨率为 `1280x720`
- 默认帧率为 `30FPS`
- 默认编码格式为 `MJPEG`
- 检测节点不再负责打开 `/dev/video*`

## 🔌 Insight9 相机集成

基于 ROS2 Topic 的相机驱动，兼容 K4A 模块接口。

### Topic 映射

| 功能 | Topic |
|------|-------|
| 彩色图 | `/camera/camera/color/image_rect_raw/compressed` |
| 深度图 | `/camera/camera/depth/image_rect_raw` |
| 检测结果 | `/insight9/target_info` |

---

## 📁 Project Structure

```
camera_bridge/
├── config/           # Configuration files
├── inc/              # Header files
├── src/              # Source files
│   ├── common/       # Common utilities
│   ├── k4a/          # Azure Kinect implementation
│   ├── insight9/     # Insight9 camera implementation
│   ├── realsense/    # RealSense implementation
│   ├── utils/        # Helper utilities
│   └── yolo/         # YOLO inference engine
├── launch/           # ROS launch files
├── workspace/        # Dataset and test files
├── CMakeLists.txt    # Build configuration
├── package.xml       # ROS package manifest
└── requirement.md    # Dependency list and some tips
```
