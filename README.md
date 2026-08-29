<div align="center">

# Camera Bridge

**ROS2 package for Azure Kinect · Intel RealSense · Orbbec · Insight9 · USB-topic YOLO**  
_YOLO object detection accelerated by TensorRT_

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=flat&logo=ros)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?style=flat&logo=ubuntu)](https://ubuntu.com/)
[![CUDA](https://img.shields.io/badge/CUDA-11.x-green?style=flat&logo=nvidia)](https://developer.nvidia.com/cuda-toolkit)
[![TensorRT](https://img.shields.io/badge/TensorRT-8.6-76B900?style=flat&logo=nvidia)](https://developer.nvidia.com/tensorrt)
[![License](https://img.shields.io/badge/License-MIT-yellow?style=flat)]()

</div>

---

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Build](#build)
- [Usage](#usage)
- [Camera Notes](#camera-notes)
- [Project Structure](#project-structure)

---

## Features

- **Multi-Camera Support**: Azure Kinect DK, Intel RealSense, Orbbec Femto Bolt, Insight9, and USB-topic input.
- **TensorRT YOLO Inference**: Shared YOLO backend under `src/utils/yolo`.
- **ROS2 Integration**: Camera/detection nodes publish results through ROS2 topics.
- **Serial Communication**: `serial_node` is available for external device communication.
- **Capture Utilities**: K4A and Orbbec image/video capture helpers are included.
- **Training / Conversion Scripts**: Dataset, ONNX, and TensorRT helper scripts live under `workspace/scripts`.

---

## Prerequisites

### System Requirements

- Ubuntu 22.04
- ROS2 Humble
- NVIDIA GPU with CUDA support

### Required Dependencies

Refer to `requirement.md`. Main dependencies:

| Dependency | Version / Notes |
|------------|-----------------|
| **CUDA** | 11.x or compatible with TensorRT |
| **cuDNN** | Compatible with CUDA version |
| **TensorRT** | 8.6.1.6, default path `/opt/TensorRT-8.6.1.6` |
| **OpenCV** | `sudo apt install libopencv-dev` |
| **PCL** | `sudo apt install libpcl-dev` |
| **ROS2 Humble** | Desktop or base install |
| **Azure Kinect SDK** | Required for K4A targets |
| **Intel RealSense SDK** | Required for RealSense targets |
| **yaml-cpp** | `sudo apt install libyaml-cpp-dev` |
| **serial** | Expected under `/home/pi/workspace/3rd_ws/install/serial` |
| **OrbbecSDK** | Optional; Orbbec targets are skipped if not found |

---

## Installation

### 1. Create ROS2 Workspace

```bash
mkdir -p ~/workspace/camera_ws/src
```

### 2. Place Package

```bash
cd ~/workspace/camera_ws/src
# put this package here as:
# ~/workspace/camera_ws/src/camera_bridge
```

### 3. Install Dependencies

Install dependencies from `requirement.md` and make sure CUDA, cuDNN, and
TensorRT versions are compatible.

---

## Configuration

### TensorRT Path

The current default path in `CMakeLists.txt` is:

```cmake
set(TensorRT_ROOT "/opt/TensorRT-8.6.1.6")
set(TENSORRT_INCLUDE_DIR "${TensorRT_ROOT}/include")
set(TENSORRT_LIB_DIR "${TensorRT_ROOT}/lib")
```

Change this path if TensorRT is installed elsewhere.

### Config Files

| File | Purpose |
|------|---------|
| `K4AConfig.yaml` | Azure Kinect settings |
| `RsConfig.yaml` | RealSense settings |
| `OrbbecConfig.yaml` | Orbbec settings |
| `Insight9Config.yaml` | Insight9 settings |
| `UsbRosConfig.yaml` | Single ROS image topic input for `usb_detect` |

`UsbRosConfig.yaml` only configures `camera_bridge/usb_detect`; it does not
configure any other package.

---

## Build

```bash
cd ~/workspace/camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_bridge
source install/setup.bash
```

If `OrbbecSDK` is missing, CMake prints a warning and skips Orbbec targets.

---

## Usage

### Run ROS2 Executables

| Command | Description |
|---------|-------------|
| `ros2 run camera_bridge k4a_detect` | Azure Kinect detection |
| `ros2 run camera_bridge rs_viewer` | RealSense viewer / detection entry |
| `ros2 run camera_bridge insight9_detect` | Insight9 detection |
| `ros2 run camera_bridge usb_detect` | YOLO detection from one ROS image topic |
| `ros2 run camera_bridge k4a_capture_images` | Capture K4A images |
| `ros2 run camera_bridge serial_node` | Serial communication utility |

Orbbec executables are available only when `OrbbecSDK` is found:

| Command | Description |
|---------|-------------|
| `ros2 run camera_bridge orbbec_detect` | Orbbec detection |
| `ros2 run camera_bridge orbbec_capture_images` | Capture Orbbec images |
| `ros2 run camera_bridge orbbec_capture_videos` | Capture Orbbec videos |

### Run with Launch Files

```bash
# K4A camera + serial communication
ros2 launch camera_bridge k4a_and_serial.launch.py

# Insight9 camera detection
ros2 launch camera_bridge insight9_detect.launch.py

# Insight9 camera + serial communication
ros2 launch camera_bridge insight9_and_serial.launch.py

# USB topic detection only
ros2 launch camera_bridge usb_detect.launch.py

# USB topic detection + serial communication
ros2 launch camera_bridge usb_and_serial.launch.py

# Orbbec camera + serial communication
ros2 launch camera_bridge orbbec_and_serial.launch.py
```

### Helper Scripts

```bash
./start_camera.sh
./start_usbcam.sh
./start_orbbec.sh
```

These scripts belong to `camera_bridge` only.

---

## Camera Notes

### USB Topic Detection

`usb_detect` subscribes to one ROS image topic configured by:

```text
config/UsbRosConfig.yaml
```

This package does not own the external camera publisher for that topic. Make
sure the configured topic exists before starting `usb_detect`.

### Insight9 Topic Mapping

| Function | Topic |
|----------|-------|
| Color image | `/camera/camera/color/image_rect_raw/compressed` |
| Depth image | `/camera/camera/depth/image_rect_raw` |
| Detection result | `/insight9/target_info` |

### Orbbec

Orbbec targets are guarded by `OrbbecSDK_FOUND`. If the SDK is unavailable,
build continues without Orbbec executables.

---

## Project Structure

```text
camera_bridge/
├── config/              # Camera and node configuration files
├── inc/                 # Header files
├── launch/              # ROS2 launch files
├── src/
│   ├── common/          # Shared camera parameters
│   ├── k4a/             # Azure Kinect implementation
│   ├── realsense/       # RealSense implementation
│   ├── orbbec/          # Orbbec implementation
│   ├── insight9/        # Insight9 implementation
│   ├── usbcam/          # USB-topic YOLO path
│   └── utils/           # Serial, TensorRT wrapper, drawing, tracking
│       └── yolo/        # YOLO TensorRT backend
├── workspace/
│   ├── models/          # TensorRT engine files
│   └── scripts/         # Dataset, training, conversion helpers
├── CMakeLists.txt       # Build configuration
├── package.xml          # ROS package manifest
└── requirement.md       # Dependency list and setup notes
```

---

