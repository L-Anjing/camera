<!-- markdownlint-disable -->
# Camera Bridge

A ROS package for Azure Kinect (K4A) and Intel RealSense camera integration with YOLO-based object detection and TensorRT inference acceleration.

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Build](#build)
- [Usage](#usage)

## Features

- **Multi-Camera Support**: Azure Kinect DK and Intel RealSense D435/D455
- **Real-time Object Detection**: YOLO integration with TensorRT acceleration
- **ROS Integration**: Publishes sensor data and detection results via ROS topics
- **Serial Communication**: Supports serial port communication for external devices
## Prerequisites

### System Requirements
- Ubuntu 22.04
- NVIDIA GPU with CUDA support
- ROS2 Humble

### Required Dependencies

Refer to `requirement.md`. All dependencies must be properly installed before building:

- **CUDA** (11.x or higher)
- **cuDNN**
- **TensorRT 8.6.1.6** - Extract to `~/TensorRT-8.6.1.6` or set `TensorRT_ROOT` environment variable
- **OpenCV 4.2** 
- **PCL** - Build from source (recommended for 20.04)
- **VTK 7.1** - Build from source (recommended for 20.04)
- **ROS2 Humble**
- **Azure Kinect SDK** - For K4A camera support
- **Intel RealSense SDK** - For RealSense camera support
- **yaml-cpp** - `sudo apt-get install libyaml-cpp-dev`

Additional dependencies will be reported during build if missing.

## Installation

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
#### Note:  
CUDA ,CUDNN and TensorRT must be *Version Adaptation* 
## Configuration

### TensorRT Path Setup
The default TensorRT path is `$ENV{HOME}/TensorRT-8.6.1.6`. 
Eidt CmakeLists.txt file in 
```
# 手动指定 TensorRT 路径 (基于你之前的安装路径)
set(TensorRT_ROOT "/opt/TensorRT-8.6.1.6")
set(TENSORRT_INCLUDE_DIR "${TensorRT_ROOT}/include")
set(TENSORRT_LIB_DIR "${TensorRT_ROOT}/lib")
```

### Camera Configuration
Edit configuration files in `config/`:
- `K4AConfig.yaml`: Azure Kinect settings
- `RsConfig.yaml`: RealSense settings

## Build

```bash
cd ~/camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_bridge
```

Or use the convenience script:

```bash
cd ~/camera_ws/src/camera_bridge
chmod +x build.sh
./build.sh
```

## Usage

### Source Environment

```bash
source /opt/ros/humble/setup.bash
cd ~/camera_ws
source install/setup.bash
```

### Run ROS2 Nodes

```bash
# Azure Kinect detection
ros2 run camera_bridge k4a_detect

# RealSense viewer
ros2 run camera_bridge rs_viewer

# Insight9 detection
ros2 run camera_bridge insight9_detect

# Capture images
ros2 run camera_bridge k4a_capture_images

# Serial communication
ros2 run camera_bridge k4a_serial_node
```

### Run with Launch File

```bash
# Ensure USB-to-TTL adapter is connected
ros2 launch camera_bridge k4a_and_serial.launch.py
```

## Project Structure
```
camera_bridge/
├── config/           # Configuration files
├── inc/              # Header files
├── src/              # Source files
│   ├── common/       # Common utilities
│   ├── k4a/          # Azure Kinect implementation
│   ├── realsense/    # RealSense implementation
│   ├── utils/        # Helper utilities
│   └── yolo/         # YOLO inference engine
├── launch/           # ROS launch files
├── workspace/        # Dataset and test files
├── CMakeLists.txt    # Build configuration
├── package.xml       # ROS package manifest
└── requirement.md    # Dependency list and some tips
```
