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
- **Standalone Mode**: Can run without ROS for testing and development

## Prerequisites

### System Requirements
- Ubuntu 20.04 or Ubuntu 22.04
- NVIDIA GPU with CUDA support
- ROS Noetic or ROS Humble

### Required Dependencies

Refer to `requirement.md`. All dependencies must be properly installed before building:

- **CUDA** (11.x or higher)
- **cuDNN**
- **TensorRT 8.6.1.6** - Extract to `~/TensorRT-8.6.1.6` or set `TensorRT_ROOT` environment variable
- **OpenCV 4.2** 
- **PCL** - Build from source (recommended for 20.04)
- **VTK 7.1** - Build from source (recommended for 20.04)
- **ROS Noetic or ROS Humble** - For ROS features
- **Azure Kinect SDK** - For K4A camera support
- **Intel RealSense SDK** - For RealSense camera support
- **yaml-cpp** - `sudo apt-get install libyaml-cpp-dev`

Additional dependencies will be reported during build if missing.

## Installation

### 1. Create ROS Workspace
```bash
mkdir -p ~/camera_ws/src && cd ~/camera_ws/src
catkin_init_workspace
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

### Build with Script (Recommended)

Use the new script in this package:

```bash
cd ~/camera_ws/src/camera_bridge
chmod +x build.sh
```

### ROS1 Build

```bash
source /opt/ros/noetic/setup.bash
cd ~/camera_ws/src/camera_bridge
./build.sh ROS1
```

### ROS2 Build

```bash
source /opt/ros/humble/setup.bash
cd ~/camera_ws/src/camera_bridge
./build.sh ROS2
```

If you use ROS2 workspace tools (`colcon`), build from workspace root:

```bash
cd ~/camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_bridge
```

`build.sh` supports only two arguments:
- `./build.sh ROS1`
- `./build.sh ROS2` 

## Usage

### Source Environment

#### ROS1

```bash
source /opt/ros/noetic/setup.bash
cd ~/camera_ws
source devel/setup.bash
```

#### ROS2

```bash
source /opt/ros/<your_ros2_distro>/setup.bash
cd ~/camera_ws
source install/setup.bash
```

### Run Standalone Programs
```bash
# Azure Kinect detection (no ROS)
./devel/lib/camera_bridge/k4a_detect

# RealSense viewer (no ROS)
./devel/lib/camera_bridge/rs_viewer

# Capture images
./devel/lib/camera_bridge/capture_images

# Record videos
./devel/lib/camera_bridge/record_videos
```

### Run ROS Nodes

**ROS1 single node:**
```bash
# Azure Kinect with ROS
rosrun camera_bridge k4a_detect_ros

# RealSense with ROS
rosrun camera_bridge rs_viewer_ros
```

**ROS2 single node:**
```bash
# Azure Kinect with ROS2
ros2 run camera_bridge k4a_detect_ros

# RealSense with ROS2
ros2 run camera_bridge rs_viewer_ros
```

**ROS1 launch file (camera + serial communication):**
```bash
# Ensure USB-to-TTL adapter is connected
roslaunch camera_bridge k4a_and_serial.launch
```

**ROS2 launch file (camera + serial communication):**
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
