# Insight9 相机集成

基于ROS2 Topic的相机驱动，兼容K4A模块接口。

## 快速开始

### 编译
```bash
colcon build --packages-select camera_bridge
```

### 运行
```bash
# 仅Insight9检测
ros2 run camera_bridge insight9_detect_ros

# 或使用launch文件（包含Serial节点）
ros2 launch camera_bridge insight9_and_serial.launch.py
```

## Topic映射

| 功能 | Topic |
|------|-------|
| 彩色图 | `/camera/camera/color/image_rect_raw/compressed` |
| 深度图 | `/camera/camera/depth/image_rect_raw` |
| 检测结果 | `/insight9/target_info` |

## 配置文件

`config/Insight9Config.yaml` - 相机参数配置（参照K4AConfig.yaml格式）

## 文件结构

```
├── inc/insight9/camera_insight9.hpp
├── src/insight9/
│   ├── camera_insight9.cpp
│   └── insight9_detect.cpp
├── config/Insight9Config.yaml
└── launch/
    ├── insight9_detect.launch.py
    └── insight9_and_serial.launch.py
```
