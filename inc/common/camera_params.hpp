#pragma once
#include <string>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

// 定义相机模式
enum class CameraMode
{
    DEFAULT,      // 彩色 + 深度
    INFRARED_ONLY // 左右红外
};

struct CameraParams
{
    int width, height, fps;                // 相机分辨率和帧率
    CameraMode mode = CameraMode::DEFAULT; // 相机模式
    Eigen::Matrix3f rotation;              // 3x3 旋转矩阵
    Eigen::Vector3f translation;           // 3x1 平移向量.即相机坐标系到机器人坐标系的平移
    float min_dist;                        // 最小距离阈值
    float max_dist;                        // 最大距离阈值

    static CameraParams LoadFromFile(const std::string &filepath);
};
