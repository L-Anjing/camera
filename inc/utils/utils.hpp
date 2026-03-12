#pragma once
#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>
// 点云显示相关头文件
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "utils/block_recognizer.hpp"

struct BoundingBox3D
{
    cv::Point3f min_pt;      // 边界最小点
    cv::Point3f max_pt;      // 边界最大点
    cv::Point3f center;      // 目标中心点，x左右均值，y上下均值，z前方距离
    cv::Vec3f principal_dir; // 目标偏角，yaw(水平偏角),pitch(垂直偏角)，弧度制
    int cls_ID;              
    std::string cls_name;
};
inline float Cal_Dist(float x, float y, float z)
{
    return sqrt(x * x + y * y + z * z);
}