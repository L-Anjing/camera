#ifndef MAIN_HPP
#define MAIN_HPP

#include <iostream>
#include <opencv2/opencv.hpp>



#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
struct BoundingBox3D
{
    cv::Point3f min_pt;      // 边界最小点
    cv::Point3f max_pt;      // 边界最大点
    cv::Point3f center;      // 目标中心点
    cv::Vec3f principal_dir; // 目标主方向（杆，枪尖使用）
    std::string cls_name;    // 目标类别（箱子,手掌，枪尖等）
};

#endif // MAIN_HPP
