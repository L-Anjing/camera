#ifndef REALSENSE_HPP
#define REALSENSE_HPP
#pragma once
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// 点云基础头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

// PCL 滤波头文件
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

// PCL PCA 计算
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>


#include <iostream>
#include <string>
#include <memory>
#include <unistd.h>

#include "common/yolo.hpp"
#include "utils/vision_draw.hpp"
#include "common/colors.hpp"
#include "utils/utils.hpp"
#include "common/camera_params.hpp"


class RealSense
{
private:
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pipeline_profile profile;
    rs2::frameset frameset;
    cv::Mat image_rs_color, image_rs_depth;
    cv::Mat image_rs_infrared_left, image_rs_infrared_right;
    cv::Mat mask;
    int frame_count = 0;
    CameraParams m_params; // 存储当前相机的配置数据

    // 私有构造函数：内部调用 Configuration()
    // 设为私有是为了强制使用 Create_FromFile 或外部显式传参
    RealSense(const CameraParams& params);
public:
    rs2_intrinsics intrinsics_depth;
    rs2_intrinsics intrinsics_color;
    rs2_intrinsics intrinsics_infrared;
    float depth_scale = 0.001f;


    // 工厂函数：普通用，通过路径加载
    static RealSense Create_FromFile(const std::string& config_path);

    // // 配置RealSense对象
    void Configuration();


    // 将摄像头获取的彩色和深度图像转换为OpenCV的cv::Mat格式
    void Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth);

    // 将获取的彩色图像转换为OpenCV的cv::Mat格式
    void Color_to_Cv(cv::Mat &image_cv_color);

    // 将获取的左右红外图像转换为OpenCV的cv::Mat格式
    void Infrared_to_Cv(cv::Mat &image_cv_infrared_left, cv::Mat &image_cv_infrared_right);

    // 将深度图像转换为PCL的点云格式
    void Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

    BoundingBox3D Extract_Object_PointCloud_FromDepth(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const cv::Mat &depth_image,
        yolo::BoxArray objs);

    // 保存指定数量的彩色图像
    void Save_Image(int amount, std::string output_dir);

    // 生成目标点云
    void Mask_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud,
                           const cv::Mat &mask);

    // 抓取图像并转换为cv
    bool Capture(cv::Mat &color, cv::Mat &depth);

    ~RealSense() = default; //析构函数,在对象销毁时自动调用,用于释放资源
};

#endif // REALSENSE_HPP