#ifndef REALSENSE_HPP
#define REALSENSE_HPP

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>


#include <iostream>
#include <string>
#include <memory>
#include <unistd.h>

#include "yolo.hpp"
#include "main.hpp"

#define COUT_RED_START std::cout << "\033[1;31m";
#define COUT_GREEN_START std::cout << "\033[1;32m";
#define COUT_YELLOW_START std::cout << "\033[1;33m";
#define COUT_BLUE_START std::cout << "\033[1;34m";
#define COUT_PURPLE_START std::cout << "\033[1;35m";
#define COUT_CYAN_START std::cout << "\033[1;36m";
#define COUT_WHITE_START std::cout << "\033[1;37m";
#define COUT_COLOR_END std::cout << "\033[0m";

#undef MIN_DISTANCE
#define MIN_DISTANCE 0.1


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

    RealSense() = default;

    void Configuration_Default();

    void Configuration_Infrared_Only();

public:
    rs2_intrinsics intrinsics_depth;
    rs2_intrinsics intrinsics_color;
    rs2_intrinsics intrinsics_infrared;
    float depth_scale = 0.001f;

    // 获取彩色数据流和深度数据流的帧数
    static RealSense Create_Default();

    // 创建一个只包含红外相机的RealSense对象
    static RealSense Create_Infrared_Only();

    // 配置RealSense对象
    void Configuration();

    // 将摄像头获取的彩色和深度图像转换为OpenCV的cv::Mat格式
    void Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth);

    // 将获取的彩色图像转换为OpenCV的cv::Mat格式
    void Color_to_Cv(cv::Mat &image_cv_color);

    // 将获取的左右红外图像转换为OpenCV的cv::Mat格式
    void Infrared_to_Cv(cv::Mat &image_cv_infrared_left, cv::Mat &image_cv_infrared_right);

    void Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray objs);

    // 过滤或者标记出特定区域的深度信息
    void Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray objs);

    // 将深度图像转换为PCL的点云格式
    void Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

    // 掩码的深度图像转换为点云格式
    BoundingBox3D Value_Mask_to_Pcl(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const cv::Mat &depth_image,
        yolo::BoxArray objs);

    // 保存指定数量的彩色图像
    void Save_Image(int amount, std::string output_dir);

    // 生成目标点云
    void Mask_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud,
                           const cv::Mat &mask);

    ~RealSense() = default;
};

#endif // REALSENSE_HPP