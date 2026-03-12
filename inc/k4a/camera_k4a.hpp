#ifndef CAMREA_K4A_HPP
#define CAMREA_K4A_HPP

#include "common/yolo.hpp"
#include "utils/vision_draw.hpp"
#include "common/colors.hpp"
#include "utils/utils.hpp"
#include "common/camera_params.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

#include <k4a/k4a.hpp>
#include <k4a/k4a.h>

#include <iostream>
#include <unistd.h>
#include <memory>
#include <string>



struct CameraIntrinsics
{
    float fx;
    float fy;
    float cx;
    float cy;
};
enum class DetectionSemantic
{
    YOLO, // 原始 YOLO 输出
    BLOCK // block_recognizer 融合结果
};

class K4a
{
private:
    // 设备与SDK对象
    k4a::device device;
    k4a_device_configuration_t config;
    k4a::capture capture;

    // 标定&转换
    k4a::calibration k4aCalibration;
    k4a::transformation k4aTransformation;

    // 设备状态
    int frame_count = 0;
    int device_count;

    k4a_calibration_camera_t depth_intrinsics;
    k4a_calibration_camera_t color_intrinsics;

    k4a::image image_k4a_color, image_k4a_depth, image_k4a_infrared;
    k4a::image image_k4a_depth_to_color;

    // 存储相机的配置数据
    CameraParams m_params;

    // 私有构造函数：内部调用 Configuration()
    K4a(const CameraParams& params);

public:
    // 工厂函数：通过路径加载配置文件
    static K4a Create_FromFile(const std::string& config_path);

    // 移动语义
    K4a(K4a&& other) noexcept = default;
    K4a& operator=(K4a&& other) noexcept = default;
    
    // 禁用复制
    K4a(const K4a&) = delete;
    K4a& operator=(const K4a&) = delete;

    // 设备初始化
    bool Open();
    void Installed_Count();

    // 配置K4a对象
    void Configuration();

    CameraIntrinsics get_color_intrinsics() const;

    CameraIntrinsics get_depth_intrinsics() const;

    void Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth);

    void Color_to_Cv(cv::Mat &iamge_cv_color);

    void Depth_to_Cv(cv::Mat &image_cv_depth);

    void Color_With_Mask(cv::Mat &iamge_cv_color, const yolo::BoxArray &objs);

    void Depth_With_Mask(cv::Mat &image_cv_depth, const yolo::BoxArray &objs);

    // 目标点云
    BoundingBox3D Value_Block_to_Pcl(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const cv::Mat &depth_image,
        const FinalBlockResult &objs
    );

    // 全局点云
    void Value_Depth_to_Pcl(
        const k4a::image &depth_to_color,
        pcl::PointCloud<pcl::PointXYZ> &cloud);

    void Save_Image(int amount, std::string output_dir);

    void record_videos(const std::string &output_path_prefix, const std::string &obj);

    void capture_images(const std::string &output_path_prefix,
                        const std::string &obj);

    ~K4a()
    {
        image_k4a_depth.reset();
        image_k4a_color.reset();
        capture.reset();
        device.close();
    }
};

#endif