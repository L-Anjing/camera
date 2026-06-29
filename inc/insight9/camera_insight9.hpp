#ifndef CAMERA_INSIGHT9_HPP
#define CAMERA_INSIGHT9_HPP

#include "common/yolo.hpp"
#include "common/camera_params.hpp"
#include "utils/vision_draw.hpp"
#include "common/colors.hpp"
#include "utils/utils.hpp"

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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <mutex>

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

class Insight9
{
private:
    // ROS2 节点指针
    std::shared_ptr<rclcpp::Node> node_;

    // 图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_sub_;

    // 相机内参（从配置文件加载）
    CameraIntrinsics color_intrinsics_;
    CameraIntrinsics depth_intrinsics_;

    // 图像缓冲区（同步存储）
    cv::Mat current_color_image_;
    cv::Mat current_depth_image_;
    
    // 分辨率跟踪（从config初始化，ROS2自动更新）
    int color_width_;
    int color_height_;
    int depth_width_;
    int depth_height_;
    
    // 同步互斥锁
    mutable std::mutex color_mutex_;
    mutable std::mutex depth_mutex_;

    // 数据有效标志
    bool color_available_ = false;
    bool depth_available_ = false;
    
    // 内参初始化标志（防止重复打印）
    bool color_intrinsics_initialized_ = false;
    bool depth_intrinsics_initialized_ = false;

    // 相机参数
    CameraParams m_params;

    // 私有构造函数
    Insight9(const CameraParams& params);

public:
    // 工厂函数：通过配置文件路径创建
    static std::unique_ptr<Insight9> Create_FromFile(const std::string& config_path,
                                    std::shared_ptr<rclcpp::Node> node);

    // 禁用移动和复制
    Insight9(Insight9&& other) noexcept = delete;
    Insight9& operator=(Insight9&& other) noexcept = delete;
    
    // 禁用复制
    Insight9(const Insight9&) = delete;
    Insight9& operator=(const Insight9&) = delete;

    // 初始化 ROS2 订阅
    void Initialize_ROS2_Subscribers(std::shared_ptr<rclcpp::Node> node);

    // 获取相机内参
    CameraIntrinsics get_color_intrinsics() const;
    CameraIntrinsics get_depth_intrinsics() const;

    // 获取同步的彩色图和深度图
    bool Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth);

    // 3D 检测：从 block 结果到点云 + 3D 包围盒
    BoundingBox3D Value_Block_to_Pcl(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const cv::Mat &depth_image,
        const FinalBlockResult &objs
    );

    // 全局点云：从深度图生成完整点云
    void Value_Depth_to_Pcl(
        const cv::Mat &depth_image,
        pcl::PointCloud<pcl::PointXYZ> &cloud
    );

    ~Insight9() = default;
};

#endif
