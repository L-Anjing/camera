#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <memory>
#include <optional>
#include <string>

#include "common/pose_estimate.hpp"
#include "utils/block_recognizer.hpp"

namespace rclcpp
{
class Node;
}

// USB 单目相机 (无深度)
class UsbCam
{
public:
    UsbCam();
    ~UsbCam();

    /// 从配置文件加载内参并打开相机
    /// 配置文件格式: YAML, 包含 fx, fy, cx, cy, device_id(可选,默认0)
    /// camera.source 可选: direct / ros / ros2
    /// camera.topic   ROS2 图像话题名, 默认 "/<camera_name>/image_raw"
    bool open(const std::string &config_path,
              std::optional<int> device_id_override = std::nullopt,
              const std::shared_ptr<rclcpp::Node> &ros_node = nullptr);

    /// 获取一帧彩色图
    /// @return true 成功, false 超时或失败
    bool read(cv::Mat &color_image);

    /// 像素坐标 → 世界系 Z 轴夹角 (弧度)
    /// 公式: P_w = (1, -(u-cx)/fx, -(v-cy)/fy), θ = acos(-y_c / sqrt(1+x_c²+y_c²))
    double pixel_to_angle_z(double u, double v) const;

    /// 像素坐标 → 世界系方向向量 (三个分量: 1, -x_c, -y_c)
    Eigen::Vector3d pixel_to_direction(double u, double v) const;

    /// 从融合结果中的 best_pattern 做粗定位 (单目近似)
    PoseEstimate estimate_pose_from_pattern(const FinalBlockResult &result,
                                            float face_size_m) const;

    /// 获取内参
    double fx() const { return fx_; }
    double fy() const { return fy_; }
    double cx() const { return cx_; }
    double cy() const { return cy_; }
    int device_id() const { return device_id_; }
    int width() const { return width_; }
    int height() const { return height_; }
    int fps() const { return fps_; }
    const Eigen::Matrix3f &rotation() const { return rotation_; }
    const Eigen::Vector3f &translation() const { return translation_; }
    bool has_transform() const { return has_transform_; }
    bool uses_ros_topic() const { return use_ros_topic_; }
    const std::string &ros_topic() const { return ros_topic_; }

    bool is_opened() const { return opened_; }

private:
    struct RosInput;

    cv::VideoCapture cap_;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    int device_id_ = 0;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 0;
    Eigen::Matrix3f rotation_ = Eigen::Matrix3f::Identity();
    Eigen::Vector3f translation_ = Eigen::Vector3f::Zero();
    bool has_transform_ = false;
    bool use_ros_topic_ = false;
    std::string ros_topic_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::unique_ptr<RosInput> ros_input_;
    bool opened_ = false;
};
