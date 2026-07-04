#include "usbcam/usbcam.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <mutex>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

struct UsbCam::RosInput
{
    std::mutex mutex;
    cv::Mat latest_frame;
    bool has_frame = false;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
};

UsbCam::UsbCam() = default;

UsbCam::~UsbCam() = default;

bool UsbCam::open(const std::string &config_path,
                  std::optional<int> device_id_override,
                  const std::shared_ptr<rclcpp::Node> &ros_node)
{
    try
    {
        // 先加载标定和相机参数
        YAML::Node cfg = YAML::LoadFile(config_path);

        auto intr = cfg["intrinsics"];
        fx_ = intr["fx"].as<double>();
        fy_ = intr["fy"].as<double>();
        cx_ = intr["cx"].as<double>();
        cy_ = intr["cy"].as<double>();

        auto cam_cfg = cfg["camera"];
        device_id_ = device_id_override.value_or(cam_cfg["device_id"].as<int>(0));
        width_ = cam_cfg["width"].as<int>(640);
        height_ = cam_cfg["height"].as<int>(480);
        fps_ = cam_cfg["fps"].as<int>(30);

        rotation_ = Eigen::Matrix3f::Identity();
        translation_.setZero();
        has_transform_ = false;
        if (cfg["transform"] && cfg["transform"]["rotation"] && cfg["transform"]["translation"])
        {
            auto transform = cfg["transform"];
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    rotation_(i, j) = transform["rotation"][i][j].as<float>();
                }
            }
            translation_ << transform["translation"][0].as<float>(),
                transform["translation"][1].as<float>(),
                transform["translation"][2].as<float>();
            has_transform_ = true;
        }

        // source 决定走直读还是 ROS 图像话题
        std::string source = "direct";
        if (cam_cfg["source"])
        {
            source = cam_cfg["source"].as<std::string>();
        }
        std::transform(source.begin(), source.end(), source.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        use_ros_topic_ = (source == "ros" || source == "topic" || source == "ros2");

        if (use_ros_topic_)
        {
            if (!ros_node)
            {
                std::cerr << "[UsbCam] ROS topic mode requires a ROS node" << std::endl;
                return false;
            }

            // ROS 模式下订阅图像话题并缓存最新帧
            ros_node_ = ros_node;
            ros_topic_ = cam_cfg["topic"].as<std::string>("");
            if (ros_topic_.empty())
            {
                std::string camera_name = cam_cfg["camera_name"].as<std::string>("cam");
                ros_topic_ = "/" + camera_name + "/image_raw";
            }

            ros_input_ = std::make_unique<RosInput>();
            ros_input_->subscription = ros_node_->create_subscription<sensor_msgs::msg::Image>(
                ros_topic_,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Image::SharedPtr msg)
                {
                    try
                    {
                        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
                        std::lock_guard<std::mutex> lock(ros_input_->mutex);
                        ros_input_->latest_frame = cv_ptr->image.clone();
                        ros_input_->has_frame = true;
                    }
                    catch (const cv_bridge::Exception &e)
                    {
                        std::cerr << "[UsbCam] cv_bridge error: " << e.what() << std::endl;
                    }
                });

            std::cout << "[UsbCam] Opened ROS topic " << ros_topic_ << std::endl;
        }
        else
        {
            // 直读模式打开 /dev/video*
            cap_.open(device_id_, cv::CAP_V4L2);
            if (!cap_.isOpened())
            {
                std::cerr << "[UsbCam] Failed to open device " << device_id_ << std::endl;
                return false;
            }

            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
            cap_.set(cv::CAP_PROP_FPS, fps_);

            width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
            height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
            fps_ = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));

            std::cout << "[UsbCam] Opened device " << device_id_
                      << " " << width_ << "x" << height_
                      << " @" << fps_ << "fps" << std::endl;
        }

        if (has_transform_)
        {
            std::cout << "[UsbCam] Loaded transform: R and T available" << std::endl;
        }

        opened_ = true;
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[UsbCam] Config error: " << e.what() << std::endl;
        return false;
    }
}

bool UsbCam::read(cv::Mat &color_image)
{
    if (!opened_)
    {
        return false;
    }

    if (use_ros_topic_)
    {
        if (ros_node_)
        {
            rclcpp::spin_some(ros_node_);
        }

        if (!ros_input_)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(ros_input_->mutex);
        if (!ros_input_->has_frame || ros_input_->latest_frame.empty())
        {
            return false;
        }

        color_image = ros_input_->latest_frame.clone();
        return true;
    }

    return cap_.read(color_image);
}

Eigen::Vector3d UsbCam::pixel_to_direction(double u, double v) const
{
    double xc = (u - cx_) / fx_;
    double yc = (v - cy_) / fy_;
    return Eigen::Vector3d(1.0, -xc, -yc);
}

double UsbCam::pixel_to_angle_z(double u, double v) const
{
    double xc = (u - cx_) / fx_;
    double yc = (v - cy_) / fy_;
    double denom = std::sqrt(1.0 + xc * xc + yc * yc);
    if (denom < 1e-8)
    {
        return 0.0;
    }
    double cos_theta = -yc / denom;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    return std::acos(cos_theta);
}

PoseEstimate UsbCam::estimate_pose_from_pattern(const FinalBlockResult &result,
                                                float face_size_m) const
{
    PoseEstimate pose;
    if (face_size_m <= 0.0f || !result.valid())
    {
        return pose;
    }

    // 用 best_pattern 的像素尺寸反推一个粗略深度
    const auto &box = result.best_pattern;
    const float width_px = std::max(1.0f, box.right - box.left);
    const float height_px = std::max(1.0f, box.bottom - box.top);
    const float apparent_px = std::max(1.0f, std::sqrt(width_px * height_px));
    const float center_u = 0.5f * (box.left + box.right);
    const float center_v = 0.5f * (box.top + box.bottom);

    const float z = static_cast<float>(fx_) * face_size_m / apparent_px;
    if (!std::isfinite(z) || z <= 0.0f)
    {
        return pose;
    }

    const float x = (center_u - static_cast<float>(cx_)) * z / static_cast<float>(fx_);
    const float y = (center_v - static_cast<float>(cy_)) * z / static_cast<float>(fy_);
    pose.camera_center = Eigen::Vector3f(x, y, z);
    // 需要的话再转到机器人坐标系
    pose.robot_center = has_transform_
        ? rotation_ * pose.camera_center + translation_
        : pose.camera_center;

    const float range = pose.robot_center.norm();
    if (range > 1e-6f)
    {
        pose.yaw_z = std::atan2(pose.robot_center.y(), pose.robot_center.x());
        pose.axis_angle = std::acos(std::clamp(pose.robot_center.z() / range, -1.0f, 1.0f));
        pose.valid = true;
    }

    return pose;
}
