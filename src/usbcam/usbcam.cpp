#include "usbcam/usbcam.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>

bool UsbCam::open(const std::string &config_path)
{
    try
    {
        YAML::Node cfg = YAML::LoadFile(config_path);

        auto intr = cfg["intrinsics"];
        fx_ = intr["fx"].as<double>();
        fy_ = intr["fy"].as<double>();
        cx_ = intr["cx"].as<double>();
        cy_ = intr["cy"].as<double>();

        auto cam_cfg = cfg["camera"];
        int device_id = cam_cfg["device_id"].as<int>(0);
        int width  = cam_cfg["width"].as<int>(640);
        int height = cam_cfg["height"].as<int>(480);
        int fps    = cam_cfg["fps"].as<int>(30);

        cap_.open(device_id);
        if (!cap_.isOpened())
        {
            std::cerr << "[UsbCam] Failed to open device " << device_id << std::endl;
            return false;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap_.set(cv::CAP_PROP_FPS, fps);

        // 读取实际设置的分辨率
        double actual_w = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_h = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        std::cout << "[UsbCam] Opened device " << device_id
                  << " " << actual_w << "x" << actual_h
                  << " @" << fps << "fps" << std::endl;

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
    if (!opened_) return false;
    return cap_.read(color_image);
}

Eigen::Vector3d UsbCam::pixel_to_direction(double u, double v) const
{
    double xc = (u - cx_) / fx_;
    double yc = (v - cy_) / fy_;
    // P_w = (1, -x_c, -y_c)
    return Eigen::Vector3d(1.0, -xc, -yc);
}

double UsbCam::pixel_to_angle_z(double u, double v) const
{
    double xc = (u - cx_) / fx_;
    double yc = (v - cy_) / fy_;
    double denom = std::sqrt(1.0 + xc * xc + yc * yc);
    if (denom < 1e-8) return 0.0;
    double cos_theta = -yc / denom;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    return std::acos(cos_theta);
}
