#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>

// USB 单目相机 (无深度)
class UsbCam
{
public:
    UsbCam() = default;

    /// 从配置文件加载内参并打开相机
    /// 配置文件格式: YAML, 包含 fx, fy, cx, cy, device_id(可选,默认0)
    bool open(const std::string &config_path);

    /// 获取一帧彩色图
    /// @return true 成功, false 超时或失败
    bool read(cv::Mat &color_image);

    /// 像素坐标 → 世界系 Z 轴夹角 (弧度)
    /// 公式: P_w = (1, -(u-cx)/fx, -(v-cy)/fy), θ = acos(-y_c / sqrt(1+x_c²+y_c²))
    double pixel_to_angle_z(double u, double v) const;

    /// 像素坐标 → 世界系方向向量 (三个分量: 1, -x_c, -y_c)
    Eigen::Vector3d pixel_to_direction(double u, double v) const;

    /// 获取内参
    double fx() const { return fx_; }
    double fy() const { return fy_; }
    double cx() const { return cx_; }
    double cy() const { return cy_; }

    bool is_opened() const { return opened_; }

private:
    cv::VideoCapture cap_;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    bool opened_ = false;
};
