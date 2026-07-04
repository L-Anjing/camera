#ifndef CAMERA_ORBBEC_HPP
#define CAMERA_ORBBEC_HPP

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
#include <pcl/segmentation/sac_segmentation.h>

#include <libobsensor/ObSensor.hpp>

#include <Eigen/Dense>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unistd.h>

struct OrbbecCameraIntrinsics
{
    float fx;
    float fy;
    float cx;
    float cy;
};

class Orbbec
{
private:
    std::shared_ptr<ob::Pipeline> pipeline;
    std::shared_ptr<ob::Config> config;
    std::shared_ptr<ob::Device> device;
    OBCameraParam camera_param{};

    int frame_count = 0;
    int device_count = 0;

    OrbbecCameraIntrinsics color_intrinsics_{};
    OrbbecCameraIntrinsics depth_intrinsics_{};

    CameraParams m_params;

    std::atomic_bool imu_started_{false};
    mutable std::mutex gyro_mutex_;
    Eigen::Vector3f last_gyro_ = Eigen::Vector3f::Zero();

    Orbbec(const CameraParams &params);

    std::shared_ptr<ob::FrameSet> wait_latest_frames(int timeout_ms = 1000);
    cv::Mat color_frame_to_bgr(const std::shared_ptr<ob::ColorFrame> &color_frame) const;
    cv::Mat depth_frame_to_mat(const std::shared_ptr<ob::DepthFrame> &depth_frame) const;

public:
    static Orbbec Create_FromFile(const std::string &config_path);

    Orbbec(Orbbec &&other) noexcept = default;
    Orbbec &operator=(Orbbec &&other) noexcept = default;

    Orbbec(const Orbbec &) = delete;
    Orbbec &operator=(const Orbbec &) = delete;

    bool Open();
    void Installed_Count();
    void Configuration();

    OrbbecCameraIntrinsics get_color_intrinsics() const { return color_intrinsics_; }
    OrbbecCameraIntrinsics get_depth_intrinsics() const { return depth_intrinsics_; }

    const Eigen::Matrix3f &get_rotation() const { return m_params.rotation; }
    const Eigen::Vector3f &get_translation() const { return m_params.translation; }

    void Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth);
    void Color_to_Cv(cv::Mat &image_cv_color);
    void Depth_to_Cv(cv::Mat &image_cv_depth);

    void Color_With_Mask(cv::Mat &image_cv_color, const yolo::BoxArray &objs);
    void Depth_With_Mask(cv::Mat &image_cv_depth, const yolo::BoxArray &objs);

    BoundingBox3D Value_Block_to_Pcl(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const cv::Mat &depth_image,
        const FinalBlockResult &objs);

    void Value_Depth_to_Pcl(
        const cv::Mat &depth_to_color,
        pcl::PointCloud<pcl::PointXYZ> &cloud);

    Eigen::Vector3f compute_precise_center(
        const cv::Mat &depth_image,
        const yolo::Box &block_box,
        const yolo::Box &face_box,
        Eigen::Vector3f *out_normal_cam = nullptr) const;

    void start_imu();
    Eigen::Vector3f get_gyro();

    void Save_Image(int amount, std::string output_dir);

    void record_videos(const std::string &output_path_prefix, const std::string &obj);

    void capture_images(const std::string &output_path_prefix,
                        const std::string &obj);

    ~Orbbec()
    {
        try
        {
            if (pipeline)
            {
                pipeline->stop();
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Stop Orbbec pipeline failed: " << e.what() << std::endl;
        }
    }
};

#endif
