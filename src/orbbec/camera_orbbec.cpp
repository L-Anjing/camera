#include "orbbec/camera_orbbec.hpp"
#include "utils/myinfer.hpp"
#include "utils/utils.hpp"

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

using namespace std;

Orbbec::Orbbec(const CameraParams &params) : m_params(params)
{
    Installed_Count();
    if (!Open())
    {
        throw std::runtime_error("Failed to open Orbbec device");
    }
    this->Configuration();
}

Orbbec Orbbec::Create_FromFile(const std::string &config_path)
{
    CameraParams params = CameraParams::LoadFromFile(config_path);
    return Orbbec(params);
}

bool Orbbec::Open()
{
    try
    {
        pipeline = std::make_shared<ob::Pipeline>();
        device = pipeline->getDevice();
        COUT_GREEN_START;
        cout << "Open Orbbec Device Success!" << endl;
        COUT_COLOR_END;
        return true;
    }
    catch (const std::exception &e)
    {
        COUT_RED_START;
        cerr << "Open Orbbec Device Error: " << e.what() << endl;
        COUT_COLOR_END;
        return false;
    }
}

void Orbbec::Installed_Count()
{
    try
    {
        ob::Context ctx;
        auto device_list = ctx.queryDeviceList();
        device_count = static_cast<int>(device_list->deviceCount());
    }
    catch (const std::exception &e)
    {
        device_count = 0;
        cerr << "Query Orbbec device count failed: " << e.what() << endl;
    }

    if (device_count == 0)
    {
        COUT_RED_START
        cout << "No Orbbec Device Found!" << endl;
        COUT_COLOR_END
    }
    else
    {
        COUT_BLUE_START
        cout << "Find " << device_count << " Orbbec Device(s)" << endl;
        COUT_COLOR_END
    }
}

void Orbbec::Configuration()
{
    config = std::make_shared<ob::Config>();

    auto color_profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    auto color_profile = color_profiles->getVideoStreamProfile(
        m_params.width,
        m_params.height,
        OB_FORMAT_RGB,
        m_params.fps);
    config->enableStream(color_profile);

    auto depth_profiles = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
    std::shared_ptr<ob::VideoStreamProfile> depth_profile;
    try
    {
        depth_profile = depth_profiles->getVideoStreamProfile(
            m_params.width,
            m_params.height,
            OB_FORMAT_Y16,
            m_params.fps);
    }
    catch (const std::exception &)
    {
        depth_profile = depth_profiles->getVideoStreamProfile(
            OB_WIDTH_ANY,
            OB_HEIGHT_ANY,
            OB_FORMAT_Y16,
            m_params.fps);
    }
    config->enableStream(depth_profile);

    // Femto Bolt supports depth-to-color alignment in Orbbec SDK. Use software
    // alignment for portability; hardware alignment can be enabled later if needed.
    config->setAlignMode(ALIGN_D2C_SW_MODE);
    pipeline->start(config);

    camera_param = pipeline->getCameraParam();
    color_intrinsics_ = {
        camera_param.rgbIntrinsic.fx,
        camera_param.rgbIntrinsic.fy,
        camera_param.rgbIntrinsic.cx,
        camera_param.rgbIntrinsic.cy};
    depth_intrinsics_ = {
        camera_param.depthIntrinsic.fx,
        camera_param.depthIntrinsic.fy,
        camera_param.depthIntrinsic.cx,
        camera_param.depthIntrinsic.cy};

    start_imu();

    COUT_GREEN_START;
    cout << "Start Orbbec Device Success!" << endl;
    COUT_COLOR_END;
}

std::shared_ptr<ob::FrameSet> Orbbec::wait_latest_frames(int timeout_ms)
{
    std::shared_ptr<ob::FrameSet> frameset;
    while (pipeline->waitForFrames(0))
    {
    }
    frameset = pipeline->waitForFrames(timeout_ms);
    return frameset;
}

cv::Mat Orbbec::color_frame_to_bgr(const std::shared_ptr<ob::ColorFrame> &color_frame) const
{
    if (!color_frame)
    {
        return {};
    }

    cv::Mat color_rgb(
        color_frame->height(),
        color_frame->width(),
        CV_8UC3,
        (void *)color_frame->data());

    cv::Mat color_bgr;
    cv::cvtColor(color_rgb, color_bgr, cv::COLOR_RGB2BGR);
    return color_bgr.clone();
}

cv::Mat Orbbec::depth_frame_to_mat(const std::shared_ptr<ob::DepthFrame> &depth_frame) const
{
    if (!depth_frame)
    {
        return {};
    }

    cv::Mat depth(
        depth_frame->height(),
        depth_frame->width(),
        CV_16U,
        (void *)depth_frame->data());
    return depth.clone();
}

void Orbbec::Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth)
{
    auto frameset = wait_latest_frames(1000);
    if (!frameset)
    {
        return;
    }

    image_cv_color = color_frame_to_bgr(frameset->colorFrame());
    image_cv_depth = depth_frame_to_mat(frameset->depthFrame());
}

void Orbbec::Color_to_Cv(cv::Mat &image_cv_color)
{
    auto frameset = wait_latest_frames(1000);
    if (!frameset)
    {
        return;
    }

    image_cv_color = color_frame_to_bgr(frameset->colorFrame());
}

void Orbbec::Depth_to_Cv(cv::Mat &image_cv_depth)
{
    auto frameset = wait_latest_frames(1000);
    if (!frameset)
    {
        return;
    }

    image_cv_depth = depth_frame_to_mat(frameset->depthFrame());
}

void Orbbec::Save_Image(int amount, std::string output_dir)
{
    if (frame_count >= amount)
    {
        return;
    }

    cv::Mat image_saved;
    Color_to_Cv(image_saved);
    if (image_saved.empty())
    {
        return;
    }

    string filename = output_dir + "obj_" + to_string(frame_count) + ".png";
    if (cv::imwrite(filename, image_saved))
    {
        COUT_GREEN_START;
        cout << "Save obj_" << frame_count << ".png Success!" << endl;
        COUT_COLOR_END;
        frame_count++;
    }
    else
    {
        COUT_RED_START;
        cout << "Save error!" << endl;
        COUT_COLOR_END;
    }
    usleep(50000);
}

void Orbbec::Color_With_Mask(cv::Mat &image_cv_color, const yolo::BoxArray &objs)
{
    vision::draw_yolo_detections(image_cv_color, objs);
}

void Orbbec::Depth_With_Mask(cv::Mat &image_cv_depth, const yolo::BoxArray &objs)
{
    vision::draw_yolo_detections(image_cv_depth, objs);
}

BoundingBox3D Orbbec::Value_Block_to_Pcl(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const cv::Mat &depth_image,
    const FinalBlockResult &objs)
{
    cloud->clear();

    BoundingBox3D bbox;
    bbox.min_pt = cv::Point3f(FLT_MAX, FLT_MAX, FLT_MAX);
    bbox.max_pt = cv::Point3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    bbox.center = cv::Point3f(0, 0, 0);
    bbox.principal_dir = cv::Vec3f(0, 0, 0);
    bbox.cls_ID = static_cast<int>(objs.block_class);
    bbox.cls_name = block_class_name(objs.block_class);

    const yolo::Box &obj = objs.best_pattern;
    size_t valid_points = 0;
    const float depth_scale = 0.001f;

    OrbbecCameraIntrinsics intr = get_color_intrinsics();
    const float fx = intr.fx;
    const float fy = intr.fy;
    const float cx = intr.cx;
    const float cy = intr.cy;

    const int cols = depth_image.cols;
    const int rows = depth_image.rows;
    const int roi_left = std::max(0, static_cast<int>(obj.left));
    const int roi_top = std::max(0, static_cast<int>(obj.top));
    const int roi_right = std::min(cols, static_cast<int>(obj.right));
    const int roi_bottom = std::min(rows, static_cast<int>(obj.bottom));

    size_t zero_depth_count = 0;
    size_t out_of_range_count = 0;

    for (int py = roi_top; py < roi_bottom; ++py)
    {
        for (int px = roi_left; px < roi_right; ++px)
        {
            float depth_mm = depth_image.at<uint16_t>(py, px);
            float depth_value = depth_mm * depth_scale;

            if (depth_mm == 0)
            {
                ++zero_depth_count;
                continue;
            }

            if (depth_value <= m_params.min_dist || depth_value > m_params.max_dist)
            {
                ++out_of_range_count;
                continue;
            }

            float X = (px - cx) * depth_value / fx;
            float Y = (py - cy) * depth_value / fy;
            float Z = depth_value;

            cloud->points.emplace_back(X, Y, Z);

            bbox.min_pt.x = std::min(bbox.min_pt.x, X);
            bbox.min_pt.y = std::min(bbox.min_pt.y, Y);
            bbox.min_pt.z = std::min(bbox.min_pt.z, Z);
            bbox.max_pt.x = std::max(bbox.max_pt.x, X);
            bbox.max_pt.y = std::max(bbox.max_pt.y, Y);
            bbox.max_pt.z = std::max(bbox.max_pt.z, Z);
            bbox.center.x += X;
            bbox.center.y += Y;
            bbox.center.z += Z;
            valid_points++;
        }
    }

    if (valid_points == 0)
    {
        const char *reason;
        if (roi_left >= roi_right || roi_top >= roi_bottom)
            reason = "ROI empty after clipping";
        else if (zero_depth_count > 0 && out_of_range_count == 0)
            reason = "all depth values are zero";
        else
            reason = "all depth values filtered by min_dist/max_dist";

        std::cerr << "[WARN] 3D zero: cls=" << bbox.cls_name
                  << " ROI=[" << obj.left << "," << obj.top
                  << " -> " << obj.right << "," << obj.bottom
                  << "] clipped=[" << roi_left << "," << roi_top
                  << " -> " << roi_right << "," << roi_bottom
                  << "] zero=" << zero_depth_count
                  << " out_of_range=" << out_of_range_count
                  << " - " << reason << "\n";
        return bbox;
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    bbox.center.x /= valid_points;
    bbox.center.y /= valid_points;
    bbox.center.z /= valid_points;

    Eigen::Vector3f camera_posit(bbox.center.x, bbox.center.y, bbox.center.z);
    Eigen::Vector3f robot_posi = m_params.rotation * camera_posit + m_params.translation;

    bbox.center.x = robot_posi.x();
    bbox.center.y = robot_posi.y();
    bbox.center.z = robot_posi.z();
    float yaw = atan2(bbox.center.y, bbox.center.x);
    float pitch = atan2(bbox.center.z, bbox.center.x);
    bbox.principal_dir = cv::Vec3f(yaw, pitch, 0);

    return bbox;
}

void Orbbec::Value_Depth_to_Pcl(
    const cv::Mat &depth_to_color,
    pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    cloud.clear();

    OrbbecCameraIntrinsics intr = get_color_intrinsics();
    const float fx = intr.fx;
    const float fy = intr.fy;
    const float cx = intr.cx;
    const float cy = intr.cy;

    for (int v = 0; v < depth_to_color.rows; v += 9)
    {
        for (int u = 0; u < depth_to_color.cols; u += 9)
        {
            float depth_value = depth_to_color.at<uint16_t>(v, u) * 0.001f;
            if (depth_value <= 0.0f)
                continue;

            float x = (u - cx) * depth_value / fx;
            float y = -(v - cy) * depth_value / fy;
            float z = depth_value;
            cloud.emplace_back(x, y, z);
        }
    }

    std::cout << "Global PointCloud: " << cloud.size() << std::endl;
}

Eigen::Vector3f Orbbec::compute_precise_center(
    const cv::Mat &depth_image,
    const yolo::Box &block_box,
    const yolo::Box &face_box,
    Eigen::Vector3f *out_normal_cam) const
{
    OrbbecCameraIntrinsics intr = get_color_intrinsics();
    const float fx = intr.fx, fy = intr.fy, cx = intr.cx, cy = intr.cy;
    const float depth_scale = 0.001f;

    auto clamp_box = [&](const yolo::Box &box, float margin_ratio)
    {
        const float w = std::max(1.0f, box.right - box.left);
        const float h = std::max(1.0f, box.bottom - box.top);
        const int l = std::max(0, (int)std::floor(box.left - w * margin_ratio));
        const int t = std::max(0, (int)std::floor(box.top - h * margin_ratio));
        const int r = std::min(depth_image.cols, (int)std::ceil(box.right + w * margin_ratio));
        const int b = std::min(depth_image.rows, (int)std::ceil(box.bottom + h * margin_ratio));
        return cv::Rect(l, t, std::max(0, r - l), std::max(0, b - t));
    };

    auto point_from_pixel = [&](int px, int py, float depth_m)
    {
        return Eigen::Vector3f((px - cx) * depth_m / fx,
                               (py - cy) * depth_m / fy,
                               depth_m);
    };

    cv::Rect seed_roi = clamp_box(face_box, -0.18f);
    if (seed_roi.width < 4 || seed_roi.height < 4)
    {
        seed_roi = clamp_box(face_box, 0.0f);
    }
    if (seed_roi.empty())
    {
        return Eigen::Vector3f::Zero();
    }

    std::vector<Eigen::Vector3f> seed_points;
    std::vector<float> seed_depths;
    seed_points.reserve((seed_roi.width / 3 + 1) * (seed_roi.height / 3 + 1));
    seed_depths.reserve(seed_points.capacity());

    constexpr int seed_step = 3;
    for (int py = seed_roi.y; py < seed_roi.y + seed_roi.height; py += seed_step)
    {
        const uint16_t *row = depth_image.ptr<uint16_t>(py);
        for (int px = seed_roi.x; px < seed_roi.x + seed_roi.width; px += seed_step)
        {
            const uint16_t raw = row[px];
            if (raw == 0)
                continue;

            const float d = raw * depth_scale;
            if (d <= m_params.min_dist || d > m_params.max_dist)
                continue;

            seed_depths.emplace_back(d);
            seed_points.emplace_back(point_from_pixel(px, py, d));
        }
    }

    if (seed_points.size() < 12)
    {
        return Eigen::Vector3f::Zero();
    }

    std::nth_element(seed_depths.begin(),
                     seed_depths.begin() + seed_depths.size() / 2,
                     seed_depths.end());
    const float median_depth = seed_depths[seed_depths.size() / 2];

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    int in_seed_count = 0;
    constexpr float seed_depth_gate_m = 0.06f;
    for (const auto &p : seed_points)
    {
        if (std::abs(p.z() - median_depth) > seed_depth_gate_m)
            continue;
        centroid += p;
        ++in_seed_count;
    }

    if (in_seed_count < 10)
    {
        return Eigen::Vector3f::Zero();
    }
    centroid /= static_cast<float>(in_seed_count);

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto &p : seed_points)
    {
        if (std::abs(p.z() - median_depth) > seed_depth_gate_m)
            continue;
        const Eigen::Vector3f q = p - centroid;
        cov += q * q.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
    if (eig.info() != Eigen::Success)
    {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f normal = eig.eigenvectors().col(0).normalized();
    if (normal.z() > 0.0f)
    {
        normal = -normal;
    }
    if (out_normal_cam)
    {
        *out_normal_cam = normal;
    }

    const float plane_d = -normal.dot(centroid);
    Eigen::Vector3f axis_u = normal.cross(Eigen::Vector3f::UnitY());
    if (axis_u.squaredNorm() < 1e-4f)
    {
        axis_u = normal.cross(Eigen::Vector3f::UnitX());
    }
    axis_u.normalize();
    Eigen::Vector3f axis_v = normal.cross(axis_u).normalized();

    cv::Rect block_roi = clamp_box(block_box, 0.10f);
    cv::Rect face_roi = clamp_box(face_box, 1.20f);
    cv::Rect grow_roi = block_roi | face_roi;
    grow_roi &= cv::Rect(0, 0, depth_image.cols, depth_image.rows);
    if (grow_roi.empty())
    {
        return Eigen::Vector3f::Zero();
    }

    constexpr int grow_step = 3;
    constexpr float plane_dist_gate_m = 0.018f;
    constexpr float depth_gate_m = 0.20f;
    constexpr int min_face_points = 35;

    float min_u = std::numeric_limits<float>::max();
    float max_u = -std::numeric_limits<float>::max();
    float min_v = std::numeric_limits<float>::max();
    float max_v = -std::numeric_limits<float>::max();
    int face_points = 0;

    for (int py = grow_roi.y; py < grow_roi.y + grow_roi.height; py += grow_step)
    {
        const uint16_t *row = depth_image.ptr<uint16_t>(py);
        for (int px = grow_roi.x; px < grow_roi.x + grow_roi.width; px += grow_step)
        {
            const uint16_t raw = row[px];
            if (raw == 0)
                continue;

            const float d = raw * depth_scale;
            if (d <= m_params.min_dist || d > m_params.max_dist)
                continue;
            if (std::abs(d - median_depth) > depth_gate_m)
                continue;

            const Eigen::Vector3f p = point_from_pixel(px, py, d);
            if (std::abs(normal.dot(p) + plane_d) > plane_dist_gate_m)
                continue;

            const Eigen::Vector3f q = p - centroid;
            const float u = q.dot(axis_u);
            const float v = q.dot(axis_v);
            min_u = std::min(min_u, u);
            max_u = std::max(max_u, u);
            min_v = std::min(min_v, v);
            max_v = std::max(max_v, v);
            ++face_points;
        }
    }

    if (face_points < min_face_points)
    {
        return centroid;
    }

    const Eigen::Vector3f face_center =
        centroid +
        axis_u * ((min_u + max_u) * 0.5f) +
        axis_v * ((min_v + max_v) * 0.5f);

    return face_center;
}

void Orbbec::record_videos(const std::string &output_path_prefix, const std::string &obj)
{
    cv::VideoWriter writer;
    bool recording = false;
    int file_index = 0;
    int recorded_frames = 0;

    std::cout << "[INFO] Press 's' to start recording, 'e' to stop current recording, 'q' to quit\n";
    while (true)
    {
        cv::Mat frame_bgr;
        Color_to_Cv(frame_bgr);
        if (frame_bgr.empty())
            continue;

        cv::imshow("Orbbec Manual Recorder", frame_bgr);
        char key = (char)cv::waitKey(1);

        if (key == 's' && !recording)
        {
            std::ostringstream oss;
            std::string path = output_path_prefix;
            if (path.back() != '/' && path.back() != '\\')
                path += '/';

            oss << path << obj << "_" << file_index << ".mp4";
            std::string fname = oss.str();

            writer.open(fname,
                        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                        m_params.fps,
                        cv::Size(frame_bgr.cols, frame_bgr.rows));
            if (!writer.isOpened())
            {
                std::cerr << "[ERROR] Cannot open video file: " << fname << std::endl;
            }
            else
            {
                recording = true;
                recorded_frames = 0;
                std::cout << "[INFO] Start recording: " << fname << std::endl;
            }
        }
        else if (key == 'e' && recording)
        {
            writer.release();
            recording = false;
            std::cout << "[INFO] Stop recording, file index: " << file_index
                      << ", frames: " << recorded_frames << "\n";
            file_index++;
        }
        else if (key == 'q')
        {
            if (recording)
            {
                writer.release();
            }
            break;
        }

        if (recording && writer.isOpened())
        {
            writer.write(frame_bgr);
            recorded_frames++;
        }

        usleep(1000);
    }

    cv::destroyWindow("Orbbec Manual Recorder");
}

void Orbbec::capture_images(const std::string &output_path_prefix,
                            const std::string &obj)
{
    int folder_index = 0;
    int image_index = 0;
    std::string current_folder;

    auto update_folder = [&]()
    {
        std::ostringstream oss;
        std::string base = output_path_prefix;
        if (base.back() != '/' && base.back() != '\\')
            base += '/';

        oss << base << obj << "_" << folder_index;
        current_folder = oss.str();

        std::string cmd = "mkdir -p " + current_folder;
        system(cmd.c_str());

        image_index = 0;

        std::cout << "[INFO] Switch to folder: " << current_folder << std::endl;
    };

    update_folder();

    std::cout << "[INFO] Press 'c' to capture, 'n' to switch folder, Ctrl+C to quit\n";

    while (true)
    {
        cv::Mat frame_bgr;
        Color_to_Cv(frame_bgr);
        if (frame_bgr.empty())
            continue;

        cv::imshow("Orbbec Image Capture", frame_bgr);
        char key = (char)cv::waitKey(1);

        if (key == 'c')
        {
            std::ostringstream oss;
            oss << current_folder << "/"
                << std::setw(6) << std::setfill('0')
                << image_index << ".png";

            std::string filename = oss.str();

            if (cv::imwrite(filename, frame_bgr))
            {
                std::cout << "[INFO] Save image: " << filename << std::endl;
                image_index++;
            }
            else
            {
                std::cerr << "[ERROR] Image save failed: " << filename << std::endl;
            }
        }
        else if (key == 'n')
        {
            folder_index++;
            update_folder();
        }

        usleep(1000);
    }

    cv::destroyWindow("Orbbec Image Capture");
}

void Orbbec::start_imu()
{
    // The Orbbec SDK exposes IMU through separate motion sensors and callbacks.
    // Keep the public API compatible with K4A for now; detection can run with
    // zero gyro until a device-specific IMU callback is enabled and calibrated.
    imu_started_ = false;
    last_gyro_ = Eigen::Vector3f::Zero();
}

Eigen::Vector3f Orbbec::get_gyro()
{
    std::lock_guard<std::mutex> lock(gyro_mutex_);
    return last_gyro_;
}
