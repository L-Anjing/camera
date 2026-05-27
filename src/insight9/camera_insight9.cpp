#include "insight9/camera_insight9.hpp"
#include "common/camera_params.hpp"
#include <iostream>
#include <thread>
#include <sensor_msgs/msg/compressed_image.hpp>

using namespace std;

// 私有构造函数
Insight9::Insight9(const CameraParams& params) : m_params(params)
{
    // 从配置文件中的相机内参初始化
    // 这里需要根据实际的相机参数设置
    // 深度相机和彩色相机可能有不同的内参
    color_intrinsics_.fx = 385.0f;  // 示例值，需要根据实际相机标定
    color_intrinsics_.fy = 385.0f;
    color_intrinsics_.cx = 320.0f;
    color_intrinsics_.cy = 240.0f;

    depth_intrinsics_ = color_intrinsics_;  // 假设对齐到彩色相机

    // 从config初始化分辨率
    color_width_ = m_params.color_width;
    color_height_ = m_params.color_height;
    depth_width_ = m_params.width;
    depth_height_ = m_params.height;

    COUT_GREEN_START;
    cout << "Insight9 Camera Initialized with config:" << endl;
    cout << "  Color Resolution: " << color_width_ << "x" << color_height_ << endl;
    cout << "  Depth Resolution: " << depth_width_ << "x" << depth_height_ << endl;
    cout << "  FPS: " << m_params.fps << endl;
    cout << "  Min Distance: " << m_params.min_dist << " m" << endl;
    cout << "  Max Distance: " << m_params.max_dist << " m" << endl;
    COUT_COLOR_END;
}

// 工厂函数：通过配置文件创建
Insight9 Insight9::Create_FromFile(const std::string& config_path,
                                   std::shared_ptr<rclcpp::Node> node)
{
    CameraParams params = CameraParams::LoadFromFile(config_path);
    Insight9 camera(params);

    if (node)
    {
        camera.Initialize_ROS2_Subscribers(node);
    }

    return std::move(camera);
}

// 初始化 ROS2 订阅
void Insight9::Initialize_ROS2_Subscribers(std::shared_ptr<rclcpp::Node> node)
{
    node_ = node;

    // 深度图回调（16-bit深度）
    auto depth_cb = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "16UC1");
            std::lock_guard<std::mutex> lock(depth_mutex_);
            current_depth_image_ = cv_ptr->image.clone();
            depth_width_ = current_depth_image_.cols;
            depth_height_ = current_depth_image_.rows;
            depth_available_ = true;
        }
        catch (cv_bridge::Exception& e)
        {
            COUT_RED_START;
            cerr << "cv_bridge exception (depth): " << e.what() << endl;
            COUT_COLOR_END;
        }
    };

    // 压缩彩色图回调（JPEG压缩）
    auto compressed_color_cb = [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            std::lock_guard<std::mutex> lock(color_mutex_);
            current_color_image_ = cv_ptr->image.clone();
            color_width_ = current_color_image_.cols;
            color_height_ = current_color_image_.rows;
            color_available_ = true;
        }
        catch (cv_bridge::Exception& e)
        {
            COUT_RED_START;
            cerr << "cv_bridge exception (compressed color): " << e.what() << endl;
            COUT_COLOR_END;
        }
    };

    // 原始彩色图回调（备用）
    auto color_cb = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            std::lock_guard<std::mutex> lock(color_mutex_);
            current_color_image_ = cv_ptr->image.clone();
            color_available_ = true;
        }
        catch (cv_bridge::Exception& e)
        {
            COUT_RED_START;
            cerr << "cv_bridge exception (color): " << e.what() << endl;
            COUT_COLOR_END;
        }
    };

    // 相机标定信息回调
    auto color_info_cb = [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        // 从camera_info自动更新内参
        color_intrinsics_.fx = msg->k[0];
        color_intrinsics_.fy = msg->k[4];
        color_intrinsics_.cx = msg->k[2];
        color_intrinsics_.cy = msg->k[5];
        
        COUT_BLUE_START;
        cout << "Color Camera Info Updated: fx=" << color_intrinsics_.fx 
             << " fy=" << color_intrinsics_.fy << endl;
        COUT_COLOR_END;
    };

    // 订阅深度图（总是存在）
    depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/depth/image_rect_raw", 10, depth_cb);

    // 优先订阅压缩彩色图（节省带宽）
    compressed_color_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/camera/camera/color/image_rect_raw/compressed", 10, compressed_color_cb);

    // 备选：订阅原始彩色图（如果压缩不可用）
    // color_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    //     "/camera/camera/color/image_rect_raw", 10, color_cb);

    // 订阅相机标定信息（可选，用于自动更新内参）
    color_camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/color/camera_info", 1, color_info_cb);

    depth_camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/depth/camera_info", 1,
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            depth_intrinsics_.fx = msg->k[0];
            depth_intrinsics_.fy = msg->k[4];
            depth_intrinsics_.cx = msg->k[2];
            depth_intrinsics_.cy = msg->k[5];
        });

    COUT_BLUE_START;
    cout << "╔═══════════════════════════════════════════════════════╗" << endl;
    cout << "║        Insight9 ROS2 Subscribers Initialized          ║" << endl;
    cout << "╠═══════════════════════════════════════════════════════╣" << endl;
    cout << "║ Color Image:  /camera/camera/color/image_rect_raw/   ║" << endl;
    cout << "║               compressed                             ║" << endl;
    cout << "║ Depth Image:  /camera/camera/depth/image_rect_raw    ║" << endl;
    cout << "║ Color Info:   /camera/camera/color/camera_info       ║" << endl;
    cout << "║ Depth Info:   /camera/camera/depth/camera_info       ║" << endl;
    cout << "╚═══════════════════════════════════════════════════════╝" << endl;
    COUT_COLOR_END;
}

// 获取相机内参
CameraIntrinsics Insight9::get_color_intrinsics() const
{
    return color_intrinsics_;
}

CameraIntrinsics Insight9::get_depth_intrinsics() const
{
    return depth_intrinsics_;
}

// 获取同步的彩色图和深度图
bool Insight9::Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth)
{
    {
        std::lock_guard<std::mutex> lock_color(color_mutex_);
        std::lock_guard<std::mutex> lock_depth(depth_mutex_);

        // 检查两个图像是否都可用
        if (!color_available_ || !depth_available_)
        {
            return false;
        }

        // 复制图像数据
        image_cv_color = current_color_image_.clone();
        image_cv_depth = current_depth_image_.clone();

        return true;
    }
}

// 从 block 结果计算 3D 包围盒和点云
BoundingBox3D Insight9::Value_Block_to_Pcl(
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

    bbox.cls_ID = -1;
    bbox.cls_name = "unknown";

    // Block 语义
    bbox.cls_ID = static_cast<int>(objs.block_class);
    bbox.cls_name = block_class_name(objs.block_class);
    // 最终 block 对应的 box
    const yolo::Box &obj = objs.best_pattern;

    size_t valid_points = 0;

    // 深度值单位转换（根据实际相机调整，通常为 mm -> m 的转换）
    const float depth_scale = 0.001f;

    // 获取相机内参（深度对齐到彩色图）
    CameraIntrinsics intr = get_color_intrinsics();
    const float fx = intr.fx;
    const float fy = intr.fy;
    const float cx = intr.cx;
    const float cy = intr.cy;

    // 计算坐标系缩放因子（从color分辨率到depth分辨率）
    float scale_x = static_cast<float>(depth_width_) / color_width_;
    float scale_y = static_cast<float>(depth_height_) / color_height_;

    // 缩放detection框到depth坐标系
    int depth_left = static_cast<int>(obj.left * scale_x);
    int depth_right = static_cast<int>(obj.right * scale_x);
    int depth_top = static_cast<int>(obj.top * scale_y);
    int depth_bottom = static_cast<int>(obj.bottom * scale_y);

    // 遍历检测框内的所有像素
    for (int py = depth_top; py < depth_bottom; ++py)
    {
        for (int px = depth_left; px < depth_right; ++px)
        {
            // 边界检查
            if (px < 0 || px >= depth_image.cols || py < 0 || py >= depth_image.rows)
            {
                continue;
            }

            float depth_value = depth_image.at<uint16_t>(py, px) * depth_scale;
            if (depth_value <= m_params.min_dist || depth_value > m_params.max_dist)
            {
                continue;
            }

            // 像素 → 相机坐标
            // OpenCV像素坐标系: x向右, y向下
            // 相机坐标系: x向右, y向下, z向前
            float X = (px - cx) * depth_value / fx;
            float Y = (py - cy) * depth_value / fy;
            float Z = depth_value;

            cloud->points.emplace_back(X, Y, Z);

            // 更新 3D 包围盒
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
        return bbox;
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    // 平均中心
    bbox.center.x /= valid_points;
    bbox.center.y /= valid_points;
    bbox.center.z /= valid_points;

    // 坐标变换：从相机坐标系到机器人坐标系
    float Xc = bbox.center.x;
    float Yc = bbox.center.y;
    float Zc = bbox.center.z;

    // P_robot = R * P_camera + T
    Eigen::Vector3f camera_posit(Xc, Yc, Zc);
    Eigen::Vector3f robot_posi = m_params.rotation * camera_posit + m_params.translation;

    bbox.center.x = robot_posi.x();
    bbox.center.y = robot_posi.y();
    bbox.center.z = robot_posi.z();

    // 计算偏角（弧度制）
    float yaw = atan2(bbox.center.y, bbox.center.x);     // 水平偏角
    float pitch = atan2(bbox.center.z, bbox.center.x);   // 垂直偏角

    bbox.principal_dir = cv::Vec3f(yaw, pitch, 0);

    return bbox;
}

// 从深度图生成全局点云
void Insight9::Value_Depth_to_Pcl(
    const cv::Mat &depth_image,
    pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    cloud.clear();

    // 获取相机内参
    CameraIntrinsics intr = get_color_intrinsics();
    const float fx = intr.fx;
    const float fy = intr.fy;
    const float cx = intr.cx;
    const float cy = intr.cy;

    const int width = depth_image.cols;
    const int height = depth_image.rows;

    // 像素 → 相机坐标
    // 为了提高效率，每9个像素采样一次
    for (int v = 0; v < height; v += 9)
    {
        for (int u = 0; u < width; u += 9)
        {
            float depth_value = depth_image.at<uint16_t>(v, u) * 0.001f;

            if (depth_value <= 0.0f || depth_value > m_params.max_dist)
                continue;

            float x = (u - cx) * depth_value / fx;
            float y = -(v - cy) * depth_value / fy;  // 翻转 Y 轴
            float z = depth_value;

            cloud.emplace_back(x, y, z);
        }
    }

    COUT_BLUE_START;
    cout << "Global PointCloud: " << cloud.size() << " points" << endl;
    COUT_COLOR_END;
}
