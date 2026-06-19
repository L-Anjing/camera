#include "k4a/camera_k4a.hpp"
#include "utils/myinfer.hpp"
#include "utils/utils.hpp"

#include <iostream>

using namespace std;

// 私有构造函数：内部调用 Configuration()
K4a::K4a(const CameraParams& params) : m_params(params)
{
    Installed_Count();
    if (!Open())
    {
        throw std::runtime_error("Failed to open K4a device");
    }
    this->Configuration(); // 一次性完成所有逻辑
}

// 工厂函数：通过配置文件路径创建 K4a 对象
K4a K4a::Create_FromFile(const std::string& config_path)
{
    CameraParams params = CameraParams::LoadFromFile(config_path);
    return K4a(params);
}

// open k4a device
bool K4a::Open()
{
    try
    {
        device = k4a::device::open(K4A_DEVICE_DEFAULT);
        COUT_GREEN_START;
        cout << "Open K4a Device Success!" << endl;
        COUT_COLOR_END;
        return true;
    }
    catch (const std::exception &e)
    {
        COUT_RED_START;
        cerr << "Open K4a Device Error!" << endl;
        COUT_COLOR_END;
        return false;
    }
}

// get device count
void K4a::Installed_Count()
{
    device_count = k4a::device::get_installed_count();
    if (device_count == 0)
    {
        COUT_RED_START
        cout << "No K4a Device Found!" << endl;
        COUT_COLOR_END
    }
    else
    {
        COUT_BLUE_START
        cout << "Find " << device_count << " Device(s)" << endl;
        COUT_COLOR_END
    }
}


// start device and configuration based on CameraParams
void K4a::Configuration()
{
    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    
    // 根据 CameraParams 中的分辨率选择相应的配置
    // K4a SDK 支持的分辨率: 720P, 1080P, 1440P, 1536P
    // CameraParams 中的宽高: width = 1280, height = 720 (对应 720P)
    if (m_params.width == 1280 && m_params.height == 720)
    {
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    }
    else if (m_params.width == 1920 && m_params.height == 1080)
    {
        config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    }
    else if (m_params.width == 2560 && m_params.height == 1440)
    {
        config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
    }
    else if (m_params.width == 2560 && m_params.height == 1536)
    {
        config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
    }
    else
    {
        // 默认使用 720P
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        COUT_BLUE_START;
        cout << "[WARNING] 不支持的分辨率 " << m_params.width << "x" << m_params.height 
             << "，默认使用 720P" << endl;
        COUT_COLOR_END;
    }
    
    // 根据 CameraParams 中的帧率选择相应的配置
    // K4a SDK 支持的帧率: 5, 15, 30
    if (m_params.fps == 5)
    {
        config.camera_fps = K4A_FRAMES_PER_SECOND_5;
    }
    else if (m_params.fps == 15)
    {
        config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    }
    else if (m_params.fps == 30)
    {
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    }
    else
    {
        // 默认使用 30 fps
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        COUT_BLUE_START;
        cout << "[WARNING] 不支持的帧率 " << m_params.fps 
             << " fps，默认使用 30 fps" << endl;
        COUT_COLOR_END;
    }
    
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;
    device.start_cameras(&config);
    start_imu();    // 启动 IMU (陀螺仪 + 加速度计)
    COUT_GREEN_START;
    cout << "Start Device Success!" << endl;
    COUT_COLOR_END;
    k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);
    k4aTransformation = k4a::transformation(k4aCalibration);
    color_intrinsics = k4aCalibration.color_camera_calibration;
}

/*get image from device and convert to cv::Mat
cv::Mat 是 OpenCV 库定义和优化的“通用矩阵”数据结构，它是整个 OpenCV 图像处理生态系统的“官方语言”
*/
// void K4a::Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth)
// {
//     if (device.get_capture(&capture, chrono::milliseconds(1000)))
//     {
//         image_k4a_color = capture.get_color_image();
//         image_k4a_depth = capture.get_depth_image();
//         image_k4a_depth_to_color = k4aTransformation.depth_image_to_color_camera(image_k4a_depth);
//         image_cv_color = cv::Mat(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC4, image_k4a_color.get_buffer());
//         cv::cvtColor(image_cv_color, image_cv_color, cv::COLOR_BGRA2BGR);

//         image_cv_depth = cv::Mat(image_k4a_depth_to_color.get_height_pixels(), image_k4a_depth_to_color.get_width_pixels(), CV_16U, image_k4a_depth_to_color.get_buffer());
//         // image_cv_depth.convertTo(image_cv_depth, CV_8U);
//     }
// }
void K4a::Image_to_Cv(
    cv::Mat &image_cv_color,
    cv::Mat &image_cv_depth)
{
    // ── 帧丢弃 ────────────────────────────────────────────
    // 上一轮 YOLO 推理可能耗时 200~500ms，期间 K4A buffer 积压了多帧。
    // 先用 0 超时不断弹出旧帧丢弃，确保处理的是最新的那一帧。
    k4a::capture discard;
    while (device.get_capture(&discard, std::chrono::milliseconds(0)))
    {
        // 丢弃积压的旧帧
    }

    k4a::capture capture;

    if (!device.get_capture(&capture,
        std::chrono::milliseconds(1000)))
    {
        return;
    }

    auto image_k4a_color =
        capture.get_color_image();

    auto image_k4a_depth =
        capture.get_depth_image();

    auto image_k4a_depth_to_color =
        k4aTransformation.depth_image_to_color_camera(
            image_k4a_depth);

    cv::Mat color_tmp(
        image_k4a_color.get_height_pixels(),
        image_k4a_color.get_width_pixels(),
        CV_8UC4,
        image_k4a_color.get_buffer());

    cv::cvtColor(
        color_tmp,
        image_cv_color,
        cv::COLOR_BGRA2BGR);

    image_cv_color =
        image_cv_color.clone();

    image_cv_depth =
        cv::Mat(
            image_k4a_depth_to_color.get_height_pixels(),
            image_k4a_depth_to_color.get_width_pixels(),
            CV_16U,
            image_k4a_depth_to_color.get_buffer()
        ).clone();
}
// get color image from device and convert to cv::Mat
void K4a::Color_to_Cv(cv::Mat &image_cv_color)
{
    if (device.get_capture(&capture, chrono::milliseconds(1000)))
    {
        image_k4a_color = capture.get_color_image();
        image_cv_color = cv::Mat(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC4, image_k4a_color.get_buffer());
        cv::cvtColor(image_cv_color, image_cv_color, cv::COLOR_BGRA2BGR);
    }
}
//   get depth image from device and convert to cv::Mat
void K4a::Depth_to_Cv(cv::Mat &image_cv_depth)
{
    if (device.get_capture(&capture, chrono::milliseconds(1000)))
    {
        image_k4a_depth = capture.get_depth_image();
        image_k4a_depth_to_color = k4aTransformation.depth_image_to_color_camera(image_k4a_depth);
        image_cv_depth = cv::Mat(image_k4a_depth_to_color.get_height_pixels(), image_k4a_depth_to_color.get_width_pixels(), CV_16U, image_k4a_depth_to_color.get_buffer());
        image_cv_depth.convertTo(image_cv_depth, CV_8U);
    }
}

// save image to file
void K4a::Save_Image(int amount, std::string output_dir)
{
    if (frame_count >= amount)
    {
        return;
    }
    if (device.get_capture(&capture, chrono::milliseconds(1000)) && frame_count < amount)
    {
        image_k4a_color = capture.get_color_image();
        cv::Mat image_saved = cv::Mat(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC4, image_k4a_color.get_buffer());
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
        image_saved.release();
        usleep(50000);
    }
}

CameraIntrinsics K4a::get_color_intrinsics() const
{
    const auto &c = k4aCalibration.color_camera_calibration;
    return {
        c.intrinsics.parameters.param.fx,
        c.intrinsics.parameters.param.fy,
        c.intrinsics.parameters.param.cx,
        c.intrinsics.parameters.param.cy};
}

CameraIntrinsics K4a::get_depth_intrinsics() const
{
    const auto &d = k4aCalibration.depth_camera_calibration;
    return {
        d.intrinsics.parameters.param.fx,
        d.intrinsics.parameters.param.fy,
        d.intrinsics.parameters.param.cx,
        d.intrinsics.parameters.param.cy};
}

// draw color image with mask and label
void K4a::Color_With_Mask(cv::Mat &image_cv_color, const yolo::BoxArray &objs)
{
    // Cycle through all objectives, frames, and labels
    vision::draw_yolo_detections(image_cv_color, objs);
}

void K4a::Depth_With_Mask(cv::Mat &image_cv_depth, const yolo::BoxArray &objs)
{
    vision::draw_yolo_detections(image_cv_depth, objs);
}

// get obj's point cloud from depth image
BoundingBox3D K4a::Value_Block_to_Pcl(
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

    // Azure Kinect depth 单位：可能是 mm 或其他
    const float depth_scale = 0.001f;

    // 获取相机内参（深度已经对齐到颜色图）
    CameraIntrinsics intr = get_color_intrinsics();
    const float fx = intr.fx;
    const float fy = intr.fy;
    const float cx = intr.cx;
    const float cy = intr.cy;

    // //调试输出
    // std::cout << depth_image.cols << " x " << depth_image.rows << std::endl;
    // std::cout << "fx=" << fx << " fy=" << fy
    //           << " cx=" << cx << " cy=" << cy << std::endl;

    // 将 ROI 裁剪到图像边界，避免无意义遍历
    const int cols = depth_image.cols;
    const int rows = depth_image.rows;
    const int roi_left   = std::max(0, static_cast<int>(obj.left));
    const int roi_top    = std::max(0, static_cast<int>(obj.top));
    const int roi_right  = std::min(cols, static_cast<int>(obj.right));
    const int roi_bottom = std::min(rows, static_cast<int>(obj.bottom));

    // 统计无效原因
    size_t zero_depth_count = 0;
    size_t out_of_range_count = 0;

    // 遍历检测框内的所有像素
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
        // ── 全零溯源 ────────────────────────────────────
        // 以下情况会导致有效点数为 0，返回的 bbox.center = (0,0,0)：
        //   ① 深度全零（ROI 在深度相机盲区 — 彩色图边缘 / 物体反光 / 过远）
        //   ② 深度值被 min_dist / max_dist 过滤掉
        //   ③ ROI 裁剪后为空（检测框完全在图像外）
        //   ④ best_pattern 是默认构造（全部为 0）
        const char *reason;
        if (roi_left >= roi_right || roi_top >= roi_bottom)
            reason = "ROI 裁剪后为空";
        else if (zero_depth_count > 0 && out_of_range_count == 0)
            reason = "全部深度值为 0（深度相机盲区 — 物体在彩色图边缘？）";
        else
            reason = "全部深度值被 min_dist/max_dist 过滤";

        std::cerr << "[WARN] 3D zero: cls=" << bbox.cls_name
                  << " ROI=[" << obj.left << "," << obj.top
                  << " -> " << obj.right << "," << obj.bottom
                  << "] clipped=[" << roi_left << "," << roi_top
                  << " -> " << roi_right << "," << roi_bottom
                  << "] zero=" << zero_depth_count
                  << " out_of_range=" << out_of_range_count
                  << " — " << reason << "\n";
        return bbox;
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    // 平均中心
    bbox.center.x /= valid_points;
    bbox.center.y /= valid_points;
    bbox.center.z /= valid_points;

    // 坐标变换
    float Xc = bbox.center.x;
    float Yc = bbox.center.y;
    float Zc = bbox.center.z;

    // 使用配置文件中的旋转矩阵和平移向量
    // 从相机坐标系变换到机器人坐标系: P_robot = R * P_camera + T
    Eigen::Vector3f camera_posit(Xc, Yc, Zc);
    Eigen::Vector3f robot_posi = m_params.rotation * camera_posit + m_params.translation;

    bbox.center.x = robot_posi.x();
    bbox.center.y = robot_posi.y();
    bbox.center.z = robot_posi.z();
    // 计算偏角（弧度制）
    float yaw = atan2(bbox.center.y, bbox.center.x); // 水平偏角
    float pitch = atan2(bbox.center.z, bbox.center.x); // 垂直偏角

    bbox.principal_dir = cv::Vec3f(yaw, pitch, 0);

    return bbox;
}

void K4a::Value_Depth_to_Pcl(
    const k4a::image &depth_to_color,
    pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    cloud.clear();

    //  获取颜色相机内参（ 若depth 已对齐到 color）
    CameraIntrinsics intr = get_color_intrinsics();
    const float fx = intr.fx;
    const float fy = intr.fy;
    const float cx = intr.cx;
    const float cy = intr.cy;

    // 访问深度数据缓冲
    const uint16_t *depth_data =
        reinterpret_cast<const uint16_t *>(depth_to_color.get_buffer());

    const int width = depth_to_color.get_width_pixels();
    const int height = depth_to_color.get_height_pixels();

    // 像素 → 相机坐标

    for (int v = 0; v < height; v += 9)
    {
        for (int u = 0; u < width; u += 9)
        {
            float depth_value =
                depth_data[v * width + u] * 0.001f;

            if (depth_value <= 0.0f)
                continue;

            float x = (u - cx) * depth_value / fx;
            float y = -(v - cy) * depth_value / fy; // 翻转Y轴
            float z = depth_value;

            cloud.emplace_back(x, y, z);
        }
    }

    std::cout << "Global PointCloud: "
              << cloud.size() << std::endl;
}

// ═══════════════════════════════════════════════════════

Eigen::Vector3f K4a::compute_precise_center(
    const cv::Mat &depth_image,
    const yolo::Box &block_box,
    const yolo::Box &face_box,
    Eigen::Vector3f *out_normal_cam) const
{
    CameraIntrinsics intr = get_color_intrinsics();
    const float fx = intr.fx, fy = intr.fy, cx = intr.cx, cy = intr.cy;
    const float depth_scale = 0.001f;

    // ── 1. 从 block 框收集点云（步长 2 加速） ──
    const int roi_l = std::max(0, (int)block_box.left);
    const int roi_t = std::max(0, (int)block_box.top);
    const int roi_r = std::min(depth_image.cols, (int)block_box.right);
    const int roi_b = std::min(depth_image.rows, (int)block_box.bottom);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(((roi_r - roi_l) / 2) * ((roi_b - roi_t) / 2));

    for (int py = roi_t; py < roi_b; py += 2)
        for (int px = roi_l; px < roi_r; px += 2)
        {
            uint16_t raw = depth_image.at<uint16_t>(py, px);
            if (raw == 0) continue;
            float d = raw * depth_scale;
            if (d <= m_params.min_dist || d > m_params.max_dist) continue;
            cloud->emplace_back((px - cx) * d / fx,
                                (py - cy) * d / fy,
                                d);
        }

    if (cloud->size() < 20) return Eigen::Vector3f::Zero();

    // ── 2. RANSAC 平面拟合 ──
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);               // 2cm 容差
    seg.setMaxIterations(200);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.size() < 10 || coeff->values.size() < 4)
        return Eigen::Vector3f::Zero();

    float A = coeff->values[0];
    float B = coeff->values[1];
    float C = coeff->values[2];
    float D = coeff->values[3];

    // 法向量（相机坐标系）
    Eigen::Vector3f normal(A, B, C);
    normal.normalize();
    if (out_normal_cam) *out_normal_cam = normal;

    // ── 3. 射线求交 ──
    float u = (face_box.left + face_box.right) / 2.0f;
    float v = (face_box.top + face_box.bottom) / 2.0f;

    float dx = (u - cx) / fx;
    float dy = (v - cy) / fy;
    float dz = 1.0f;

    float denom = A * dx + B * dy + C * dz;
    if (std::abs(denom) < 1e-6f) return Eigen::Vector3f::Zero();

    float t = -D / denom;
    if (t <= 0) return Eigen::Vector3f::Zero();

    // 相机坐标系下的精确中心
    return Eigen::Vector3f(t * dx, t * dy, t);
}

void K4a::record_videos(const std::string &output_path_prefix, const std::string &obj)
{
    cv::VideoWriter writer;
    bool recording = false;
    int file_index = 0; // 自动编号
    int recorded_frames = 0;

    std::cout << "[INFO] 按 's' 开始录制，'e' 停止当前录制，'q' 退出（若在录制则先停止）。\n";
    while (true)
    {
        // 获取一帧（阻塞超时1000ms）
        if (!device.get_capture(&capture, std::chrono::milliseconds(1000)))
            continue;

        k4a::image image_k4a_color = capture.get_color_image();
        if (!image_k4a_color.handle())
            continue;

        cv::Mat frame_rgba(image_k4a_color.get_height_pixels(),
                           image_k4a_color.get_width_pixels(),
                           CV_8UC4,
                           (void *)image_k4a_color.get_buffer());
        cv::Mat frame_bgr;
        cv::cvtColor(frame_rgba, frame_bgr, cv::COLOR_BGRA2BGR);

        // 显示画面（便于按键控制）
        cv::imshow("K4A Manual Recorder", frame_bgr);

        // 非阻塞读取键（等待 1 ms）
        char key = (char)cv::waitKey(1);

        if (key == 's' && !recording)
        {
            // 打开新文件
            std::ostringstream oss;
            std::string path = output_path_prefix;
            if (path.back() != '/' && path.back() != '\\')
                path += '/';

            oss << path << obj << "_" << file_index << ".mp4";
            std::string fname = oss.str();

            // 四字符编码 mp4v (可根据需要改)
            writer.open(fname,
                        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                        30.0,
                        cv::Size(frame_bgr.cols, frame_bgr.rows));
            if (!writer.isOpened())
            {
                std::cerr << "[ERROR] 无法打开视频文件: " << fname << std::endl;
            }
            else
            {
                recording = true;
                recorded_frames = 0;
                std::cout << "[INFO] 开始录制: " << fname << std::endl;
            }
        }
        else if (key == 'e' && recording)
        {
            // 停止当前录制
            writer.release();
            recording = false;
            std::cout << "[INFO] 停止录制，保存文件索引: " << file_index << "（帧数: " << recorded_frames << "）\n";
            file_index++;
        }
        else if (key == 'q')
        {
            // 退出：如果正在录制，先停止并保存
            if (recording)
            {
                writer.release();
                recording = false;
                std::cout << "[INFO] 停止录制并退出，保存文件索引: " << file_index << "（帧数: " << recorded_frames << "）\n";
                file_index++;
            }
            break;
        }

        // 若处于录制状态则写帧
        if (recording && writer.isOpened())
        {
            writer.write(frame_bgr);
            recorded_frames++;
        }

        // 轻微睡眠以降低 CPU 占用
        usleep(1000); // 1 ms
    }

    // 清理窗口（不处理 device 的关闭）
    cv::destroyWindow("K4A Manual Recorder");
}
void K4a::capture_images(const std::string &output_path_prefix,
                         const std::string &obj)
{
    int folder_index = 0; // 类别编号
    int image_index = 0;  // 当前文件夹内图片编号

    std::string current_folder;

    auto update_folder = [&]()
    {
        std::ostringstream oss;
        std::string base = output_path_prefix;
        if (base.back() != '/' && base.back() != '\\')
            base += '/';

        oss << base << obj << "_" << folder_index;
        current_folder = oss.str();

        // 创建文件夹（若已存在则忽略）
        std::string cmd = "mkdir -p " + current_folder;
        system(cmd.c_str());

        image_index = 0;

        std::cout << "[INFO] 切换到新类别文件夹: "
                  << current_folder << std::endl;
    };

    update_folder();

    std::cout << "[INFO] 按 'c' 拍照保存，'n' 切换下一个类别文件夹，Ctrl+C 退出程序\n";

    while (true)
    {
        // 获取一帧
        if (!device.get_capture(&capture, std::chrono::milliseconds(1000)))
            continue;

        k4a::image image_k4a_color = capture.get_color_image();
        if (!image_k4a_color.handle())
            continue;

        cv::Mat frame_rgba(image_k4a_color.get_height_pixels(),
                           image_k4a_color.get_width_pixels(),
                           CV_8UC4,
                           (void *)image_k4a_color.get_buffer());

        cv::Mat frame_bgr;
        cv::cvtColor(frame_rgba, frame_bgr, cv::COLOR_BGRA2BGR);

        // 显示画面
        cv::imshow("K4A Image Capture", frame_bgr);

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
                std::cout << "[INFO] 保存图片: " << filename << std::endl;
                image_index++;
            }
            else
            {
                std::cerr << "[ERROR] 图片保存失败: " << filename << std::endl;
            }
        }
        else if (key == 'n')
        {
            folder_index++;
            update_folder();
        }

        // 降低 CPU 占用
        usleep(1000);
    }

    cv::destroyWindow("K4A Image Capture");
}

// ═══════════════════════════════════════════════════════
//  IMU
// ═══════════════════════════════════════════════════════

void K4a::start_imu()
{
    try
    {
        device.start_imu();
        imu_started_ = true;
        COUT_GREEN_START;
        cout << "Start IMU Success!" << endl;
        COUT_COLOR_END;
    }
    catch (const std::exception &e)
    {
        COUT_RED_START;
        cerr << "Start IMU Failed: " << e.what() << endl;
        COUT_COLOR_END;
        imu_started_ = false;
    }
}

Eigen::Vector3f K4a::get_gyro()
{
    if (!imu_started_) return Eigen::Vector3f::Zero();

    k4a_imu_sample_t sample;
    while (device.get_imu_sample(&sample, std::chrono::milliseconds(0)))
        last_imu_ = sample;

    return Eigen::Vector3f(last_imu_.gyro_sample.xyz.x,
                           last_imu_.gyro_sample.xyz.y,
                           last_imu_.gyro_sample.xyz.z);
}