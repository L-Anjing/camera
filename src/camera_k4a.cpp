#include "camera_k4a.hpp"
#include <main.hpp>
#include "myinfer.hpp"
#include <iostream>

using namespace std;

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

// start device and configuration
// It's not necessary to call this function,just instantiated classes K4a!!
void K4a::Configuration()
{
    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;
    device.start_cameras(&config);
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
void K4a::Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth)
{
    if (device.get_capture(&capture, chrono::milliseconds(1000)))
    {
        image_k4a_color = capture.get_color_image();
        image_k4a_depth = capture.get_depth_image();
        image_k4a_depth_to_color = k4aTransformation.depth_image_to_color_camera(image_k4a_depth);
        image_cv_color = cv::Mat(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC4, image_k4a_color.get_buffer());
        cv::cvtColor(image_cv_color, image_cv_color, cv::COLOR_BGRA2BGR);

        image_cv_depth = cv::Mat(image_k4a_depth_to_color.get_height_pixels(), image_k4a_depth_to_color.get_width_pixels(), CV_16U, image_k4a_depth_to_color.get_buffer());
        // image_cv_depth.convertTo(image_cv_depth, CV_8U);
    }
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

// draw color image with mask and label
void K4a::Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray &objs)
{
    // Cycle through all objectives, frames, and labels
    for (auto &obj : objs)
    {
        if (obj.left >= 0 && obj.right < image_cv_color.cols && obj.top >= 0 && obj.bottom <= image_cv_color.rows)
        {
            uint8_t b, g, r;
            std::tie(b, g, r) = yolo::random_color(obj.class_label);
            cv::rectangle(image_cv_color, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom),
                          cv::Scalar(b, g, r), 5);
            auto name = labels[obj.class_label];
            auto caption = cv::format("%s %.2f", name, obj.confidence);
            int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
            cv::rectangle(image_cv_color, cv::Point(obj.left - 3, obj.top - 33),
                          cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
            cv::putText(image_cv_color, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
            if (obj.seg && obj.seg->data != nullptr)
            {
                if (obj.left >= 0 && obj.seg->width >= 0 && obj.left + obj.seg->width < image_cv_color.cols &&
                    obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_cv_color.rows)
                {

                    cv::Mat mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                    mask.convertTo(mask, CV_8UC1);

                    // Ensure mask dimensions are valid before resizing
                    int mask_width = std::max(1, static_cast<int>(obj.right - obj.left));
                    int mask_height = std::max(1, static_cast<int>(obj.bottom - obj.top));

                    // Resize mask to fit the bounding box
                    cv::resize(mask, mask, cv::Size(mask_width, mask_height), 0, 0, cv::INTER_LINEAR);
                    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR); // Convert to 3-channel image

                    // Ensure that the image region and mask have the same size
                    cv::Mat region = image_cv_color(cv::Rect(obj.left, obj.top, mask_width, mask_height));
                    cv::addWeighted(region, 0.5, mask, 0.5, 0.0, region); // Blend mask onto the image region

                    mask.copyTo(image_cv_color(cv::Rect(obj.left, obj.top, mask_width, mask_height)));
                }
            }
        }
    }
}

// get point cloud from depth image
BoundingBox3D K4a::Value_Mask_to_Pcl(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const cv::Mat &depth_image,
    yolo::BoxArray objs)
{
    cloud->clear();
    BoundingBox3D bbox;
    bbox.min_pt = cv::Point3f(FLT_MAX, FLT_MAX, FLT_MAX);
    bbox.max_pt = cv::Point3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    bbox.center = cv::Point3f(0, 0, 0);
    bbox.principal_dir = cv::Vec3f(0, 0, 0);
    bbox.cls_name = "basket";

    size_t valid_points = 0;

    // Azure Kinect depth 单位是 mm，这里转成 m
    const float depth_scale = 0.001f;

    // 直接取 K4A 深度相机内参
    const k4a_calibration_camera_t &depth_intrinsics = k4aCalibration.depth_camera_calibration;
    float fx = depth_intrinsics.intrinsics.parameters.param.fx;
    float fy = depth_intrinsics.intrinsics.parameters.param.fy;
    float cx = depth_intrinsics.intrinsics.parameters.param.cx;
    float cy = depth_intrinsics.intrinsics.parameters.param.cy;

    for (auto &obj : objs)
    {
        bbox.cls_name = obj.class_label; // YOLO 推理类别

        cv::Mat mask;
        if (obj.seg)
        {
            mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
            mask.convertTo(mask, CV_8UC1);
            cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top));
            // [NEW] 腐蚀 + 膨胀，清理噪声，增强边界
            cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
            cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
        }
        else
        {
            mask = cv::Mat::zeros(obj.bottom - obj.top, obj.right - obj.left, CV_8UC1);
            cv::rectangle(mask, cv::Rect(0, 0, mask.cols, mask.rows), cv::Scalar(255), cv::FILLED);
        }

        // 提取点云
        for (int v = 0; v < mask.rows; v++)
        {
            for (int u = 0; u < mask.cols; u++)
            {
                if (mask.at<uchar>(v, u) > 0)
                {
                    int px = obj.left + u;
                    int py = obj.top + v;

                    if (px < 0 || px >= depth_image.cols || py < 0 || py >= depth_image.rows)
                        continue;

                    float depth_value = depth_image.at<uint16_t>(py, px) * depth_scale;
                    if (depth_value <= 0.0f || depth_value > 3.0f) // [NEW] 过滤远距离噪声 (>3m)
                        continue;
                    // 像素 -> 相机坐标
                    float X = (px - cx) * depth_value / fx;
                    float Y = -(py - cy) * depth_value / fy;
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
        }
    }

    if (valid_points > 0)
    {
        bbox.center.x /= valid_points;
        bbox.center.y /= valid_points;
        bbox.center.z /= valid_points;
    }
    else
    {
        bbox.center = cv::Point3f(0, 0, 0);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    if (cloud->empty())
        return bbox;

    // [NEW] 体素滤波，点云更均匀
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.002f, 0.002f, 0.002f); // 2mm 网格
    voxel.filter(*voxel_filtered);

    if (voxel_filtered->empty())
        return bbox;

    // 2. PCA 主方向
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*voxel_filtered, centroid);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*voxel_filtered, centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eig_vectors = eigen_solver.eigenvectors();

    // 取最大特征值对应的特征向量（主方向）
    Eigen::Vector3f principal = eig_vectors.col(2);
    bbox.principal_dir = cv::Vec3f(principal(0), principal(1), principal(2));

    // [NEW] 用滤波后的点云重算 bbox 中心
    bbox.center = cv::Point3f(centroid(0), centroid(1), centroid(2));

    return bbox;
}
void K4a::Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray &objs)
{
    for (auto &obj : objs)
    { // 检查目标框是否在深度图范围内
        if (obj.left >= 0 && obj.right < image_cv_depth.cols && obj.top >= 0 && obj.bottom <= image_cv_depth.rows)
        { // 随机颜色
            uint8_t b, g, r;
            std::tie(b, g, r) = yolo ::random_color(obj.class_label);
            // 画矩形框
            cv::rectangle(image_cv_depth,
                          cv::Point(obj.left, obj.top),
                          cv::Point(obj.right, obj.bottom),
                          cv::Scalar(b, g, r), 2);
            // 类别名称+置信度
            auto name = labels[obj.class_label];
            auto caption = cv::format("%s %.2f", name, obj.confidence);

            // 放文字
            int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
            cv::rectangle(image_cv_depth,
                          cv::Point(obj.left - 3, obj.top - 33),
                          cv::Point(obj.left + width, obj.top),
                          cv::Scalar(b, g, r), -1);
            cv::putText(image_cv_depth, caption,
                        cv::Point(obj.left, obj.top - 5),
                        0, 1,
                        cv::Scalar::all(0), 2, 16);

            // 画掩码
            if (obj.seg)
            {
                if (obj.left >= 0 && obj.seg->width >= 0 && obj.left + obj.seg->width < image_cv_depth.cols &&
                    obj.top >= 0 && obj.seg->height >= 0 &&
                    obj.top + obj.seg->height <= image_cv_depth.rows)
                {
                    // 掩码
                    cv::Mat mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                    mask.convertTo(mask, CV_8UC1);
                    cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR);

                    // 将掩码转成 3 通道，以便与 ROI 融合（GPT generate)
                    cv::Mat mask_color;
                    cv::cvtColor(mask, mask_color, cv::COLOR_GRAY2BGR);

                    cv::addWeighted(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask, 1.0, 0.0, mask);
                    mask.copyTo(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
                }
            }
        }
    }
}

void K4a::Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    cloud.clear();
    uint16_t *depth_data = (uint16_t *)image_k4a_depth_to_color.get_buffer();
    for (int v = 0; v < image_k4a_depth_to_color.get_height_pixels(); v += 9)
    {
        for (int u = 0; u < image_k4a_depth_to_color.get_width_pixels(); u += 9)
        {
            float depth_value = static_cast<float>(depth_data[v * image_k4a_depth_to_color.get_width_pixels() + u] / 1000.0);
            if (depth_value != 0)
            {
                float x = (u - color_intrinsics.intrinsics.parameters.param.cx) * depth_value / color_intrinsics.intrinsics.parameters.param.fx;
                float y = (v - color_intrinsics.intrinsics.parameters.param.cy) * depth_value / color_intrinsics.intrinsics.parameters.param.fy;
                float z = depth_value;
                pcl::PointXYZ point(x, y, z);
                cloud.push_back(point);
            }
        }
    }
    std::cout << "Global PointCloud:" << cloud.size() << std::endl;
}
void K4a::get_intrinsics(float &fx, float &fy, float &cx, float &cy)
{
    k4a_calibration_camera_t calib = k4aCalibration.color_camera_calibration;
    fx = calib.intrinsics.parameters.param.fx;
    fy = calib.intrinsics.parameters.param.fy;
    cx = calib.intrinsics.parameters.param.cx;
    cy = calib.intrinsics.parameters.param.cy;
}
void K4a::record_videos(const std::string &output_path_prefix,const std::string &obj)
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
