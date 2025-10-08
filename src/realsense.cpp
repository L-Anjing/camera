#include "realsense.hpp"
#include "main.hpp"
#include "myinfer.hpp"

using namespace std;

void RealSense::Configuration_Default()
{
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    profile = pipe.start(cfg);
    std::cout << "管道启动成功！" << std::endl;
    rs2::video_stream_profile color_profile =
        profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    intrinsics_color = color_profile.get_intrinsics();

    // gpt generate
    auto depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    intrinsics_depth = depth_profile.get_intrinsics(); // get intrinsics

    COUT_GREEN_START;
    std::cout << "Open Realsense Default Success!" << std::endl;
    COUT_COLOR_END;
}

void RealSense::Configuration_Infrared_Only()
{
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    profile = pipe.start(cfg);
    rs2::video_stream_profile infrared_profile =
        profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
    intrinsics_infrared = infrared_profile.get_intrinsics();
    for (auto &&sensor : profile.get_device().query_sensors()) // Disable the infrared laser emitter of RealSense camera
    {
        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        {
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
        }
    }
    COUT_GREEN_START
    std::cout << "Open Realsense Infrared Only Success!" << std::endl;
    COUT_COLOR_END;
}

RealSense RealSense::Create_Default()
{
    RealSense rs;
    rs.Configuration_Default();
    return rs;
}

RealSense RealSense::Create_Infrared_Only()
{
    RealSense rs;
    rs.Configuration_Infrared_Only();
    return rs;
}

void RealSense::Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth)
{
    // Wait for frames and align them to color
    frameset = pipe.wait_for_frames();
    if (frameset.size() == 0)
    {
        std::cerr << "Error: No frames received!" << std::endl;
        return;
    }
    // Align the depth frame to color frame
    rs2::align align_to_color(RS2_STREAM_COLOR);
    frameset = align_to_color.process(frameset);

    // Get the color and depth frames from aligned frameset
    rs2::video_frame frame_color = frameset.get_color_frame();
    rs2::depth_frame frame_depth = frameset.get_depth_frame();

    // Convert the color and depth frames to Mat objects
    image_rs_color = cv::Mat(frame_color.get_height(), frame_color.get_width(), CV_8UC3, (void *)frame_color.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(image_rs_color, image_rs_color, cv::COLOR_RGB2BGR);
    image_rs_depth = cv::Mat(frame_depth.get_height(), frame_depth.get_width(), CV_16UC1, (void *)frame_depth.get_data(), cv::Mat::AUTO_STEP);

    // Convert the depth frame to 8-bit grayscale image
    // image_rs_depth.convertTo(image_rs_depth, CV_8UC1, 255.0 / 1000.0);

    // Copy the Mat objects to the output parameters
    image_cv_color = image_rs_color;
    image_cv_depth = image_rs_depth;
}

void RealSense::Color_to_Cv(cv::Mat &image_cv_color)
{
    frameset = pipe.wait_for_frames();
    rs2::video_frame frame_color = frameset.get_color_frame();
    image_rs_color = cv::Mat(frame_color.get_height(), frame_color.get_width(), CV_8UC3, (void *)frame_color.get_data(), cv::Mat::AUTO_STEP);
    image_cv_color = image_rs_color;
}

void RealSense::Infrared_to_Cv(cv::Mat &image_cv_infrared_left, cv::Mat &image_cv_infrared_right)
{
    frameset = pipe.wait_for_frames();
    rs2::video_frame frame_infrared_left = frameset.get_infrared_frame(1);
    rs2::video_frame frame_infrared_right = frameset.get_infrared_frame(2);
    image_rs_infrared_left = cv::Mat(frame_infrared_left.get_height(), frame_infrared_left.get_width(),
                                     CV_8UC1, (void *)frame_infrared_left.get_data(), cv::Mat::AUTO_STEP);
    image_rs_infrared_right = cv::Mat(frame_infrared_right.get_height(), frame_infrared_right.get_width(),
                                      CV_8UC1, (void *)frame_infrared_right.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(image_rs_infrared_left, image_cv_infrared_left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(image_rs_infrared_right, image_cv_infrared_right, cv::COLOR_GRAY2BGR);
}

// 只是把分割结果覆盖到彩色图像上。
void RealSense::Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray objs)
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

// 在深度图上做可视化叠加
void RealSense::Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray objs)
{
    for (auto &obj : objs)
    {
        if (obj.left >= 0 && obj.right < image_cv_depth.cols && obj.top >= 0 && obj.bottom <= image_cv_depth.rows)
        {
            uint8_t b, g, r;
            std::tie(b, g, r) = yolo::random_color(obj.class_label);
            cv::rectangle(image_cv_depth, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom),
                          cv::Scalar(b, g, r), 5);
            auto name = labels[obj.class_label];
            auto caption = cv::format("%s %.2f", name, obj.confidence);
            int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
            cv::rectangle(image_cv_depth, cv::Point(obj.left - 3, obj.top - 33),
                          cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
            cv::putText(image_cv_depth, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
            if (obj.seg)
            {
                if (obj.left >= 0 && obj.seg->width >= 0 && obj.left + obj.seg->width < image_cv_depth.cols &&
                    obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_cv_depth.rows)
                {
                    mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                    mask.convertTo(mask, CV_8UC1);
                    cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR);
                    cv::addWeighted(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask, 1.0, 0.0, mask);
                    mask.copyTo(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
                }
            }
        }
    }
}

void RealSense::Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    cloud.clear();
    rs2::depth_frame frame_depth = frameset.get_depth_frame();
    for (int u = 0; u < frame_depth.get_width(); u += 10)
    {
        for (int v = 0; v < frame_depth.get_height(); v += 10)
        {
            float depth_value = frame_depth.get_distance(u, v);
            if (depth_value != 0)
            {
                float x = (u - intrinsics_depth.ppx) * depth_value / intrinsics_depth.fx;
                float y = (v - intrinsics_depth.ppy) * depth_value / intrinsics_depth.fy;
                float z = depth_value;
                pcl::PointXYZ point(x, y, z);
                cloud.push_back(point);
            }
        }
    }
    std::cout << "Global PointCloud:" << cloud.size() << std::endl;
}

BoundingBox3D RealSense::Value_Mask_to_Pcl(
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
    bbox.cls_name = "unknown";

    size_t valid_points = 0;

    // depth单位 mm -> m
    const float depth_scale = 0.001f;

    // 相机内参
    float fx = intrinsics_depth.fx;
    float fy = intrinsics_depth.fy;
    float cx = intrinsics_depth.ppx;
    float cy = intrinsics_depth.ppy;

    for (auto &obj : objs)
    {
        bbox.cls_name = obj.class_label;

        // 生成 mask
        cv::Mat mask;
        if (obj.seg)
        {
            mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
            mask.convertTo(mask, CV_8UC1);
            cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top));
        }
        else
        {
            mask = cv::Mat::zeros(obj.bottom - obj.top, obj.right - obj.left, CV_8UC1);
            cv::rectangle(mask, cv::Rect(0, 0, mask.cols, mask.rows), cv::Scalar(255), cv::FILLED);
        }

        // 遍历 mask 提取点云
        for (int v = 0; v < mask.rows; ++v)
        {
            for (int u = 0; u < mask.cols; ++u)
            {
                if (mask.at<uchar>(v, u) == 0)
                    continue;

                int px = obj.left + u;
                int py = obj.top + v;

                if (px < 0 || px >= depth_image.cols || py < 0 || py >= depth_image.rows)
                    continue;

                uint16_t d = depth_image.at<uint16_t>(py, px);
                if (d == 0)
                    continue; // depth无效

                float depth_value = d * depth_scale;

                // 坐标转换
                float X = (px - cx) * depth_value / fx;
                float Y = -(py - cy) * depth_value / fy; // 保持Y向上
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

    if (valid_points > 0)
    {
        bbox.center.x /= valid_points;
        bbox.center.y /= valid_points;
        bbox.center.z /= valid_points;
    }
    else
    {
        bbox.center = cv::Point3f(0, 0, 0);
        return bbox; // 无点
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    // PCA 主方向
    if (!cloud->empty())
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        Eigen::Matrix3f cov;
        pcl::computeCovarianceMatrixNormalized(*cloud, centroid, cov);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
        solver.compute(cov, Eigen::ComputeEigenvectors);
        Eigen::Vector3f principal = solver.eigenvectors().col(2);

        bbox.principal_dir = cv::Vec3f(principal(0), principal(1), principal(2));
    }

    return bbox;
}

void RealSense::Save_Image(int amount, std::string output_dir)
{
    if (amount <= frame_count)
    {
        return;
    }
    frameset = pipe.wait_for_frames();
    rs2::video_frame frame_infrared = frameset.get_infrared_frame(1);
    cv::Mat image_infrared_saved = cv::Mat(frame_infrared.get_height(), frame_infrared.get_width(),
                                           CV_8UC1, (void *)frame_infrared.get_data());
    image_infrared_saved.convertTo(image_infrared_saved, cv::COLOR_GRAY2BGR);
    string filename = output_dir + "objs_" + to_string(frame_count) + ".png";
    if (cv::imwrite(filename, image_infrared_saved))
    {
        COUT_YELLOW_START;
        cout << "Save objs_" << frame_count << ".png Success!" << endl;
        COUT_COLOR_END;
        frame_count++;
    }
    else
    {
        COUT_RED_START;
        cout << "Save error!" << endl;
        COUT_COLOR_END;
    }
    cv::imshow("Infrared Image", image_infrared_saved);
    cv::waitKey(10);
    usleep(50000);
}

void RealSense::Mask_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                  const cv::Mat &mask)
{
    cloud.clear();
    rs2::depth_frame frame_depth = frameset.get_depth_frame();

    for (int u = 0; u < frame_depth.get_width(); u += 1) // 不要太稀疏，避免漏点
    {
        for (int v = 0; v < frame_depth.get_height(); v += 1)
        {
            // 如果 mask 尺寸和深度图一致
            if (mask.at<uchar>(v, u) == 0)
                continue; // 跳过非目标区域

            float depth_value = frame_depth.get_distance(u, v);
            if (depth_value > 0)
            {
                float x = (u - intrinsics_depth.ppx) * depth_value / intrinsics_depth.fx;
                float y = (v - intrinsics_depth.ppy) * depth_value / intrinsics_depth.fy;
                float z = depth_value;
                pcl::PointXYZ point(x, y, z);
                cloud.push_back(point);
            }
        }
    }
    std::cout << "Mask PointCloud:" << cloud.size() << std::endl;
}
bool RealSense::Capture(cv::Mat &color, cv::Mat &depth)
{
    static rs2::pipeline pipe;
    static auto last_capture_time = std::chrono::steady_clock::now();

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_capture_time);
    if (elapsed.count() < 1000)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 - elapsed.count()));
    }
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    rs2::frame depth_frame = frames.get_depth_frame();

    if (!color_frame || !depth_frame)
        return false;

    color = cv::Mat(cv::Size(color_frame.as<rs2::video_frame>().get_width(),
                             color_frame.as<rs2::video_frame>().get_height()),
                    CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

    depth = cv::Mat(cv::Size(depth_frame.as<rs2::video_frame>().get_width(),
                             depth_frame.as<rs2::video_frame>().get_height()),
                    CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

    cv::cvtColor(color, color, cv::COLOR_BGR2RGB); // 转换为OpenCV默认格式
    last_capture_time = std::chrono::steady_clock::now();
    return true;
}