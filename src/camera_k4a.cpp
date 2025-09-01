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
        image_cv_depth.convertTo(image_cv_depth, CV_8U);
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
                    // cv::Mat mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                    // mask.convertTo(mask, CV_8UC1);
                    // // adjust mask size to fit the object
                    // cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR);
                    // cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
                    // cv::addWeighted(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask, 0.8, 0.0, mask);
                    // mask.copyTo(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
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
void K4a::Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud, yolo::BoxArray &objs)
{
    cloud.clear();
    uint16_t *depth_data = (uint16_t *)image_k4a_depth_to_color.get_buffer();
    for (auto &obj : objs)
    {
        for (int v = obj.top; v < obj.top + 4 * obj.seg->height; v += 2)
        {
            for (int u = obj.left; u < obj.right; u += 2)
            {
                if (u < image_k4a_depth_to_color.get_width_pixels() && v < image_k4a_depth_to_color.get_height_pixels())
                {
                    float depth_value = static_cast<float>(depth_data[v * image_k4a_depth_to_color.get_width_pixels() + u] / 1000.0);
                    if (depth_value != 0)
                    {
                        float x = (u - color_intrinsics.intrinsics.parameters.param.cx) * depth_value / color_intrinsics.intrinsics.parameters.param.fx;
                        float y = (v - color_intrinsics.intrinsics.parameters.param.cy) * depth_value / color_intrinsics.intrinsics.parameters.param.fy;
                        float z = depth_value;
                        cloud.push_back(pcl::PointXYZ(x, y, z));
                    }
                }
            }
        }
    }
    std::cout << "Mask PointCloud:" << cloud.size() << std::endl;
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
