#include "realsense.hpp"
#include "main.hpp"
#include "myinfer.hpp"

using namespace std;

void RealSense::Configuration_Default()
{
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    profile = pipe.start(cfg);
    rs2::video_stream_profile color_profile =
        profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    intrinsics_color = color_profile.get_intrinsics();
    
    // gpt generate
    auto depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    intrinsics_depth  = depth_profile.get_intrinsics();


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
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
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
    image_rs_depth.convertTo(image_rs_depth, CV_8UC1, 255.0 / 1000.0);

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


 void RealSense::Color_With_Mask(cv::Mat &image_cv_color,yolo::BoxArray objs)
 {
      for (auto &obj : objs)
     {
         if(obj.left >=0 && obj.right < image_cv_color.cols && obj.top >= 0 && obj.bottom <= image_cv_color.rows)
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
             if (obj.seg)
             {
                 if(obj.left >= 0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_cv_color.cols &&
                     obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_cv_color.rows)
                     {
                         mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                         mask.convertTo(mask, CV_8UC1);
                         cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR);
                         cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
                         cv::addWeighted(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask, 0.8, 0.0, mask);
                         mask.copyTo(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
                     }
                 }
         }
     }
 }

void RealSense::Depth_With_Mask(cv::Mat &image_cv_depth,yolo::BoxArray objs)
{
     for (auto &obj : objs)
    {
        if(obj.left >=0 && obj.right < image_cv_depth.cols && obj.top >= 0 && obj.bottom <= image_cv_depth.rows)
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
                if(obj.left >= 0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_cv_depth.cols &&
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

void RealSense::Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
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