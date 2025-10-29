#include <iostream>

#include <opencv2/opencv.hpp>

#include "camera_k4a.hpp"
#include "myinfer.hpp"
#include <main.hpp>

// 条件编译ROS支持
#ifdef BUILD_WITH_ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

// 自定义检测消息类型（简单版本）
struct DetectionMsg
{
    std::string class_name;
    geometry_msgs::Point min_pt;
    geometry_msgs::Point max_pt;
    geometry_msgs::Point center;
    geometry_msgs::Vector3 principal_dir;
};
#endif
int main(
#ifdef BUILD_WITH_ROS
    int argc, char **argv
#endif
)
{
#ifdef BUILD_WITH_ROS
    // ROS初始化
    ros::init(argc, argv,  "k4a_detector");
    ros::NodeHandle nh;

    // 创建发布者
    ros::Publisher bbox_pub = nh.advertise<std_msgs::String>("/k4a/bounding_boxes", 10);
    ros::Publisher color_pub = nh.advertise<sensor_msgs::Image>("/k4a/color_image", 10);
    ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>("/k4a/depth_image", 10);

    ROS_INFO("K4A Detector started with ROS support");
#else
    std::cout << "[INFO] K4A Detector started (non-ROS mode)" << std::endl;
#endif

    try
    {
        K4a k4a_device; // 类里是构造函数，自动调用configuration函数
        // k4a_device.Configuration(); //不要有这一句，会报错
        Yolo yolo;                                                                        // 实例化类
        std::string engine_path = "/home/li/camera_cxx/workspace/model_temp/best.engine"; // 引擎路径

        yolo::BoxArray detections;

        bool first_cloud = true;                                                       // 第一次生成点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 定义点云指针
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(0.2);
        // yolo.Yolov8_Enable(engine_path);  //检测引擎适用于yolov8s.engine
        yolo.Yolov8_Seg_Enable(engine_path); // 分割引擎适用于best.engine

        while (true)
        {
            cv::Mat color_image, depth_image;
            cv::Mat depth_display;

            k4a_device.Image_to_Cv(color_image, depth_image); // 获取彩色和深度图像

            yolo.Single_Inference(color_image, detections);

            k4a_device.Color_With_Mask(color_image, detections); // 显示带有检测结果的彩色图像

            depth_image.convertTo(depth_display, CV_8U, 255.0 / 4000.0);
            k4a_device.Depth_With_Mask(depth_display, detections); // 用显示图，不用原始 CV_16U

            static int frame_id = 0;

            BoundingBox3D bbox = k4a_device.Value_Mask_to_Pcl(cloud, depth_image, detections);
            frame_id++;
            if (frame_id % 10 == 0)
            { // 每10帧打印一次
                std::cout << "BBox for class: " << bbox.cls_name << std::endl;
                std::cout << "  Min: [" << bbox.min_pt.x << ", " << bbox.min_pt.y << ", " << bbox.min_pt.z << "]" << std::endl;
                std::cout << "  Max: [" << bbox.max_pt.x << ", " << bbox.max_pt.y << ", " << bbox.max_pt.z << "]" << std::endl;
                std::cout << "  Center: [" << bbox.center.x << ", " << bbox.center.y << ", " << bbox.center.z << "]" << std::endl;
                std::cout << "  Principal dir: [" << bbox.principal_dir[0] << ", " << bbox.principal_dir[1] << ", " << bbox.principal_dir[2] << "]" << std::endl;
            }
#ifdef BUILD_WITH_ROS
            // 发布边界框信息到ROS
            std_msgs::String bbox_msg;
            std::stringstream ss;
            ss << "Class: " << bbox.cls_name
               << " | Center: [" << bbox.center.x << ", " << bbox.center.y << ", " << bbox.center.z << "]"
               << " | Min: [" << bbox.min_pt.x << ", " << bbox.min_pt.y << ", " << bbox.min_pt.z << "]"
               << " | Max: [" << bbox.max_pt.x << ", " << bbox.max_pt.y << ", " << bbox.max_pt.z << "]";
            bbox_msg.data = ss.str();
            bbox_pub.publish(bbox_msg);

            // 发布彩色图像
            if (!color_image.empty())
            {
                cv_bridge::CvImage color_bridge;
                color_bridge.header.stamp = ros::Time::now();
                color_bridge.header.frame_id = "k4a_camera";
                color_bridge.encoding = "bgr8";
                color_bridge.image = color_image.clone();
                color_pub.publish(color_bridge.toImageMsg());
            }

            // 发布深度图像
            if (!depth_display.empty())
            {
                cv_bridge::CvImage depth_bridge;
                depth_bridge.header.stamp = ros::Time::now();
                depth_bridge.header.frame_id = "k4a_camera";
                depth_bridge.encoding = "mono8";
                depth_bridge.image = depth_display.clone();
                depth_pub.publish(depth_bridge.toImageMsg());
            }
#endif
            if (!color_image.empty())
                cv::imshow(" Seg Color Image", color_image); // 显示彩色图像
            if (!depth_display.empty())
            {
                cv::imshow("Depth Image", depth_display);
            }

            //  更新 PCL 可视化器
            if (first_cloud)
            {
                viewer->addPointCloud<pcl::PointXYZ>(cloud, "target_cloud");
                viewer->resetCameraViewpoint("target_cloud"); // 自动对准目标点云
                first_cloud = false;
            }
            else
            {
                viewer->updatePointCloud<pcl::PointXYZ>(cloud, "target_cloud");
            }

            char key = (char)cv::waitKey(10);
            if (key == 'q' || key == 27) // 按 q 或 Esc 键退出
                break;
            viewer->spinOnce(10); // 更新显示
#ifdef BUILD_WITH_ROS
            // ROS循环控制
            ros::spinOnce();
#endif
        }
        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
