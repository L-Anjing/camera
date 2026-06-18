#include <iostream>
#include <csignal>
#include <chrono>

#include "utils/block_recognizer.hpp"
#include "utils/kalman_tracker.hpp"
#include "k4a/camera_k4a.hpp"
#include "utils/myinfer.hpp"
#include "utils/vision_draw.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>

volatile sig_atomic_t stop_flag = 0;

void sigintHandler(int)
{
    stop_flag = 1;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("k4a_detector");
    signal(SIGINT, sigintHandler);

    auto target_pub =
        node->create_publisher<std_msgs::msg::String>("/k4a/target_info", 10);

    try
    {
        // 初始化 - 使用绝对路径或从环境变量获取
        std::string config_path = "/home/pi/workspace/camera_ws/src/camera_bridge/config/K4AConfig.yaml";
        K4a k4a_device = K4a::Create_FromFile(config_path);
        Yolo yolo;
        BlockRecognizer block_recognizer;

        std::string engine_path =
            "/home/pi/workspace/camera_ws/src/camera_bridge/workspace/models/260425.engine";

        yolo.Yolov8_Enable(engine_path);

        // Set confidence threshold to 0.5 (50%) and NMS threshold to 0.5
        yolo.Set_Confidence_Threshold(0.5f, 0.5f);

        yolo::BoxArray detections;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        // 点云可视化初始化
        // pcl::visualization::PCLVisualizer::Ptr viewer(
        //     new pcl::visualization::PCLVisualizer("PointCloud Viewer"));

        // viewer->setBackgroundColor(0, 0, 0);
        // viewer->addCoordinateSystem(0.2);

        int frame_id = 0;

        // ── 卡尔曼滤波 + 航迹管理 ─────────────────────
        TrackManager track_manager;
        auto prev_time = std::chrono::steady_clock::now();

        while (!stop_flag)
        {
            cv::Mat color_image, depth_image;
            cv::Mat gray, gray3;

            k4a_device.Image_to_Cv(color_image, depth_image);
            if (color_image.empty() || depth_image.empty())
            {
                std::cerr << "get_capture timeout" << std::endl;
                continue;
            }
               

            cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(gray, gray3, cv::COLOR_GRAY2BGR);

            // YOLO 推理
            detections.clear();
            yolo.Single_Inference(gray3, detections);

            // YOLO Debug 显示
            cv::Mat yolo_vis = color_image.clone();
            vision::draw_yolo_detections(yolo_vis, detections);
            cv::imshow("YOLO Debug", yolo_vis);

            // Block 融合 - 返回所有检测到的blocks 结构体容器，包含多个结构体对象
            FinalBlockResults blocks_patterns = block_recognizer.recognize(detections);

            // 显示 Final Block 窗口
            cv::Mat block_vis = color_image.clone();

            // 处理所有检测到的blocks
            if (!blocks_patterns.empty())
            {
                for (auto &results : blocks_patterns)
                {
                    // 绘制融合结果
                    vision::draw_block_result(block_vis, results);

                    // 3D 计算（始终用 best_pattern — 几何最正面）
                    BoundingBox3D bbox =
                        k4a_device.Value_Block_to_Pcl(cloud, depth_image, results);
                    const char *class_name = block_class_name(results.block_class);

                    // ── 面选择：PCA 拟合面法向量，选最正对机器人的面 ──
                    // 预期方向 = 物体指向机器人原点 = -center_robot.normalized()
                    if (results.candidates.size() > 1)
                    {
                        const Eigen::Matrix3f &R = k4a_device.get_rotation();
                        Eigen::Vector3f dir_to_robot = -Eigen::Vector3f(
                            bbox.center.x, bbox.center.y, bbox.center.z);
                        dir_to_robot.normalize();

                        int best_label = results.detection.class_label;
                        float best_dot = -1.0f;

                        for (const auto &c : results.candidates)
                        {
                            Eigen::Vector3f n_cam = k4a_device.compute_roi_normal(
                                depth_image, c.box);
                            Eigen::Vector3f n_robot = R * n_cam;     // 转到机器人系
                            float d = std::abs(n_robot.dot(dir_to_robot));
                            if (d > best_dot)
                            {
                                best_dot = d;
                                best_label = c.class_label;
                            }
                        }

                        if (best_label != results.detection.class_label)
                        {
                            bbox.cls_ID = best_label;
                            bbox.cls_name = "NORMAL";
                        }
                    }

                    // ── 卡尔曼滤波 + 航迹管理 ─────────────
                    auto now = std::chrono::steady_clock::now();
                    float dt = std::chrono::duration<float>(now - prev_time).count();
                    prev_time = now;
                    dt = std::min(dt, 0.5f);  // 限幅，防止长时间卡顿时跳变

                    Eigen::Vector3f gyro = k4a_device.get_gyro();
                    Eigen::Vector3f raw_center(bbox.center.x, bbox.center.y, bbox.center.z);
                    Eigen::Vector3f kf_center = track_manager.process(
                        raw_center, dt, bbox.cls_ID, results.confidence, &gyro);

                    bbox.center.x = kf_center.x();
                    bbox.center.y = kf_center.y();
                    bbox.center.z = kf_center.z();
                    bbox.principal_dir[0] = std::atan2(bbox.center.y, bbox.center.x);

                    if (frame_id++ % 5 == 0 )
                    {
                        std::cout << "Block Class: " << class_name
                                  << " Confidence: " << results.confidence
                                  << " Center: ["
                                  << bbox.center.x << ", "
                                  << bbox.center.y << ", "
                                  << bbox.center.z << ", "
                                  << bbox.principal_dir[0] << "]\n";
                    }

                    std_msgs::msg::String msg;
                    std::stringstream ss;
                    ss << bbox.cls_ID << ","
                       << bbox.center.x << ","
                       << bbox.center.y << ","
                       << bbox.center.z << ","
                       << bbox.principal_dir[0]; // yaw
                    msg.data = ss.str();
                    target_pub->publish(msg);

                    // // PCL 显示
                    // if (first_cloud)
                    // {
                    //     viewer->addPointCloud<pcl::PointXYZ>(cloud, "target_cloud");
                    //     viewer->resetCameraViewpoint("target_cloud");
                    //     first_cloud = false;
                    // }
                    // else
                    // {
                    //     viewer->updatePointCloud<pcl::PointXYZ>(cloud, "target_cloud");
                    // }
                }
            }
            else
            {
                std_msgs::msg::String msg;
                std::stringstream ss;
                ss << 0 << ","   // cls_ID (UNKNOWN)
                   << 0 << ","   // center.x
                   << 0 << ","   // center.y
                   << 0 << ","   // center.z
                   << 0;          // yaw
                msg.data = ss.str();
                target_pub->publish(msg);
            }

            cv::imshow("Final Block", block_vis);
            // cv::imshow("depth_iamge",depth_image);

            // viewer->spinOnce(10);

            char key = (char)cv::waitKey(10);
            if (key == 'q' || key == 27)
                break;

            rclcpp::spin_some(node);
        }

        rclcpp::shutdown();

        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
