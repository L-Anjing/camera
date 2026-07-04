#include <iostream>
#include <csignal>
#include <chrono>
#include <iomanip>

#include "utils/block_recognizer.hpp"
#include "utils/kalman_tracker.hpp"
#include "orbbec/camera_orbbec.hpp"
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
    auto node = rclcpp::Node::make_shared("orbbec_detector");
    signal(SIGINT, sigintHandler);

    RCLCPP_INFO(node->get_logger(), "Inference input: fixed grayscale");

    auto target_pub =
        node->create_publisher<std_msgs::msg::String>("/target_info", 10);

    try
    {
        std::string config_path = "/home/pi/workspace/camera_ws/src/camera_bridge/config/OrbbecConfig.yaml";
        Orbbec orbbec_device = Orbbec::Create_FromFile(config_path);
        Yolo yolo;
        BlockRecognizer block_recognizer;

        std::string engine_path =
            "/home/pi/workspace/camera_ws/src/camera_bridge/workspace/models/260425.engine";

        yolo.Yolov8_Enable(engine_path);
        yolo.Set_Confidence_Threshold(0.5f, 0.5f);

        yolo::BoxArray detections;
        int frame_id = 0;

        TrackManager track_manager;
        auto prev_time = std::chrono::steady_clock::now();

        while (!stop_flag)
        {
            auto frame_start = std::chrono::steady_clock::now();
            const bool debug_frame = (frame_id % 5 == 0);
            cv::Mat color_image, depth_image;
            orbbec_device.Image_to_Cv(color_image, depth_image);
            if (color_image.empty() || depth_image.empty())
            {
                std::cerr << "get_capture timeout" << std::endl;
                ++frame_id;
                continue;
            }

            detections.clear();
            yolo.Single_Inference_Letterbox_Gray(color_image, detections, 640);

            if (debug_frame)
            {
                cv::Mat yolo_vis = color_image.clone();
                vision::draw_yolo_detections(yolo_vis, detections);
                cv::imshow("YOLO Debug", yolo_vis);
            }

            FinalBlockResults blocks_patterns = block_recognizer.recognize(detections);

            cv::Mat block_vis;
            if (debug_frame)
            {
                block_vis = color_image.clone();
            }

            const FinalBlockResult *target =
                select_highest_confidence_target(blocks_patterns);

            if (target != nullptr)
            {
                if (debug_frame)
                {
                    for (const auto &results : blocks_patterns)
                    {
                        vision::draw_block_result(block_vis, results);
                    }
                }

                const Eigen::Matrix3f &Rmat = orbbec_device.get_rotation();
                const Eigen::Vector3f &Tvec = orbbec_device.get_translation();

                Eigen::Vector3f normal_cam;
                Eigen::Vector3f center_cam = orbbec_device.compute_precise_center(
                    depth_image, target->detection, target->best_pattern, &normal_cam);

                BoundingBox3D bbox;
                const char *class_name = block_class_name(target->block_class);
                bbox.cls_ID = static_cast<int>(target->block_class);
                bbox.cls_name = class_name;

                if (center_cam.x() != 0 || center_cam.y() != 0 || center_cam.z() != 0)
                {
                    Eigen::Vector3f center_robot = Rmat * center_cam + Tvec;
                    bbox.center.x = center_robot.x();
                    bbox.center.y = center_robot.y();
                    bbox.center.z = center_robot.z();

                    Eigen::Vector3f n_robot = Rmat * normal_cam;
                    Eigen::Vector3f dir_to_robot = -center_robot.normalized();
                    float dot = std::abs(n_robot.dot(dir_to_robot));

                    if (dot < 0.5f)
                    {
                        bbox.center.x = bbox.center.y = bbox.center.z = 0;
                        std::cout << "[NORMAL] dot=" << dot
                                  << " < 0.5, reject\n";
                    }
                }

                auto now = std::chrono::steady_clock::now();
                float dt = std::chrono::duration<float>(now - prev_time).count();
                prev_time = now;
                dt = std::min(dt, 0.5f);

                Eigen::Vector3f gyro = orbbec_device.get_gyro();
                Eigen::Vector3f raw_center(bbox.center.x, bbox.center.y, bbox.center.z);
                Eigen::Vector3f kf_center = track_manager.process(
                    raw_center, dt, bbox.cls_ID, target->confidence, &gyro);

                bbox.center.x = kf_center.x();
                bbox.center.y = kf_center.y();
                bbox.center.z = kf_center.z();
                bbox.principal_dir[0] = std::atan2(bbox.center.y, bbox.center.x);

                if (debug_frame)
                {
                    auto now = std::chrono::steady_clock::now();
                    float ms = std::chrono::duration<float>(now - frame_start).count();
                    std::cout << "Block Class: " << class_name
                              << " Confidence: " << target->confidence
                              << " Center: ["
                              << bbox.center.x << ", "
                              << bbox.center.y << ", "
                              << bbox.center.z << ", "
                              << bbox.principal_dir[0] << "] Time: "
                              << std::fixed << std::setprecision(3) << ms << "s\n";
                }

                std_msgs::msg::String msg;
                std::stringstream ss;
                ss << bbox.cls_ID << ","
                   << bbox.center.x << ","
                   << bbox.center.y << ","
                   << bbox.center.z << ","
                   << bbox.principal_dir[0];
                msg.data = ss.str();
                target_pub->publish(msg);
            }
            else
            {
                std_msgs::msg::String msg;
                std::stringstream ss;
                ss << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0;
                msg.data = ss.str();
                target_pub->publish(msg);
            }

            if (debug_frame && !block_vis.empty())
            {
                cv::imshow("Final Block", block_vis);
            }

            char key = (char)cv::waitKey(10);
            if (key == 'q' || key == 27)
                break;

            rclcpp::spin_some(node);
            ++frame_id;
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
