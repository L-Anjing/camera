#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include "utils.hpp"

class ROSInterface {
public:
    ROSInterface();
    void spinOnce();                       // 每帧调用
    Pose getLidarPose() const;             // 获取雷达位姿数据
    void publishDebugInfo(const Pose& p);  // 发布调试数据

private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Publisher  pose_pub_;

    Pose current_pose_;

    void lidarCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
};
