#include "ros_interface.hpp"
#include "utils.hpp"

ROSInterface::ROSInterface() {
    // 订阅雷达位姿话题，例如 /lidar_pose
    lidar_sub_ = nh_.subscribe("/lidar_pose", 10, &ROSInterface::lidarCallback, this);

    // 发布一个调试用话题
    pose_pub_  = nh_.advertise<geometry_msgs::Pose2D>("/debug_pose", 10);
}

void ROSInterface::lidarCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    current_pose_.x = msg->x;
    current_pose_.y = msg->y;
    current_pose_.theta = msg->theta;
}

Pose ROSInterface::getLidarPose() const {
    return current_pose_;
}

void ROSInterface::publishDebugInfo(const Pose& p) {
    geometry_msgs::Pose2D msg;
    msg.x = p.x;
    msg.y = p.y;
    msg.theta = p.theta;
    pose_pub_.publish(msg);
}

void ROSInterface::spinOnce() {
    ros::spinOnce();
}
