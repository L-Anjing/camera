#pragma once

#include <Eigen/Dense>

struct PoseEstimate
{
    Eigen::Vector3f camera_center = Eigen::Vector3f::Zero();
    Eigen::Vector3f robot_center = Eigen::Vector3f::Zero();
    float yaw_z = 0.0f;
    float axis_angle = 0.0f;
    bool valid = false;
};
