#pragma once
#include <iostream>
#include <cmath>

inline int getRowFromY(float y, int img_height = 720)
{
    int row = static_cast<int>(y / (img_height / 4.0));
    return std::min(row, 2); // 防止越界
}

inline int getColFromX(float x, int img_width = 1280)
{
    int col = static_cast<int>(x / (img_width / 3.0));
    return std::min(col, 3);
}
struct Pose {
    double x;     // 单位：米
    double y;     // 单位：米
    double theta; // 朝向角，弧度
};