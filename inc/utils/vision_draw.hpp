#pragma once
#include <opencv2/opencv.hpp>

#include "utils/block_recognizer.hpp"
#include "common/yolo.hpp"

#include <string>
namespace vision
{

    // 画 detection（bbox + label + conf）
    void draw_yolo_detections(
        cv::Mat &image,
        const yolo::BoxArray &objs,
        int thickness = 2);
        
    // 画分割 mask
    void draw_yolo_masks(
        cv::Mat &image,
        const yolo::BoxArray &objs,
        float alpha = 0.5f);

    // 单独画一个 box（以后给 block 用）
    void draw_block_result(
        cv::Mat &image,
        const FinalBlockResult &block,
        int thickness = 3);

    // 绘制3D立方体中心点投影（用于深度验证）
    void draw_3d_center_projection(
        cv::Mat &image,
        float center_x, float center_y, float center_z,
        float fx, float fy, float cx, float cy);


}

