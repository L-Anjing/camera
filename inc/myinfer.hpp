#ifndef __MYINFER_HPP__
#define __MYINFER_HPP__

#include <opencv2/opencv.hpp>

// #include "cpm.hpp"  // Concurrent Priority Module - 并发优先级模块
#include "infer.hpp"
#include "yolo.hpp"

#include <chrono>

static const char *labels[] = { // TODO: Add labels
    "Basket",
    "Target"};
struct GridDetection
{
    int row;          // 网格行
    int col;          // 网格列
    int class_label;  // 类别索引
    float confidence; // 置信度
    cv::Rect bbox;    // 原始像素框
};
class Yolo
{
private:
    std::string engine;
    yolo::Type type;
    yolo::Image cvimg(const cv::Mat &image);
    bool load_flag;
    std::shared_ptr<yolo::Infer> yolo;

public:
    void Yolov8_Enable(std::string &engine_);

    void Yolov8_Seg_Enable(std::string &engine_seg);

    void Single_Inference(std::string path);

    void Single_Inference(cv::Mat &image);

    void Single_Inference(cv::Mat &image, yolo::BoxArray &objs_out);

    std::vector<GridDetection> Inference_Grid(cv::Mat &image, bool draw_result);

    Yolo();

    ~Yolo();
};

#endif