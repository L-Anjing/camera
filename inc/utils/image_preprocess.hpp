#pragma once

#include <opencv2/opencv.hpp>

namespace vision
{

inline cv::Mat build_infer_input(const cv::Mat &src)
{
    if (src.empty())
    {
        return {};
    }

    if (src.channels() == 1)
    {
        cv::Mat out;
        cv::cvtColor(src, out, cv::COLOR_GRAY2BGR);
        return out;
    }

    cv::Mat gray;
    if (src.channels() == 3)
    {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    }
    else if (src.channels() == 4)
    {
        cv::cvtColor(src, gray, cv::COLOR_BGRA2GRAY);
    }
    else
    {
        return src.clone();
    }

    cv::Mat out;
    cv::cvtColor(gray, out, cv::COLOR_GRAY2BGR);
    return out;
}

} // namespace vision
