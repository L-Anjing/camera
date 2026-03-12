#include "utils/vision_draw.hpp"
#include "utils/myinfer.hpp" // yolo_labels, random_color

namespace vision
{

    // Block 语义颜色（核心）
    static inline cv::Scalar block_color(BlockClass c)
    {
        switch (c)
        {
        case BlockClass::R1:
            return cv::Scalar(255, 0, 0); // 蓝
        case BlockClass::R2r:
            return cv::Scalar(0, 255, 0); // 绿
        case BlockClass::R2f:
            return cv::Scalar(0, 0, 255); // 红
        default:
            return cv::Scalar(128, 128, 128); // UNKNOWN
        }
    }

    static inline cv::Scalar class_color(int class_id)
    {
        // 预定义高对比 BGR 颜色（可扩展到 32 类）
        static const cv::Scalar color_table[32] = {
            {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {128, 0, 0}, {0, 128, 0}, {0, 0, 128}, {128, 128, 0}, {128, 0, 128}, {0, 128, 128}, {64, 0, 0}, {0, 64, 0}, {0, 0, 64}, {64, 64, 0}, {64, 0, 64}, {0, 64, 64}, {192, 0, 0}, {0, 192, 0}, {0, 0, 192}, {192, 192, 0}, {192, 0, 192}, {0, 192, 192}, {128, 128, 128}, {64, 128, 128}, {128, 64, 128}, {128, 128, 64}, {64, 64, 128}, {64, 128, 64}, {128, 64, 64}, {192, 128, 64}};

        int idx = class_id % 32;
        return color_table[idx];
    }

    void draw_yolo_detections(
        cv::Mat &image,
        const yolo::BoxArray &objs,
        int thickness)
    {
        for (const auto &obj : objs)
        {
            cv::Rect rect(
                obj.left,
                obj.top,
                obj.right - obj.left,
                obj.bottom - obj.top);

            cv::Scalar color = class_color(obj.class_label);

            // 框
            cv::rectangle(image, rect, color, thickness);

            // 文本
            std::string text = cv::format(
                "%s %.2f",
                yolo_labels[obj.class_label],
                obj.confidence);

            cv::putText(
                image,
                text,
                {rect.x, rect.y - 6},
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2);
        }
    }
    void draw_yolo_masks(cv::Mat &img, const yolo::BoxArray &objs, float alpha)
    {
        for (const auto &obj : objs)
        {
            if (!obj.seg || obj.seg->data == nullptr)
                continue;

            int left = obj.left;
            int top = obj.top;
            int width = obj.right - obj.left;
            int height = obj.bottom - obj.top;

            if (left < 0 || top < 0 ||
                left + width > img.cols ||
                top + height > img.rows ||
                width <= 0 || height <= 0)
                continue;

            cv::Mat raw_mask(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
            raw_mask.convertTo(raw_mask, CV_8UC1);

            cv::Mat mask;
            cv::resize(raw_mask, mask, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);

            cv::Scalar color = class_color(obj.class_label);

            cv::Mat color_mask(mask.size(), CV_8UC3, color);
            cv::Mat mask_3c;
            cv::cvtColor(mask, mask_3c, cv::COLOR_GRAY2BGR);

            cv::Mat region = img(cv::Rect(left, top, width, height));
            cv::Mat blended;
            cv::addWeighted(region, 1 - alpha, color_mask, alpha, 0, blended);

            blended.copyTo(region, mask);
        }
    }

    // Block 最终结果绘制
    void draw_block_result(
        cv::Mat &image,
        const FinalBlockResult &block,
        int thickness)
    {
        // Block 有有效的 detection box
        const auto &box = block.detection;

        cv::Rect rect(
            box.left,
            box.top,
            box.right - box.left,
            box.bottom - box.top);

        cv::Scalar color = block_color(block.block_class);

        cv::rectangle(image, rect, color, thickness);

        const char *name = block_class_name(block.block_class);

        std::string label =
            cv::format("%s %.2f", name, block.confidence);

        cv::putText(
            image,
            label,
            {rect.x, rect.y - 6},
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2);
    }

} // namespace vision
