#include "utils/block_recognizer.hpp"
#include "usbcam/usbcam.hpp"

#include <algorithm>
#include <cmath>
#include <set>

// YOLO 检测融合为 block 结果
FinalBlockResults BlockRecognizer::recognize(const yolo::BoxArray &dets) const
{
    FinalBlockResults results;

    // 分离 block 和 face 检测
    std::vector<const yolo::Box *> blocks;
    std::vector<const yolo::Box *> faces;
    for (const auto &d : dets)
    {
        if (d.class_label == 0)
        {
            blocks.push_back(&d);
        }
        else
        {
            faces.push_back(&d);
        }
    }

    if (blocks.empty())
    {
        return results;
    }

    const float kFaceConfThreshold = 0.50f;

    for (const auto &block : blocks)
    {
        cv::Rect block_rect(
            block->left, block->top,
            block->right - block->left,
            block->bottom - block->top);

        // 用 block 中心和尺度评估 face 是否更居中
        cv::Point2f block_center(
            block->left + block_rect.width / 2.0f,
            block->top + block_rect.height / 2.0f);
        const float block_area = block_rect.area();
        const float diag_length = std::sqrt(std::pow(block_rect.width, 2) + std::pow(block_rect.height, 2)) + 1e-5f;

        std::set<int> face_classes;
        const yolo::Box *best_face_ptr = nullptr;
        float best_face_score = -1e9f;
        std::vector<FaceCandidate> all_candidates;

        // 关联当前 block 的所有 face
        for (const auto *f : faces)
        {
            if (f == nullptr || f->confidence < kFaceConfThreshold)
            {
                continue;
            }

            cv::Rect fr(
                f->left, f->top,
                f->right - f->left,
                f->bottom - f->top);

            const float inter_area = (block_rect & fr).area();
            if (inter_area <= 0.7f * fr.area())
            {
                continue;
            }

            face_classes.insert(f->class_label);

            const float face_area_ratio = fr.area() / block_area;
            cv::Point2f face_center(
                f->left + fr.width / 2.0f,
                f->top + fr.height / 2.0f);
            const float dx = face_center.x - block_center.x;
            const float dy = face_center.y - block_center.y;
            const float center_offset = std::sqrt(dx * dx + dy * dy) / diag_length;
            const float geom_score = 1.0f * face_area_ratio - 1.5f * center_offset;

            all_candidates.push_back({*f, geom_score, f->class_label});

            if (geom_score > best_face_score)
            {
                best_face_ptr = f;
                best_face_score = geom_score;
            }
        }

        std::sort(all_candidates.begin(), all_candidates.end(),
                  [](const FaceCandidate &a, const FaceCandidate &b)
                  {
                      return a.geom_score > b.geom_score;
                  });

        BlockClass block_class = BlockClass::UNKNOWN;
        if (!face_classes.empty())
        {
            bool all_r1 = true;
            bool has_r2r = false;
            bool has_r2f = false;

            for (int c : face_classes)
            {
                if (c != 1)
                {
                    all_r1 = false;
                }
                if (c >= 2 && c <= 16)
                {
                    has_r2r = true;
                }
                if (c >= 17 && c <= 31)
                {
                    has_r2f = true;
                }
            }

            if (all_r1)
            {
                block_class = BlockClass::R1;
            }
            else if (has_r2r)
            {
                block_class = BlockClass::R2r;
            }
            else if (has_r2f)
            {
                block_class = BlockClass::R2f;
            }
        }

        if (block_class == BlockClass::UNKNOWN || best_face_ptr == nullptr)
        {
            continue;
        }

        FinalBlockResult result;
        result.detection = *block;
        result.detection.class_label = static_cast<int>(block_class);
        result.block_class = block_class;
        result.best_pattern = *best_face_ptr;
        result.candidates = std::move(all_candidates);
        result.confidence = 0.5f * block->confidence + 0.5f * best_face_ptr->confidence;
        result.detection.confidence = result.confidence;
        results.push_back(result);
    }

    return results;
}

#ifndef BLOCK_RECOGNIZER_STANDALONE_ONLY
TargetSelection select_best_target(const UsbCam &cam,
                                   const FinalBlockResults &blocks,
                                   float face_size_m)
{
    TargetSelection best;

    for (const auto &result : blocks)
    {
        // 先估位姿，再按距离和置信度选主目标
        PoseEstimate pose = cam.estimate_pose_from_pattern(result, face_size_m);
        const auto &box = result.best_pattern;
        const float width = std::max(1.0f, box.right - box.left);
        const float height = std::max(1.0f, box.bottom - box.top);
        const float area = width * height;
        const float range_bonus = pose.valid ? 1.0f / std::max(0.05f, pose.robot_center.norm()) : 1.0f;
        const float score = std::max(0.01f, result.confidence) * area * range_bonus;

        if (score > best.score)
        {
            best.result = &result;
            best.pose = pose;
            best.score = score;
        }
    }

    return best;
}
#endif
