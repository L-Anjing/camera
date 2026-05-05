#include "utils/block_recognizer.hpp"
#include <cmath> // 引入数学库用于 std::sqrt 和 std::pow
#include <set>

// 核心识别逻辑：处理所有blocks
FinalBlockResults BlockRecognizer::recognize(const yolo::BoxArray& dets) const {

    FinalBlockResults results;

    // 1. 收集所有 blocks 和 faces
    std::vector<const yolo::Box*> blocks;
    std::vector<const yolo::Box*> faces;
    
    for (const auto& d : dets) {
        if (d.class_label == 0) { // block
            blocks.push_back(&d);
        } else {
            faces.push_back(&d);
        }
    }

    // 没有检测到 block 直接返回空
    if (blocks.empty()) return results;

    // 设置一个基础的置信度阈值，过滤掉网络瞎猜的假 face 框
    const float FACE_CONF_THRESHOLD = 0.60f; 

    // 2. 对每个 block 进行处理融合
    for (const auto& block : blocks) {
        cv::Rect block_rect(
            block->left, block->top,
            block->right - block->left,
            block->bottom - block->top
        );

        // 预先计算 Block 的几何特征（用于寻找几何中心最正的面）
        cv::Point2f block_center(
            block->left + block_rect.width / 2.0f,
            block->top + block_rect.height / 2.0f
        );
        float block_area = block_rect.area();
        float diag_length = std::sqrt(std::pow(block_rect.width, 2) + std::pow(block_rect.height, 2)) + 1e-5f;

        std::set<int> face_classes;
        const yolo::Box* best_face_ptr = nullptr; 
        float max_geom_score = -1e9f; // 记录几何最高分

        // 3. 遍历所有 face，关联到当前的 block
        for (auto f : faces) {
            if (f == nullptr) continue; 
            
            // 保类别:置信度太低的假框直接跳过，不让它污染类别判定
            if (f->confidence < FACE_CONF_THRESHOLD) continue;

            cv::Rect fr(
                f->left, f->top,
                f->right - f->left,
                f->bottom - f->top
            );

            float inter_area = (block_rect & fr).area();
            
            // 判定包含关系：面必须大部分在 block 内部 (IoF > 0.7)
            if (inter_area > 0.7f * fr.area()) {
                // 记录该 block 上出现的所有真实面类别
                face_classes.insert(f->class_label);
                
                //找正面:用纯几何打分法选出用于下游点云截取的最佳图案
                float face_area_ratio = fr.area() / block_area;
                
                cv::Point2f face_center(
                    f->left + fr.width / 2.0f,
                    f->top + fr.height / 2.0f
                );
                float dx = face_center.x - block_center.x;
                float dy = face_center.y - block_center.y;
                float center_offset = std::sqrt(dx * dx + dy * dy) / diag_length;

                // 纯几何打分：面积越大越好 (权重 1.0)，偏心越小越好 (权重 -1.5)
                float geom_score = 1.0f * face_area_ratio - 1.5f * center_offset;

                if (geom_score > max_geom_score) {
                    best_face_ptr = f;
                    max_geom_score = geom_score;
                }
            }
        }

        // 根据收集到的所有高质量面，判定 Block 的整体类别
        BlockClass block_class = BlockClass::UNKNOWN;
        
        if (!face_classes.empty()) {
            bool all_r1 = true;
            bool has_r2r = false;
            bool has_r2f = false;

            // 一次循环完成所有特征的统计
            for (int c : face_classes) {
                if (c != 1) all_r1 = false;
                if (c >= 2 && c <= 16) has_r2r = true;
                if (c >= 17 && c <= 31) has_r2f = true;
            }
            
            // 判定逻辑树
            if (all_r1) {
                block_class = BlockClass::R1;
            } else if (has_r2r) {
                block_class = BlockClass::R2r; // 优先判定 R2r
            } else if (has_r2f) {
                block_class = BlockClass::R2f;
            }
        }

        // 4. 组装结果：只有当 Block 类别明确，且成功找到了正面框时，才输出结果
        if (block_class != BlockClass::UNKNOWN && best_face_ptr != nullptr) {
            FinalBlockResult result;
            result.detection = *block;
            result.detection.class_label = static_cast<int>(block_class);
            result.block_class = block_class;
            
            // 将几何上最正的面交给下游，供点云提取或 ICP 使用
            result.best_pattern = *best_face_ptr; 

            // 整体置信度融合（融合 block 的检测确信度和正面 face 的检测确信度）
            result.confidence = 0.5f * block->confidence + 0.5f * best_face_ptr->confidence;
            result.detection.confidence = result.confidence;
            
            results.push_back(result);
        }
    }

    return results;
}