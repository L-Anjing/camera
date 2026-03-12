#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <set>
#include <string>
#include "common/yolo.hpp" //yolo::Box / BoxArray

// Block 枚举（最终语义）
enum class BlockClass
{
    UNKNOWN = 0,
    R1 = 1,
    R2r = 2,
    R2f = 3
};
// Block 名称（统一）
static inline const char *block_class_name(BlockClass c)
{
    switch (c)
    {
    case BlockClass::R1:
        return "R1";
    case BlockClass::R2r:
        return "R2_Real";
    case BlockClass::R2f:
        return "R2_Fake";
    default:
        return "UNKNOWN";
    }
}

// 最终识别结果（单个block）
struct FinalBlockResult
{
    yolo::Box detection;        // 单个最终 block
    BlockClass block_class;
    float confidence;
    yolo::Box best_pattern;

    FinalBlockResult() : block_class(BlockClass::UNKNOWN), confidence(0.0f) {}
    
    bool valid() const
    {
        return block_class != BlockClass::UNKNOWN;
    }
};

// 批量识别结果（多个blocks）
typedef std::vector<FinalBlockResult> FinalBlockResults;

// BlockRecognizer

class BlockRecognizer
{
public:
    BlockRecognizer() = default;

    // 从 YOLO dets 中识别所有 blocks
    // 返回融合后的块列表 (可能有多个)
    FinalBlockResults recognize(const yolo::BoxArray &dets) const;
};
