#include "common/camera_params.hpp"
#include <iostream>

CameraParams CameraParams::LoadFromFile(const std::string &filepath)
{
    CameraParams params;
    try
    {
        YAML::Node node = YAML::LoadFile(filepath);

        params.width = node["camera"]["width"].as<int>();
        params.height = node["camera"]["height"].as<int>();
        params.color_width = node["camera"]["color_width"].as<int>(1920);   // 默认值1920
        params.color_height = node["camera"]["color_height"].as<int>(1088); // 默认值1088
        params.fps = node["camera"]["fps"].as<int>();

        std::string mode_str = node["camera"]["mode"].as<std::string>();
        params.mode = (mode_str == "infrared") ? CameraMode::INFRARED_ONLY : CameraMode::DEFAULT;

        // 解析 3x3 矩阵
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                params.rotation(i, j) = node["transform"]["rotation"][i][j].as<float>();
            }
        }

        params.translation << node["transform"]["translation"][0].as<float>(),
            node["transform"]["translation"][1].as<float>(),
            node["transform"]["translation"][2].as<float>();

        params.min_dist = node["params"]["min_dist"].as<float>();
        params.max_dist = node["params"]["max_dist"].as<float>();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Config] 使用默认参数，加载失败: " << e.what() << std::endl;
    }
    return params;
}
