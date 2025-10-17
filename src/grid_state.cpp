#include "grid_state.hpp"

GridState::GridState() { clear(); }

void GridState::clear()
{
    for (auto &r : grid_)
        r.fill(0);
}

// 更新世界坐标对应的格子
void GridState::updateWorldPosition(float Xw, float Yw, int label)
{
    // 定义固定的全局感知区域 (单位: m)
    const float WORLD_WIDTH = 3.0f;  // X方向长度（对应3列）
    const float WORLD_HEIGHT = 4.0f; // Y方向高度（对应4行）

    // 每个格子的大小
    float cell_w = WORLD_WIDTH / COLS;  // 1.0m
    float cell_h = WORLD_HEIGHT / ROWS; // 1.0m

    // 世界坐标 → 栅格索引（左上角是 row=0,col=0）
    int col = static_cast<int>(Xw / cell_w);
    int row = static_cast<int>(Yw / cell_h);

    if (row >= 0 && row < ROWS && col >= 0 && col < COLS)
    {
        if (grid_[row][col] == 0) // 只在未填充时更新
            grid_[row][col] = label;

        std::cout << "Mapped to grid: row=" << row
                  << ", col=" << col
                  << ", label=" << label << std::endl;
    }
    else
    {
        std::cout << " Warning: (" << Xw << "," << Yw << ") 超出3×4感知范围\n";
    }
}

void GridState::print() const
{
    std::cout << "Grid matrix:\n";
    for (const auto &r : grid_)
    {
        for (int val : r)
            std::cout << val << " ";
        std::cout << "\n";
    }
}

std::array<std::array<int, GridState::COLS>, GridState::ROWS> GridState::get() const
{
    return grid_;
}
