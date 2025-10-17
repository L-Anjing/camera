#pragma once
#include <array>
#include <iostream>

class GridState
{
public:
    static constexpr int ROWS = 4;
    static constexpr int COLS = 3;
    int grid[ROWS][COLS];
    GridState();

    void clear();
    void updateWorldPosition(float Xw, float Yw, int label);
    void print() const;
    std::array<std::array<int, COLS>, ROWS> get() const;

private:
    std::array<std::array<int, COLS>, ROWS> grid_;
};
