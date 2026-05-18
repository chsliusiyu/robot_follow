/**
 * @file local_occupancy_grid.hpp
 * @brief 局部占据栅格 — 基于动静分离区分人腿和静态障碍物
 *
 * 5cm 分辨率，40×40 覆盖 2m×2m 范围，每格 256 级计数。
 * 同一个世界坐标连续出现 STATIC_THRESH 帧后判定为静态障碍物。
 * 每次占据 +OCCUPY_INCREMENT，每帧衰减 -1。
 */

#ifndef LOCAL_OCCUPANCY_GRID_HPP
#define LOCAL_OCCUPANCY_GRID_HPP

#include <cmath>
#include <cstdint>
#include <algorithm>

class LocalOccupancyGrid {
public:
    static constexpr double RESOLUTION = 0.05;
    static constexpr int SIZE = 40;
    static constexpr uint8_t STATIC_THRESH = 3;
    static constexpr uint8_t OCCUPY_INCREMENT = 2;

    void setOrigin(double wx, double wy) {
        origin_x_ = wx - SIZE * RESOLUTION / 2.0;
        origin_y_ = wy - SIZE * RESOLUTION / 2.0;
    }

    // 所有已占格 -1，不再出现的点逐渐归零
    void decay() {
        for (int i = 0; i < SIZE; ++i)
            for (int j = 0; j < SIZE; ++j)
                if (cells_[i][j] > 0) cells_[i][j]--;
    }

    uint8_t getCell(double wx, double wy) const {
        int ix = static_cast<int>((wx - origin_x_) / RESOLUTION);
        int iy = static_cast<int>((wy - origin_y_) / RESOLUTION);
        if (ix < 0 || ix >= SIZE || iy < 0 || iy >= SIZE) return 0;
        return cells_[ix][iy];
    }

    void occupy(double wx, double wy) {
        int ix = static_cast<int>((wx - origin_x_) / RESOLUTION);
        int iy = static_cast<int>((wy - origin_y_) / RESOLUTION);
        if (ix < 0 || ix >= SIZE || iy < 0 || iy >= SIZE) return;
        cells_[ix][iy] = static_cast<uint8_t>(std::min(255, cells_[ix][iy] + OCCUPY_INCREMENT));
    }

private:
    uint8_t cells_[SIZE][SIZE] = {};
    double origin_x_ = 0, origin_y_ = 0;
};

#endif // LOCAL_OCCUPANCY_GRID_HPP
