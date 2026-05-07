/**
 * @file dwa_planner.hpp
 * @brief DWA (Dynamic Window Approach) 局部避障规划器
 *
 * 在速度空间中采样并前向仿真轨迹，通过多目标评分选出最优速度指令。
 * 适用于全向移动机器人（vx, vy, wz 三自由度）。
 */

#ifndef DWA_PLANNER_HPP
#define DWA_PLANNER_HPP

#include "common_types.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

class DWAPlanner {
public:
    struct Sample {
        double vx, vy, wz;
        double score;
    };

    /**
     * @brief DWA 主入口：根据障碍物、目标位置和当前速度规划最优速度指令
     * @param obstacles  障碍点云（机器人坐标系，已过滤自身框架）
     * @param target_x   目标 X 坐标（机器人坐标系）
     * @param target_y   目标 Y 坐标（机器人坐标系）
     * @param cur_vx     当前机器人 Vx (m/s)
     * @param cur_vy     当前机器人 Vy (m/s)
     * @param cur_wz     当前机器人 Wz (rad/s)
     * @return 最优采样 (vx, vy, wz, score)
     */
    Sample plan(const std::vector<std::pair<double, double>>& obstacles,
                double target_x, double target_y,
                double cur_vx, double cur_vy, double cur_wz)
    {
        double target_dist = std::sqrt(target_x * target_x + target_y * target_y);

        // 无有效目标时返回零速
        if (target_dist < 0.01 || target_dist > DWA_MAX_TARGET_RANGE) {
            return {0.0, 0.0, 0.0, 0.0};
        }

        VelocityWindow window = computeWindow(cur_vx, cur_vy, cur_wz);
        int num_steps = static_cast<int>(DWA_SIM_TIME / DWA_DT);

        Sample best{0.0, 0.0, 0.0, -std::numeric_limits<double>::max()};

        double dvx = (window.max_vx - window.min_vx) / std::max(1, DWA_VX_SAMPLES - 1);
        double dvy = (window.max_vy - window.min_vy) / std::max(1, DWA_VY_SAMPLES - 1);
        double dwz = (window.max_wz - window.min_wz) / std::max(1, DWA_WZ_SAMPLES - 1);

        for (int ivx = 0; ivx < DWA_VX_SAMPLES; ++ivx) {
            double vx = (DWA_VX_SAMPLES == 1) ? window.min_vx : window.min_vx + ivx * dvx;
            for (int ivy = 0; ivy < DWA_VY_SAMPLES; ++ivy) {
                double vy = (DWA_VY_SAMPLES == 1) ? window.min_vy : window.min_vy + ivy * dvy;
                for (int iwz = 0; iwz < DWA_WZ_SAMPLES; ++iwz) {
                    double wz = (DWA_WZ_SAMPLES == 1) ? window.min_wz : window.min_wz + iwz * dwz;

                    double score = scoreSample(vx, vy, wz, obstacles,
                                               target_x, target_y, num_steps);
                    if (score > best.score) {
                        best = {vx, vy, wz, score};
                    }
                }
            }
        }

        return best;
    }

private:
    struct VelocityWindow {
        double min_vx, max_vx;
        double min_vy, max_vy;
        double min_wz, max_wz;
    };

    VelocityWindow computeWindow(double cvx, double cvy, double cwz) {
        VelocityWindow w;
        double dt = 0.1;  // 与控制周期一致
        w.min_vx = std::max(DWA_MIN_VX, cvx - DWA_ACC_VX * dt);
        w.max_vx = std::min(DWA_MAX_VX, cvx + DWA_ACC_VX * dt);
        w.min_vy = std::max(DWA_MIN_VY, cvy - DWA_ACC_VY * dt);
        w.max_vy = std::min(DWA_MAX_VY, cvy + DWA_ACC_VY * dt);
        w.min_wz = std::max(DWA_MIN_WZ, cwz - DWA_ACC_WZ * dt);
        w.max_wz = std::min(DWA_MAX_WZ, cwz + DWA_ACC_WZ * dt);
        return w;
    }

    double scoreSample(double vx, double vy, double wz,
                       const std::vector<std::pair<double, double>>& obstacles,
                       double target_x, double target_y, int num_steps)
    {
        double target_angle = std::atan2(target_y, target_x);

        // 前向仿真轨迹，同时计算 clearance
        double x = 0.0, y = 0.0, theta = 0.0;
        double min_clearance = std::numeric_limits<double>::max();

        for (int k = 0; k < num_steps; ++k) {
            // 欧拉积分
            x += (vx * std::cos(theta) - vy * std::sin(theta)) * DWA_DT;
            y += (vx * std::sin(theta) + vy * std::cos(theta)) * DWA_DT;
            theta += wz * DWA_DT;

            // 计算当前位置到所有障碍的最近距离
            double step_min_dist = std::numeric_limits<double>::max();
            for (const auto& obs : obstacles) {
                double dx = x - obs.first;
                double dy = y - obs.second;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < step_min_dist) step_min_dist = dist;
            }
            if (step_min_dist < min_clearance) min_clearance = step_min_dist;

            // 硬否决：碰撞
            if (min_clearance < DWA_EMERGENCY_DIST) {
                return -std::numeric_limits<double>::max();
            }
        }

        // 计算轨迹终点处目标在机器人坐标系中的位置
        double pred_target_x = (target_x - x) * std::cos(theta) + (target_y - y) * std::sin(theta);
        double pred_target_y = -(target_x - x) * std::sin(theta) + (target_y - y) * std::cos(theta);
        double pred_target_dist = std::sqrt(pred_target_x * pred_target_x +
                                            pred_target_y * pred_target_y);

        // 硬否决：目标太远
        if (pred_target_dist > DWA_MAX_TARGET_RANGE) {
            return -std::numeric_limits<double>::max();
        }

        // 评分 1: 朝向 —— 终点处目标偏角越小越好
        double angle_error = std::abs(std::atan2(pred_target_y, pred_target_x));
        double heading_score = 1.0 - angle_error / M_PI;

        // 评分 2: 安全距离 —— 轨迹上最近障碍距离
        double clearance_score = std::min(1.0, min_clearance / DWA_SAFE_DIST);

        // 评分 3: 速度 —— 沿目标方向的速度投影
        double vel_proj = vx * std::cos(target_angle) + vy * std::sin(target_angle);
        double velocity_score = std::max(0.0, vel_proj) / DWA_MAX_VX;

        // 评分 4: 目标距离 —— 终点与理想跟随距离的偏差
        double dist_error = std::abs(pred_target_dist - FOLLOW_DIST);
        double target_dist_score = std::max(0.0, 1.0 - dist_error / FOLLOW_DIST);

        return DWA_WEIGHT_HEADING   * heading_score +
               DWA_WEIGHT_CLEARANCE * clearance_score +
               DWA_WEIGHT_VELOCITY  * velocity_score +
               DWA_WEIGHT_TARGET_DIST * target_dist_score;
    }
};

#endif // DWA_PLANNER_HPP
