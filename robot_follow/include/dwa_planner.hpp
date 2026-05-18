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
#include <cstdio>
#include <string>

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

        // 前方走廊无障碍 → PD控制器对准目标直走
        if (isCorridorClear(obstacles, target_x, target_y, target_dist)) {
            double heading_error = std::atan2(target_y, target_x);
            // 角度差分归一化（防止±π跳变导致微分爆炸）
            double diff = heading_error - prev_heading_error_;
            if (diff > M_PI) diff -= 2.0 * M_PI;
            else if (diff < -M_PI) diff += 2.0 * M_PI;
            double heading_error_rate = diff / 0.1;
            prev_heading_error_ = heading_error;

            double cos_heading = std::cos(heading_error);
            double vx = DWA_MAX_VX * cos_heading;
            double dist_factor = std::clamp((target_dist - FOLLOW_DIST) / 0.5, 0.0, 1.0);
            vx = std::clamp(vx * dist_factor, window.min_vx, window.max_vx);

            // 死区：小角度不转，防止抖动
            if (std::abs(heading_error) < HEADING_DEADBAND) {
                return {vx, 0.0, 0.0, 1.0};
            }

            double wz = std::clamp(KP_HEADING * heading_error + KD_HEADING * heading_error_rate,
                                   window.min_wz, window.max_wz);
            return {vx, 0.0, wz, 1.0};
        }

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

        // 卡死恢复：最优轨迹被否决时，尝试横向移动找空隙
        if (std::abs(best.vx) < 0.05 && std::abs(best.vy) < 0.05 && std::abs(best.wz) < 0.05) {
            bool all_vetoed = (best.score < -1e100);

            if (all_vetoed) {
                fprintf(stderr, "\n[DWA] ALL SAMPLES VETOED — all %d trajectories blocked by EMERGENCY_DIST=%.2fm\n",
                        DWA_VX_SAMPLES * DWA_VY_SAMPLES * DWA_WZ_SAMPLES, DWA_EMERGENCY_DIST);
            } else {
                fprintf(stderr, "\n[DWA] STUCK detected — best sample near zero, printing score breakdown:\n");
                scoreSample(best.vx, best.vy, best.wz, obstacles,
                           target_x, target_y, num_steps, true);
            }

            double left_score = scoreSample(0.05,  0.25, 0.0, obstacles,
                                            target_x, target_y, num_steps,
                                            false, true);
            double right_score = scoreSample(0.05, -0.25, 0.0, obstacles,
                                             target_x, target_y, num_steps,
                                             false, true);

            bool left_ok  = (left_score > -1e100);
            bool right_ok = (right_score > -1e100);
            fprintf(stderr, "[DWA] Recovery: left=%s right=%s best=%s\n",
                    left_ok  ? fmtScore(left_score).c_str()  : "VETOED",
                    right_ok ? fmtScore(right_score).c_str() : "VETOED",
                    all_vetoed ? "VETOED" : fmtScore(best.score).c_str());

            bool recovery_selected = false;

            if (left_ok && left_score > best.score) {
                best = {0.05, 0.25, 0.0, left_score};
                recovery_selected = true;
                fprintf(stderr, "[DWA] -> Selected LEFT lateral recovery (vy=+0.25)\n");
            }
            if (right_ok && right_score > best.score) {
                best = {0.05, -0.25, 0.0, right_score};
                recovery_selected = true;
                fprintf(stderr, "[DWA] -> Selected RIGHT lateral recovery (vy=-0.25)\n");
            }
            if (!recovery_selected) {
                fprintf(stderr, "[DWA] -> Recovery FAILED, staying at zero\n");
            }

            // 全部否决且恢复也失败时才归零
            if (all_vetoed && !recovery_selected) {
                best = {0.0, 0.0, 0.0, 0.0};
            }
        }

        return best;
    }

private:
    static constexpr double CORRIDOR_HALF_WIDTH = 0.25;  // 前方走廊半宽 (m)
    static constexpr double CORRIDOR_MAX_LENGTH = 1.2;   // 前方走廊最大检查距离 (m)
    static constexpr double KP_HEADING = 1.0;             // 朝向P增益
    static constexpr double KD_HEADING = 0.3;             // 朝向D增益（阻尼振荡）
    static constexpr double HEADING_DEADBAND = 0.05;      // 朝向死区 (rad, ~3°)

    double prev_heading_error_ = 0.0;  // PD控制器状态

    // 检查目标方向矩形走廊内是否有障碍物
    bool isCorridorClear(const std::vector<std::pair<double, double>>& obstacles,
                         double target_x, double target_y, double target_dist) {
        for (const auto& obs : obstacles) {
            double ox = obs.first, oy = obs.second;
            double proj = (ox * target_x + oy * target_y) / target_dist;
            double max_proj = std::min(target_dist, CORRIDOR_MAX_LENGTH);
            if (proj < 0.0 || proj > max_proj) continue;
            double perp = std::abs(ox * target_y - oy * target_x) / target_dist;
            if (perp < CORRIDOR_HALF_WIDTH) return false;
        }
        return true;
    }

    static std::string fmtScore(double s) {
        if (s < -1e100) return "VETOED";
        char buf[32];
        snprintf(buf, sizeof(buf), "%.4f", s);
        return std::string(buf);
    }

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
                       double target_x, double target_y, int num_steps,
                       bool debug = false, bool recovery = false)
    {
        double target_angle = std::atan2(target_y, target_x);

        // 恢复轨迹需要趋势判断：记录起点 clearance，只否决持续恶化的轨迹
        double start_clearance = std::numeric_limits<double>::max();
        if (recovery) {
            for (const auto& obs : obstacles) {
                double d2 = obs.first * obs.first + obs.second * obs.second;
                double dist = std::sqrt(d2);
                if (dist < start_clearance) start_clearance = dist;
            }
        }

        // 前向仿真轨迹，同时计算 clearance 和障碍密度
        double x = 0.0, y = 0.0, theta = 0.0;
        double min_clearance = std::numeric_limits<double>::max();
        double density_penalty = 0.0;
        double heading_sum = 0.0;

        for (int k = 0; k < num_steps; ++k) {
            // 欧拉积分
            x += (vx * std::cos(theta) - vy * std::sin(theta)) * DWA_DT;
            y += (vx * std::sin(theta) + vy * std::cos(theta)) * DWA_DT;
            theta += wz * DWA_DT;

            // 每步朝向评分累积（避免"承诺15步后对齐但只走1步"作弊）
            double step_tx = (target_x - x) * std::cos(theta) + (target_y - y) * std::sin(theta);
            double step_ty = -(target_x - x) * std::sin(theta) + (target_y - y) * std::cos(theta);
            double step_angle = std::abs(std::atan2(step_ty, step_tx));
            double step_cos = std::cos(step_angle);
            heading_sum += step_cos * step_cos;

            double step_min_dist = std::numeric_limits<double>::max();
            for (const auto& obs : obstacles) {
                double dx = x - obs.first;
                double dy = y - obs.second;
                double d2 = dx * dx + dy * dy;
                double dist = std::sqrt(d2);
                if (dist < step_min_dist) step_min_dist = dist;
                if (dist < DWA_SAFE_DIST) {
                    density_penalty += std::exp(-d2 / (DWA_SAFE_DIST * DWA_SAFE_DIST));
                }
            }
            if (step_min_dist < min_clearance) min_clearance = step_min_dist;

            // 硬否决：碰撞（零速不否决，不动不会撞）
            bool zero_vel = (std::abs(vx) < 1e-6 && std::abs(vy) < 1e-6 && std::abs(wz) < 1e-6);
            if (!zero_vel && min_clearance < DWA_EMERGENCY_DIST) {
                // 恢复轨迹使用趋势否决：clearance 比起点恶化超过 3cm 才否决
                if (recovery && min_clearance >= start_clearance - 0.03) {
                    continue; // 保持距离或远离中，不否决
                }
                if (debug) {
                    fprintf(stderr, "  [DWA] step %d: VETO collision min_clearance=%.3f < %.3f\n",
                            k, min_clearance, DWA_EMERGENCY_DIST);
                }
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
            if (debug) {
                fprintf(stderr, "  [DWA] VETO pred_target_dist=%.3f > %.3f\n",
                        pred_target_dist, DWA_MAX_TARGET_RANGE);
            }
            return -std::numeric_limits<double>::max();
        }

        // 评分 1: 朝向 —— 全程15步cos²平均，杜绝"承诺对齐"作弊
        double heading_score = heading_sum / num_steps;

        // 评分 2: 安全距离 —— 轨迹上最近障碍距离（非线性指数衰减，近距离惩罚剧烈）
        double clearance_score = 1.0 - std::exp(-3.0 * min_clearance / DWA_SAFE_DIST);

        // 障碍密度罚分：轨迹周围障碍物越多扣分越多
        double density_score = 1.0 / (1.0 + density_penalty);

        // 评分 3: 速度 —— 沿目标方向的速度投影，带距离衰减
        double target_dist = std::sqrt(target_x * target_x + target_y * target_y);
        double dist_factor = std::clamp(
            (target_dist - FOLLOW_DIST) / (DWA_MAX_TARGET_RANGE - FOLLOW_DIST) * 0.5 + 0.15,
            0.15, 1.0);
        double vel_proj = vx * std::cos(target_angle) + vy * std::sin(target_angle);
        double heading_alignment = std::abs(std::cos(target_angle));
        double velocity_score = vel_proj / DWA_MAX_VX * dist_factor * heading_alignment;

        // 评分 4: 目标距离 —— 终点与理想跟随距离的偏差
        double dist_error = std::abs(pred_target_dist - FOLLOW_DIST);
        double target_dist_score = std::max(0.0, 1.0 - dist_error / FOLLOW_DIST);

        double total = DWA_WEIGHT_HEADING   * heading_score +
                       DWA_WEIGHT_CLEARANCE * (0.7 * clearance_score + 0.3 * density_score) +
                       DWA_WEIGHT_VELOCITY  * velocity_score +
                       DWA_WEIGHT_TARGET_DIST * target_dist_score;

        if (debug) {
            double w_heading   = DWA_WEIGHT_HEADING * heading_score;
            double w_clearance = DWA_WEIGHT_CLEARANCE * 0.7 * clearance_score;
            double w_density   = DWA_WEIGHT_CLEARANCE * 0.3 * density_score;
            double w_velocity  = DWA_WEIGHT_VELOCITY * velocity_score;
            double w_target    = DWA_WEIGHT_TARGET_DIST * target_dist_score;

            fprintf(stderr, "\n========== DWA Stuck Debug ==========\n");
            fprintf(stderr, "Target: dist=%.3fm angle=%.1f° | Obstacles: %zu\n",
                    target_dist, target_angle * 180.0 / M_PI, obstacles.size());
            fprintf(stderr, "Sample: vx=%.3f vy=%.3f wz=%.3f\n", vx, vy, wz);
            fprintf(stderr, "Trajectory: min_clearance=%.3fm density_penalty=%.3f pred_target_dist=%.3fm\n",
                    min_clearance, density_penalty, pred_target_dist);
            fprintf(stderr, "--- Score breakdown (raw -> weighted) ---\n");
            fprintf(stderr, "heading:      %6.3f -> %6.3f  (w=%.2f)\n",
                    heading_score, w_heading, DWA_WEIGHT_HEADING);
            fprintf(stderr, "clearance:    %6.3f -> %6.3f  (w=%.2f*0.7)  min_clr=%.3f\n",
                    clearance_score, w_clearance, DWA_WEIGHT_CLEARANCE, min_clearance);
            fprintf(stderr, "density:      %6.3f -> %6.3f  (w=%.2f*0.3)  penalty=%.3f\n",
                    density_score, w_density, DWA_WEIGHT_CLEARANCE, density_penalty);
            fprintf(stderr, "velocity:     %6.3f -> %6.3f  (w=%.2f)  vel_proj=%.3f dist_factor=%.3f\n",
                    velocity_score, w_velocity, DWA_WEIGHT_VELOCITY, vel_proj, dist_factor);
            fprintf(stderr, "target_dist:  %6.3f -> %6.3f  (w=%.2f)  dist_error=%.3f\n",
                    target_dist_score, w_target, DWA_WEIGHT_TARGET_DIST, dist_error);
            fprintf(stderr, "TOTAL SCORE: %.4f\n", total);
            fprintf(stderr, "======================================\n\n");
        }

        return total;
    }
};

#endif // DWA_PLANNER_HPP
