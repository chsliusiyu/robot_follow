/**
 * @file nmpc_planner.hpp
 * @brief NMPC (Nonlinear Model Predictive Control) 局部避障规划器
 *
 * 采用梯度下降 + 伴随法反向传播求解非线性 MPC 问题。
 * 预测 1.5s 内的轨迹，通过终端跟踪代价 + 阶段障碍代价 + 控制平滑代价
 * 优化控制序列，输出第一步的速度指令。
 */

#ifndef NMPC_PLANNER_HPP
#define NMPC_PLANNER_HPP

#include "common_types.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <array>

class NMPCPlanner {
public:
    struct Control {
        double vx, vy, wz;
    };

    void resetWarmstart() { has_warmstart_ = false; }

    /**
     * @brief 求解 NMPC，返回避障后的最优第一步控制
     * @param obstacles  障碍点云（机器人坐标系，已过滤自身框架和遮罩）
     * @param cur_vx     当前速度 Vx
     * @param cur_vy     当前速度 Vy
     * @param cur_wz     当前速度 Wz
     * @param des_vx     P 跟随层期望 Vx
     * @param des_vy     P 跟随层期望 Vy
     * @param des_wz     P 跟随层期望 Wz
     * @return 避障后的最优速度指令 [vx, vy, wz]
     */
    Control solve(const std::vector<std::pair<double, double>>& obstacles,
                  double cur_vx, double cur_vy, double cur_wz,
                  double des_vx, double des_vy, double des_wz)
    {
        int N = NMPC_HORIZON;
        double dt = NMPC_DT;

        // 1. Warm start 初始化
        std::vector<Control> U(N);
        if (has_warmstart_ && prev_U_.size() == static_cast<size_t>(N)) {
            for (int k = 0; k < N - 1; ++k) U[k] = prev_U_[k + 1];
            U[N - 1] = prev_U_[N - 1];
        } else {
            for (int k = 0; k < N; ++k) U[k] = {cur_vx, cur_vy, cur_wz};
        }

        // 2. 障碍物过滤：只保留影响范围内的点
        std::vector<std::pair<double, double>> near_obs;
        near_obs.reserve(obstacles.size());
        for (const auto& o : obstacles) {
            double d = std::sqrt(o.first * o.first + o.second * o.second);
            if (d < NMPC_OBS_CUTOFF * 2.0) near_obs.push_back(o);
        }

        // 3. 梯度下降优化
        std::vector<Control> best_U = U;
        double best_cost = std::numeric_limits<double>::max();
        double step = NMPC_INIT_STEP;

        for (int iter = 0; iter < NMPC_MAX_ITER; ++iter) {
            // 3a. 前向仿真 + 代价计算
            std::vector<double> xs(N + 1), ys(N + 1), thetas(N + 1);
            xs[0] = 0.0; ys[0] = 0.0; thetas[0] = 0.0;
            double cost = 0.0;

            // 紧急距离内障碍的最近距离
            double min_emergency = std::numeric_limits<double>::max();

            for (int k = 0; k < N; ++k) {
                double c = std::cos(thetas[k]);
                double s = std::sin(thetas[k]);
                xs[k + 1]     = xs[k] + (U[k].vx * c - U[k].vy * s) * dt;
                ys[k + 1]     = ys[k] + (U[k].vx * s + U[k].vy * c) * dt;
                thetas[k + 1] = thetas[k] + U[k].wz * dt;

                // 障碍代价 + 紧急距离检测
                double obs_cost = 0.0;
                for (const auto& o : near_obs) {
                    double dx = xs[k + 1] - o.first;
                    double dy = ys[k + 1] - o.second;
                    double d2 = dx * dx + dy * dy;
                    double d = std::sqrt(d2);
                    if (d < min_emergency) min_emergency = d;
                    if (d < NMPC_OBS_CUTOFF) {
                        obs_cost += std::exp(-d2 / (NMPC_OBS_SIGMA * NMPC_OBS_SIGMA));
                    }
                }
                cost += NMPC_W_OBS * obs_cost;

                // 控制平滑代价
                double dvx = U[k].vx - (k == 0 ? cur_vx : U[k - 1].vx);
                double dvy = U[k].vy - (k == 0 ? cur_vy : U[k - 1].vy);
                double dwz = U[k].wz - (k == 0 ? cur_wz : U[k - 1].wz);
                cost += NMPC_W_SMOOTH * (dvx * dvx + dvy * dvy + dwz * dwz);

                // 控制代价
                cost += NMPC_W_CTRL * (U[k].vx * U[k].vx + U[k].vy * U[k].vy + U[k].wz * U[k].wz);

                // 偏离代价：第一步尽量接近 P 控制输出
                if (k == 0) {
                    double dvx0 = U[0].vx - des_vx;
                    double dvy0 = U[0].vy - des_vy;
                    double dwz0 = U[0].wz - des_wz;
                    cost += NMPC_W_DEVIATION * (dvx0 * dvx0 + dvy0 * dvy0 + dwz0 * dwz0);
                }
            }

            // 紧急否决
            if (min_emergency < NMPC_EMERGENCY_DIST) {
                cost = std::numeric_limits<double>::max();
            }

            if (cost < best_cost && cost < 1e9) {
                best_cost = cost;
                best_U = U;
            }

            // 3b. 伴随法反向传播计算梯度（无终端代价，λ_N = 0）
            double lx = 0.0, ly = 0.0, lt = 0.0;

            // 反向传播
            for (int k = N - 1; k >= 0; --k) {
                double ck = std::cos(thetas[k]);
                double sk = std::sin(thetas[k]);

                // 障碍代价对状态梯度 ∂L_obs/∂z_k
                double dL_dx = 0.0, dL_dy = 0.0;
                for (const auto& o : near_obs) {
                    double dx = xs[k + 1] - o.first;
                    double dy = ys[k + 1] - o.second;
                    double d2 = dx * dx + dy * dy;
                    if (d2 < NMPC_OBS_CUTOFF * NMPC_OBS_CUTOFF) {
                        double sigma2 = NMPC_OBS_SIGMA * NMPC_OBS_SIGMA;
                        double w = std::exp(-d2 / sigma2);
                        dL_dx += (-2.0 * NMPC_W_OBS / sigma2) * dx * w;
                        dL_dy += (-2.0 * NMPC_W_OBS / sigma2) * dy * w;
                    }
                }

                // 伴随更新: λ_k = ∂L/∂z_k + (∂f/∂z_k)^T * λ_{k+1}
                double dzh_dx = (-U[k].vx * sk - U[k].vy * ck) * dt;
                double dzh_dy = ( U[k].vx * ck - U[k].vy * sk) * dt;
                lx = dL_dx + lx;                        // (∂f/∂z)^T[0,:]·λ = [1,0,0]·λ = λ[0]
                ly = dL_dy + ly;                        // (∂f/∂z)^T[1,:]·λ = [0,1,0]·λ = λ[1]
                lt = 0.0 + dzh_dx * lx + dzh_dy * ly + lt;  // (∂f/∂z)^T[2,:]·λ + λ[2]

                // 控制梯度: g_k = (∂f/∂u_k)^T * λ
                double g_vx = ck * dt * lx + sk * dt * ly;
                double g_vy = -sk * dt * lx + ck * dt * ly;
                double g_wz = dt * lt;

                // 平滑代价梯度
                if (k == 0) {
                    g_vx += 2.0 * NMPC_W_SMOOTH * (U[k].vx - cur_vx);
                    g_vy += 2.0 * NMPC_W_SMOOTH * (U[k].vy - cur_vy);
                    g_wz += 2.0 * NMPC_W_SMOOTH * (U[k].wz - cur_wz);
                } else {
                    g_vx += 2.0 * NMPC_W_SMOOTH * (U[k].vx - U[k - 1].vx);
                    g_vy += 2.0 * NMPC_W_SMOOTH * (U[k].vy - U[k - 1].vy);
                    g_wz += 2.0 * NMPC_W_SMOOTH * (U[k].wz - U[k - 1].wz);
                }
                if (k < N - 1) {
                    g_vx -= 2.0 * NMPC_W_SMOOTH * (U[k + 1].vx - U[k].vx);
                    g_vy -= 2.0 * NMPC_W_SMOOTH * (U[k + 1].vy - U[k].vy);
                    g_wz -= 2.0 * NMPC_W_SMOOTH * (U[k + 1].wz - U[k].wz);
                }

                // 偏离代价梯度（仅第一步）
                if (k == 0) {
                    g_vx += 2.0 * NMPC_W_DEVIATION * (U[0].vx - des_vx);
                    g_vy += 2.0 * NMPC_W_DEVIATION * (U[0].vy - des_vy);
                    g_wz += 2.0 * NMPC_W_DEVIATION * (U[0].wz - des_wz);
                }

                // 控制代价梯度
                g_vx += 2.0 * NMPC_W_CTRL * U[k].vx;
                g_vy += 2.0 * NMPC_W_CTRL * U[k].vy;
                g_wz += 2.0 * NMPC_W_CTRL * U[k].wz;

                // 存储更新后的 U
                double new_vx = U[k].vx - step * g_vx;
                double new_vy = U[k].vy - step * g_vy;
                double new_wz = U[k].wz - step * g_wz;
                U[k] = {new_vx, new_vy, new_wz};
            }

            // 3c. 投影到可行域
            projectBox(U);
            projectAccel(U, cur_vx, cur_vy, cur_wz, dt);

            // 3d. 收敛检查
            double grad_norm = 0.0;
            for (int k = 0; k < N; ++k) {
                grad_norm += U[k].vx * U[k].vx + U[k].vy * U[k].vy + U[k].wz * U[k].wz;
            }
            if (grad_norm < NMPC_CONVERGE_TOL * NMPC_CONVERGE_TOL) break;

            // 3e. 步长自适应
            step = std::max(NMPC_MIN_STEP, step * 0.98);
        }

        // 4. 保存 warm start
        prev_U_ = best_U;
        has_warmstart_ = true;

        return best_U[0];
    }

private:
    std::vector<Control> prev_U_;
    bool has_warmstart_ = false;

    void projectBox(std::vector<Control>& U) {
        for (auto& u : U) {
            u.vx = std::clamp(u.vx, NMPC_MIN_VX, NMPC_MAX_VX);
            u.vy = std::clamp(u.vy, NMPC_MIN_VY, NMPC_MAX_VY);
            u.wz = std::clamp(u.wz, NMPC_MIN_WZ, NMPC_MAX_WZ);
        }
    }

    void projectAccel(std::vector<Control>& U,
                      double cvx, double cvy, double cwz, double dt) {
        double ax = NMPC_ACC_VX * dt;
        double ay = NMPC_ACC_VY * dt;
        double az = NMPC_ACC_WZ * dt;

        U[0].vx = std::clamp(U[0].vx, cvx - ax, cvx + ax);
        U[0].vy = std::clamp(U[0].vy, cvy - ay, cvy + ay);
        U[0].wz = std::clamp(U[0].wz, cwz - az, cwz + az);

        for (size_t k = 1; k < U.size(); ++k) {
            U[k].vx = std::clamp(U[k].vx, U[k - 1].vx - ax, U[k - 1].vx + ax);
            U[k].vy = std::clamp(U[k].vy, U[k - 1].vy - ay, U[k - 1].vy + ay);
            U[k].wz = std::clamp(U[k].wz, U[k - 1].wz - az, U[k - 1].wz + az);
        }
    }
};

#endif // NMPC_PLANNER_HPP
