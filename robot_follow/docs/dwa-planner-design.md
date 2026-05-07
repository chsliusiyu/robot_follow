# DWA (Dynamic Window Approach) 局部避障规划器设计文档

---

## 文件控制

| 项目 | 内容 |
|------|------|
| 文档标题 | DWA 局部避障规划器设计文档 |
| 版本号 | V1.0 |
| 作者 | chs |
| 创建日期 | 2026-05-07 |
| 审核状态 | 已评审 |
| 关联分支 | `feature/dwa-planner` |

---

## 目录

1. [引言](#1-引言)
2. [术语与缩略语](#2-术语与缩略语)
3. [需求分析](#3-需求分析)
4. [系统架构](#4-系统架构)
5. [算法设计](#5-算法设计)
6. [参数设计](#6-参数设计)
7. [详细设计](#7-详细设计)
8. [状态与数据流](#8-状态与数据流)
9. [接口定义](#9-接口定义)
10. [验证方案](#10-验证方案)
11. [与 APF 方案对比](#11-与-apf-方案对比)
12. [约束与局限](#12-约束与局限)

附录 [A. 修改文件清单](#a-修改文件清单) [B. 参考文献](#b-参考文献)

---

## 1. 引言

### 1.1 背景

本项目是四足机器狗目标跟随系统。原版使用人工势场法（APF）进行避障：在位置空间计算排斥力矢量和，直接叠加到速度指令上。虽然实现简单，但存在根本性局限——缺乏对机器人运动学和未来状态的预测，在面对复杂障碍时行为不够智能。

### 1.2 问题陈述

| 编号 | 问题 | APF 表现 |
|------|------|---------|
| P-01 | 无轨迹预测 | 仅基于当前距离计算力，不预判未来碰撞 |
| P-02 | 无动力学约束 | 输出的速度可能瞬时跳变，不考虑加速度限制 |
| P-03 | 单一避障策略 | 只有"推开"一种行为，无法权衡速度、朝向、安全多目标 |
| P-04 | 局部极小值 | 对称障碍场中斥力与引力抵消，机器人卡住 |

### 1.3 设计目标

用 DWA 完全替换 APF，实现基于速度空间搜索的局部避障规划器：

- 前向仿真轨迹并预测碰撞，提前规避
- 遵守加速度/速度硬约束，输出物理可达的速度指令
- 多目标评分（朝向、安全、速度、目标距离），权衡选择最优方案
- 与现有跟随系统无缝集成，不改变外部接口

### 1.4 设计范围

| 涉及模块 | 是否修改 | 说明 |
|----------|---------|------|
| `common_types.hpp` | 是 | 删除 APF 常量，新增 DWA 参数 |
| `dwa_planner.hpp` | **新建** | DWA 核心算法（~150 行） |
| `lidar_tracker.hpp` | 是 | 删除 `calculateFollowVelocity()`，接入 DWA |
| `robot_nexus.cpp` | 否 | 无接口变化 |
| `direct_control.hpp` | 否 | 不经过 DWA 管线 |
| `kalman_filter.hpp` | 否 | 无关 |

---

## 2. 术语与缩略语

| 术语 | 全称 | 说明 |
|------|------|------|
| DWA | Dynamic Window Approach | 在速度空间中搜索最优可行速度指令的局部规划算法 |
| Dynamic Window | — | 考虑加速度约束后当前可达到的速度范围 |
| Trajectory Rollout | — | 对候选速度进行前向仿真，生成轨迹序列 |
| Clearance | — | 轨迹上所有点到最近障碍物的距离（安全余量） |
| Hard Veto | — | 硬否决：轨迹碰撞障碍直接排除，score = -∞ |
| Multi-Objective Scoring | — | 综合朝向、安全、速度、目标距离四个维度的加权评分 |

---

## 3. 需求分析

### 3.1 功能需求

| 编号 | 需求描述 | 优先级 |
|------|---------|--------|
| F-01 | 根据当前速度和加速度约束计算可达速度窗口 | P0 |
| F-02 | 在速度窗口中采样 (vx, vy, wz) 组合 | P0 |
| F-03 | 对每个采样前向仿真轨迹并检查障碍物碰撞 | P0 |
| F-04 | 综合朝向、安全距离、速度、目标距离四个维度评分 | P0 |
| F-05 | 碰撞轨迹硬否决（score = -∞） | P0 |
| F-06 | 输出最优速度指令 (vx, vy, wz) | P0 |
| F-07 | 目标丢失或过远时输出零速 | P1 |

### 3.2 非功能需求

| 编号 | 需求描述 |
|------|---------|
| NF-01 | 每帧规划耗时 < 20ms（保证 10Hz 实时性） |
| NF-02 | 速度输出平滑，无跳变（加速度约束生效） |
| NF-03 | MODE_DIRECT 模式不受影响 |
| NF-04 | 无障碍时行为等价于纯跟随控制 |

---

## 4. 系统架构

### 4.1 数据流

```
LiDAR (/scan)
    │
    ▼
┌────────────────────────────────────────────────────┐
│              LidarTracker::processScan()             │
│                                                     │
│  ┌─────────────────────────────────────────────┐   │
│  │  1. 读取目标位置                              │   │
│  │  2. 遍历点云 → 过滤自身框架 → 收集障碍点       │   │
│  │  3. state_.setPoints(障碍点)                  │   │
│  └─────────────────────────────────────────────┘   │
│                       │                             │
│                       ▼                             │
│  ┌─────────────────────────────────────────────┐   │
│  │         DWAPlanner::plan()                    │   │
│  │                                              │   │
│  │  ┌──────────────────────────────────────┐    │   │
│  │  │  动态窗口计算                         │    │   │
│  │  │  vx∈[cur-acc*dt, cur+acc*dt]         │    │   │
│  │  │  vy∈[cur-acc*dt, cur+acc*dt]         │    │   │
│  │  │  wz∈[cur-acc*dt, cur+acc*dt]         │    │   │
│  │  └──────────────────────────────────────┘    │   │
│  │                    │                         │   │
│  │                    ▼                         │   │
│  │  ┌──────────────────────────────────────┐    │   │
│  │  │  速度采样 15×5×7=525 组               │    │   │
│  │  └──────────────────────────────────────┘    │   │
│  │                    │                         │   │
│  │                    ▼                         │   │
│  │  ┌──────────────────────────────────────┐    │   │
│  │  │  对每组采样:                           │    │   │
│  │  │  a. 前向仿真 1.5s 轨迹 (15 步)         │    │   │
│  │  │  b. 碰撞检测 (硬否决)                  │    │   │
│  │  │  c. 四维评分                           │    │   │
│  │  │  d. 保留最高分                         │    │   │
│  │  └──────────────────────────────────────┘    │   │
│  │                    │                         │   │
│  │                    ▼                         │   │
│  │        return 最优 (vx, vy, wz)              │   │
│  └─────────────────────────────────────────────┘   │
│                       │                             │
│                       ▼                             │
│             cmd_vel → setVelocity()                  │
│             cmd_vel → publishVelocity()              │
│                                                     │
└────────────────────────────────────────────────────┘
    │
    ▼
/cmd_vel → HighLevel::move(vx, vy, yaw_rate)
```

### 4.2 模块依赖

```
         common_types.hpp (常量)
              │
    ┌─────────┼─────────┐
    ▼         ▼         ▼
dwa_planner  lidar_   direct_
  .hpp       tracker   control
    │         .hpp      .hpp
    └─────────┼─────────┘
              ▼
       robot_nexus.cpp
```

---

## 5. 算法设计

### 5.1 动态窗口计算

```python
def computeWindow(cur_vx, cur_vy, cur_wz):
    dt = 0.1  # 控制周期

    wx = [max(DWA_MIN_VX, cur_vx - DWA_ACC_VX * dt),
          min(DWA_MAX_VX, cur_vx + DWA_ACC_VX * dt)]
    wy = [max(DWA_MIN_VY, cur_vy - DWA_ACC_VY * dt),
          min(DWA_MAX_VY, cur_vy + DWA_ACC_VY * dt)]
    wz = [max(DWA_MIN_WZ, cur_wz - DWA_ACC_WZ * dt),
          min(DWA_MAX_WZ, cur_wz + DWA_ACC_WZ * dt)]

    return VelocityWindow(wx, wy, wz)
```

加速度约束保证速度不会瞬时跳变。以 vx 为例：当前 0.3 m/s，下帧最大仅可达 0.3 + 1.0×0.1 = 0.4 m/s。

### 5.2 前向轨迹仿真

采用欧拉积分，运动学模型：

```
x_{k+1} = x_k + (vx·cos(θ_k) - vy·sin(θ_k))·dt
y_{k+1} = y_k + (vx·sin(θ_k) + vy·cos(θ_k))·dt
θ_{k+1} = θ_k + wz·dt
```

仿真参数：sim_time = 1.5s, dt = 0.1s, 共 15 步。

每步同时计算到所有障碍物的最近距离，用于碰撞检测和 clearance 评分。

### 5.3 轨迹终点目标预测

轨迹终点机器人在原始坐标系中的位姿为 (x_end, y_end, θ_end)。目标在原始坐标系中的位置为 (target_x, target_y)。目标在终点机器人坐标系中的位置：

```
pred_target_x = (target_x - x_end)·cos(θ_end) + (target_y - y_end)·sin(θ_end)
pred_target_y = -(target_x - x_end)·sin(θ_end) + (target_y - y_end)·cos(θ_end)
```

### 5.4 评分函数

```
score = 0.35·heading + 0.35·clearance + 0.15·velocity + 0.15·target_dist
```

**heading**（朝向得分）：

```
angle_error = |atan2(pred_target_y, pred_target_x)|
heading_score = 1.0 - angle_error / π     # 范围 [0, 1]
```

轨迹终点机器人朝向与目标方向的偏差越小，得分越高。1.0 = 正对目标，0.0 = 背对目标。

**clearance**（安全距离得分）：

```
min_dist = min(所有轨迹点的最近障碍距离)
if min_dist < DWA_EMERGENCY_DIST: return -∞  # 硬否决
clearance_score = clamp(min_dist / DWA_SAFE_DIST, 0, 1)
```

0.0 = 紧贴障碍，1.0 = 安全距离外（≥0.4m）。

**velocity**（速度得分）：

```
vel_proj = vx·cos(target_angle) + vy·sin(target_angle)  # 沿目标方向的速度投影
velocity_score = max(0, vel_proj) / DWA_MAX_VX
```

鼓励沿目标方向快速接近。

**target_dist**（目标距离得分）：

```
pred_dist = sqrt(pred_target_x² + pred_target_y²)  # 轨迹终点到目标距离
error = |pred_dist - FOLLOW_DIST|
target_dist_score = max(0, 1.0 - error / FOLLOW_DIST)
```

鼓励轨迹终点与目标保持理想跟随距离 0.6m。

### 5.5 硬否决条件

| 条件 | 效果 |
|------|------|
| 轨迹上任何点到障碍距离 < 0.15m | score = -∞，此轨迹永不被选中 |
| 轨迹终点目标距离 > 3.0m | score = -∞，目标可能丢失 |

### 5.6 复杂度分析

- 采样数：15 × 5 × 7 = 525 组/帧
- 每组轨迹 15 步，每步检查 N 个障碍点
- 总距离计算：525 × 15 × N ≈ 7875N
- 典型 N = 200 点 → ~1.6M 次距离计算
- 现代 CPU 上 ≈ 1-3ms，满足 10Hz 实时要求

---

## 6. 参数设计

### 6.1 核心参数

| 参数 | 值 | 单位 | 说明 |
|------|------|------|------|
| `DWA_SIM_TIME` | 1.5 | s | 前向仿真时长 |
| `DWA_DT` | 0.1 | s | 仿真步长 |
| `DWA_VX_SAMPLES` | 15 | — | vx 采样数 |
| `DWA_VY_SAMPLES` | 5 | — | vy 采样数 |
| `DWA_WZ_SAMPLES` | 7 | — | wz 采样数 |

### 6.2 速度约束

| 参数 | 值 | 单位 | 说明 |
|------|------|------|------|
| `DWA_MIN_VX` | -0.3 | m/s | vx 下限（允许低速后退） |
| `DWA_MAX_VX` | 1.0 | m/s | vx 上限 |
| `DWA_MIN_VY` | -0.3 | m/s | vy 下限 |
| `DWA_MAX_VY` | 0.3 | m/s | vy 上限（横向移动较慢） |
| `DWA_MIN_WZ` | -1.0 | rad/s | wz 下限 |
| `DWA_MAX_WZ` | 1.0 | rad/s | wz 上限 |

### 6.3 加速度约束

| 参数 | 值 | 单位 | 说明 |
|------|------|------|------|
| `DWA_ACC_VX` | 1.0 | m/s² | vx 加速度（前后方向加速最快） |
| `DWA_ACC_VY` | 0.5 | m/s² | vy 加速度（横向加速较慢） |
| `DWA_ACC_WZ` | 2.0 | rad/s² | wz 角加速度（旋转最快） |

### 6.4 评分权重

| 参数 | 值 | 说明 |
|------|------|------|
| `DWA_WEIGHT_HEADING` | 0.35 | 朝向目标权重 |
| `DWA_WEIGHT_CLEARANCE` | 0.35 | 障碍安全距离权重 |
| `DWA_WEIGHT_VELOCITY` | 0.15 | 目标方向速度投影权重 |
| `DWA_WEIGHT_TARGET_DIST` | 0.15 | 理想跟随距离偏差权重 |

**权重设计原则**：安全（0.35）+ 朝向（0.35）合计 0.70，是主导因素；速度与距离为辅助调节项。

### 6.5 安全阈值

| 参数 | 值 | 单位 | 说明 |
|------|------|------|------|
| `DWA_EMERGENCY_DIST` | 0.15 | m | 硬否决距离 |
| `DWA_SAFE_DIST` | 0.4 | m | clearance 满分距离 |
| `DWA_MAX_TARGET_RANGE` | 3.0 | m | 最大有效目标距离 |

### 6.6 参数调优指南

| 场景 | 调整建议 |
|------|---------|
| 避障过于激进（离障碍很远就绕） | 降低 `DWA_WEIGHT_CLEARANCE`，降低 `DWA_SAFE_DIST` |
| 绕行时过于贴近障碍 | 提高 `DWA_EMERGENCY_DIST` 到 0.2 |
| 速度响应迟钝 | 提高 `DWA_ACC_VX` / `DWA_ACC_WZ` |
| 采样耗时过高 | 降低采样数，优先降 `DWA_VY_SAMPLES` 到 3 |
| 轨迹预测不够远 | 提高 `DWA_SIM_TIME` 到 2.0 |
| 障碍密集区找不到出路 | 降低 `DWA_WEIGHT_VELOCITY`，提高 `DWA_WEIGHT_CLEARANCE` |

---

## 7. 详细设计

### 7.1 文件: `common_types.hpp` (修改)

路径: `robot_follow/include/common_types.hpp`

删除 4 个 APF 常量：

```
APF_INFLUENCE_DIST, APF_REPULSE_GAIN, APF_EMERGENCY_DIST, APF_SLOWDOWN_DIST
```

新增 19 个 DWA 常量（见 §6）。

### 7.2 文件: `dwa_planner.hpp` (新建)

路径: `robot_follow/include/dwa_planner.hpp`

```cpp
class DWAPlanner {
public:
    struct Sample {
        double vx, vy, wz;   // 速度指令
        double score;         // 综合评分
    };

    Sample plan(obstacles, target_x, target_y, cur_vx, cur_vy, cur_wz);

private:
    struct VelocityWindow { double min/max for vx,vy,wz; };

    VelocityWindow computeWindow(cvx, cvy, cwz);
    double scoreSample(vx, vy, wz, obstacles, target_x, target_y, num_steps);
};
```

**plan()**: 主入口。检查目标有效性 → 计算动态窗口 → 三重循环采样 (vx, vy, wz) → 调用 scoreSample 评分 → 返回最优。

**computeWindow()**: 加速度约束 ∩ 速度硬约束。

**scoreSample()**: 欧拉积分仿真轨迹 → 每步碰撞检测 → 硬否决检查 → 四维评分。

### 7.3 文件: `lidar_tracker.hpp` (修改)

路径: `robot_follow/include/lidar_tracker.hpp`

**删除**：
- `calculateFollowVelocity()` 方法（约 55 行）
- APF 排斥力累积代码（repulse_x, repulse_y, force 计算）
- 排斥力限幅逻辑
- APF 颜色阈值（`APF_EMERGENCY_DIST`, `APF_SLOWDOWN_DIST`, `APF_INFLUENCE_DIST`）

**新增**：
- `#include "dwa_planner.hpp"`
- `DWAPlanner dwa_planner_;` 成员
- DWA 规划调用：`dwa_planner_.plan(obstacles, target_x, target_y, cur_vx, cur_vy, cur_wz)`

**保留**：
- `processScan()` 的点云过滤和可视化功能
- `processtalker()` 的目标定位功能
- `publishVelocity()` 速度发布
- OpenCV 调试显示

### 7.4 processScan() 修改前后对比

| 步骤 | 修改前 (APF) | 修改后 (DWA) |
|------|-------------|-------------|
| 1. 读目标 | getTarget() | 相同 |
| 2. 点云处理 | 遍历 + 排斥力累加 + 路径通道 | 遍历 + 收集障碍点 |
| 3. 排斥力 | 累加 + 限幅 | **移除** |
| 4. 速度计算 | calculateFollowVelocity(target, left/right, repulse, min_dist) | **dwa_planner_.plan(obstacles, target, cur_vel)** |
| 5. 发布 | publishVelocity() | 相同 |

---

## 8. 状态与数据流

### 8.1 时序

```
Frame N:               Frame N+1:
  LiDAR arrives          LiDAR arrives
    │                       │
    ▼                       ▼
  processScan()           processScan()
    │                       │
    ├─ filter points        ├─ filter points
    ├─ state_.setPoints()   ├─ state_.setPoints()
    ├─ getVelocity() ───────├─ 读 Frame N 的输出速度
    ├─ DWA.plan()           ├─ DWA.plan()
    ├─ state_.setVelocity() ├─ state_.setVelocity()
    └─ publishVelocity()    └─ publishVelocity()
         │                       │
         ▼                       ▼
      /cmd_vel                /cmd_vel
```

每帧 DWA 使用上一帧输出的速度作为当前速度，计算动态窗口。有一帧延迟但不影响效果。

### 8.2 线程模型

```
ROS2 spin 线程:
  scanCallback() → lidar_tracker_.processScan() → DWA.plan() → publish

单线程执行，无并发问题。所有状态读写均在 ROS2 回调线程中。
```

---

## 9. 接口定义

### 9.1 DWAPlanner 对外接口

```cpp
Sample plan(
    const std::vector<std::pair<double, double>>& obstacles,  // 障碍点云 (机器人系)
    double target_x, double target_y,                          // 目标位置 (机器人系)
    double cur_vx, double cur_vy, double cur_wz                // 当前速度
);
// 返回: 最优速度采样 (vx, vy, wz, score)
```

### 9.2 调用示例

```cpp
DWAPlanner dwa;
std::vector<std::pair<double, double>> obs = state_.getPoints();
double cvx, cvy, cwz;
state_.getVelocity(cvx, cvy, cwz);

auto best = dwa.plan(obs, target_x, target_y, cvx, cvy, cwz);
cmd_vel.linear.x  = best.vx;
cmd_vel.linear.y  = best.vy;
cmd_vel.angular.z = best.wz;
```

---

## 10. 验证方案

### 10.1 编译验证

```bash
colcon build --packages-select robot_follow
```

预期：零 error，零 warning。✅ 已通过。

### 10.2 单元级验证

| 编号 | 测试项 | 输入 | 预期 |
|------|--------|------|------|
| UT-01 | 动态窗口-静止 | v=(0,0,0) | wx=[-0.1, 0.1], wy=[-0.05, 0.05], wz=[-0.2, 0.2] |
| UT-02 | 动态窗口-边界 | v=(0.95, 0, 0) | max_vx=1.0 (clamped by MAX_VX) |
| UT-03 | 无目标 | target=(0,0) 或 太远 | 返回 (0,0,0,0) |
| UT-04 | 无障碍直行 | 目标 (1.5, 0, 0)，无障碍 | heading≈1, velocity 高, 最佳 vx>0 |
| UT-05 | 正面障碍 | 目标 (1.0, 0)，障碍在 (0.3, 0) | 碰撞轨迹被否决，选中绕行轨迹 |
| UT-06 | 侧方障碍 | 目标 (1.0, 0)，障碍在 (0, 0.3) | clearance 分数降低，倾向有 vy 的绕行 |
| UT-07 | 硬否决 | 障碍在 (0.10, 0) | 任何前进轨迹均被否决，可能选中原地旋转 |

### 10.3 场景级验证

| 编号 | 场景 | 预期行为 |
|------|------|---------|
| ST-01 | 空旷直行 | 机器人直线前进，速度接近 1.0 m/s |
| ST-02 | 正面墙绕行 | DWA 采样到含 wz 的轨迹得分更高 → 转向绕行 |
| ST-03 | 窄通道 | 机器人在 velocity 与 clearance 间权衡，慢速通过 |
| ST-04 | 目标到达 | 靠近 0.6m 时 velocity 和 target_dist 得分降低→减速 |
| ST-05 | MODE_DIRECT | DWA 不被调用，直接控制正常 |

### 10.4 性能验证

```cpp
// 在 plan() 中插入计时
auto t1 = std::chrono::steady_clock::now();
// ... plan logic ...
auto t2 = std::chrono::steady_clock::now();
auto elapsed = std::chrono::duration<double, std::milli>(t2 - t1).count();
// 预期: elapsed < 20ms
```

---

## 11. 与 APF 方案对比

| 维度 | APF (旧) | DWA (新) |
|------|---------|---------|
| **搜索空间** | 位置空间（力矢量合成） | 速度空间（轨迹采样评估） |
| **预测能力** | 无，仅当前距离 | 前向仿真 1.5s，预判未来碰撞 |
| **动力学约束** | 无，可输出任意速度 | 加速度约束，速度平滑过渡 |
| **优化目标** | 单一（排斥+吸引） | 多目标加权（4 维评分） |
| **局部极小值** | 会卡住 | 多条轨迹采样中自然选择绕行 |
| **参数数量** | 4 个 | 19 个（更灵活但需调参） |
| **计算复杂度** | O(N) 遍历一次 | O(S×K×N)，S=525 采样，K=15 步 |
| **代码量** | ~60 行（APF 部分） | ~150 行（DWA 核心） |
| **可扩展性** | 难以增加新约束 | 评分函数可自由扩展新维度 |

### 核心优势总结

DWA 通过**在速度空间中显式预测和评估多条轨迹**，解决了 APF 的三个根本性问题：

1. **预判碰撞**：不是等障碍逼到眼前才反应，而是提前 1.5s 看到轨迹上的碰撞并绕开
2. **物理可达**：加速度约束保证输出的速度机器狗真的能做到，不会发"跳变"指令
3. **智能权衡**：在避障与跟随之间通过加权评分动态平衡，而不是简单力的线性叠加

---

## 12. 约束与局限

| 编号 | 局限 | 说明 | 改进方向 |
|------|------|------|---------|
| L-01 | 目标假定静止 | 轨迹仿真中目标位置不变 | 用卡尔曼滤波估计目标速度并预推 |
| L-02 | 无动态障碍预测 | 障碍物移动时无速度估计 | 多帧差分检测 + 障碍轨迹预测 |
| L-03 | 3D 采样离散化 | 525 个采样可能遗漏最优解 | 自适应采样密度或粗-精两阶段搜索 |
| L-04 | 恒定仿真参数 | sim_time/dt 固定 | 根据场景自适应（拥挤时延长 sim_time） |
| L-05 | 点模型碰撞检测 | 将机器人视为点 | 用机器人包围盒（矩形/圆形）做膨胀检测 |

---

## A. 修改文件清单

| 文件 | 改动类型 | 说明 |
|------|---------|------|
| `robot_follow/include/common_types.hpp` | 修改 | 删除 4 个 APF 常量，新增 19 个 DWA 常量 |
| `robot_follow/include/dwa_planner.hpp` | **新建** | DWA 规划器核心实现（~150 行） |
| `robot_follow/include/lidar_tracker.hpp` | 修改 | 删除 `calculateFollowVelocity()`（~55 行）、APF 排斥力逻辑；接入 DWA 调用 |
| `robot_follow/docs/apf-vortex-avoidance-design.md` | 保留 | APF+Vortex 方案设计文档（历史参考） |
| `robot_follow/docs/dwa-planner-design.md` | **新建** | 本文档 |

## B. 参考文献

- Fox, D., Burgard, W., & Thrun, S. (1997). *The Dynamic Window Approach to Collision Avoidance.* IEEE Robotics & Automation Magazine.
- Brock, O., & Khatib, O. (1999). *High-Speed Navigation Using the Global Dynamic Window Approach.* ICRA.
- ROS2 Navigation2 Documentation: https://navigation.ros.org/
