# 增强型人工势场法 + 涡旋场 避障模块设计文档

---

## 文件控制

| 项目 | 内容 |
|------|------|
| 文档标题 | 增强型人工势场法 + 涡旋场 避障模块设计文档 |
| 版本号 | V1.0 |
| 作者 | chs |
| 创建日期 | 2026-05-07 |
| 审核状态 | 已评审 |
| 关联分支 | `feature/enhanced-apf-vortex` |
| 关联提交 | `156ab99` |

### 修订历史

| 版本 | 日期 | 修订者 | 修订内容 |
|------|------|--------|---------|
| V1.0 | 2026-05-07 | chs | 初版：基础 APF 增强 + 涡旋场逃逸机制 |

---

## 目录

1. [引言](#1-引言)
2. [术语与缩略语](#2-术语与缩略语)
3. [需求分析](#3-需求分析)
4. [系统架构](#4-系统架构)
5. [详细设计](#5-详细设计)
6. [算法设计](#6-算法设计)
7. [参数设计](#7-参数设计)
8. [状态机设计](#8-状态机设计)
9. [数据结构设计](#9-数据结构设计)
10. [接口定义](#10-接口定义)
11. [验证方案](#11-验证方案)
12. [约束与局限](#12-约束与局限)

---

## 1. 引言

### 1.1 背景

本项目是一个基于 ROS2 的四足机器狗目标跟随系统。系统通过激光雷达获取环境点云，利用人工势场法（Artificial Potential Field, APF）在跟随目标的同时进行障碍物规避，输出速度指令控制机器狗运动。

### 1.2 问题陈述

当前 APF 避障模块存在以下不足：

| 编号 | 问题 | 影响 |
|------|------|------|
| P-01 | 排斥力增益过低（0.01），单点排斥力在 0.22m 处仅 ~0.005 | 障碍物逼近时无法产生足够的排斥力，避障反应迟钝 |
| P-02 | 增益为静态常量，无速度感知 | 快速运动时无法提前加大排斥力，存在碰撞风险 |
| P-03 | 无局部极小值逃逸机制 | 面对对称障碍（如狭窄通道、U形障碍）可能被卡住，无法自主脱困 |

### 1.3 设计目标

在不引入新依赖、不改变整体架构的前提下，通过局部增强使避障具备：

- **响应速度大幅提升**：基础增益提升 8 倍，快速运动时进一步自适应增强
- **局部极小值自主逃逸**：检测卡住状态并触发涡旋场（Vortex Field），沿障碍切线方向绕行
- **更早的减速预警**：减速距离从 0.25m 扩大到 0.35m，适应机器狗制动特性

### 1.4 设计范围

| 涉及模块 | 是否修改 | 说明 |
|----------|---------|------|
| `common_types.hpp` | 是 | 常量参数调整 + SharedState 扩展 |
| `lidar_tracker.hpp` | 是 | 避障核心逻辑增强 |
| `robot_nexus.cpp` | 否 | 无接口变化，无需修改 |
| `direct_control.hpp` | 否 | 直接控制模式不经过 APF 管线 |
| `kalman_filter.hpp` | 否 | 无关 |
| `web_comm.hpp` / `android_comm.hpp` | 否 | 无关 |

---

## 2. 术语与缩略语

| 术语 | 全称 | 说明 |
|------|------|------|
| APF | Artificial Potential Field | 人工势场法：通过引力场（目标方向）+ 斥力场（障碍物方向）合成运动方向 |
| Vortex Field | — | 涡旋场：在局部极小值处将排斥力方向旋转 90°，产生沿障碍边界的切向绕行力 |
| CCW / CW | Counter-Clockwise / Clockwise | 涡旋方向：CCW = 逆时针（从上方看，驱动向右绕行），CW = 顺时针 |
| Local Minima | — | 局部极小值：斥力与引力的合力为零或互相抵消，导致机器人停止运动的位置 |
| Stuck Detection | — | 卡住检测：连续多帧速度接近零但目标未到达，判定为卡住状态 |
| Dynamic Gain | — | 动态增益：排斥力增益随当前速度自适应变化 |

---

## 3. 需求分析

### 3.1 功能需求

| 编号 | 需求描述 | 优先级 |
|------|---------|--------|
| F-01 | 排斥力增益可根据机器人当前速度自适应调整 | P0 |
| F-02 | 检测机器人在运动中被障碍物卡住的状态 | P0 |
| F-03 | 卡住时自动触发涡旋场，沿障碍切线方向绕行 | P0 |
| F-04 | 绕行方向根据障碍物空间分布不对称性智能选择 | P1 |
| F-05 | 恢复正常运动后自动退出涡旋场模式 | P0 |
| F-06 | 减速距离提前，为机器狗提供更充足的制动余量 | P1 |

### 3.2 非功能需求

| 编号 | 需求描述 |
|------|---------|
| NF-01 | 改动仅限 `common_types.hpp` 和 `lidar_tracker.hpp`，不修改接口 |
| NF-02 | MODE_DIRECT 模式行为完全不受影响 |
| NF-03 | 紧急停止（`APF_EMERGENCY_DIST = 0.2m`）行为不变 |
| NF-04 | 编译无新增错误或警告 |

---

## 4. 系统架构

### 4.1 整体数据流

```
LiDAR (/scan)
    │
    ▼
┌─────────────────────────────────────────────────┐
│              LidarTracker::processScan()          │
│                                                  │
│  ┌──────────────────────────────────────────┐   │
│  │  1. 读取目标位置 (state_.getTarget)       │   │
│  └──────────────────────────────────────────┘   │
│                     │                            │
│                     ▼                            │
│  ┌──────────────────────────────────────────┐   │
│  │  2. [新增] 动态增益计算 + 卡住检测         │   │
│  │     - computeDynamicGain(current_speed)   │   │
│  │     - isRobotStuck()                      │   │
│  │     - 涡旋状态管理                         │   │
│  └──────────────────────────────────────────┘   │
│                     │                            │
│                     ▼                            │
│  ┌──────────────────────────────────────────┐   │
│  │  3. 遍历激光点云                           │   │
│  │     - 机器人框架过滤                       │   │
│  │     - [修改] 排斥力累加(使用 dynamic_gain)  │   │
│  │     - 路径通道宽度计算                     │   │
│  └──────────────────────────────────────────┘   │
│                     │                            │
│                     ▼                            │
│  ┌──────────────────────────────────────────┐   │
│  │  4. calculateFollowVelocity()             │   │
│  │     - 紧急停止检查 (不变)                  │   │
│  │     - 前后/旋转/横向运动计算 (不变)         │   │
│  │     - [新增] 涡旋调制(旋转排斥力90°)       │   │
│  │     - 排斥力融合 + 减速 + 限幅             │   │
│  └──────────────────────────────────────────┘   │
│                                                  │
└─────────────────────────────────────────────────┘
    │
    ▼
/cmd_vel (geometry_msgs::Twist)
    │
    ▼
HighLevel::move(vx, vy, yaw_rate)
```

### 4.2 模块依赖关系

```
                  ┌───────────────────┐
                  │  common_types.hpp │
                  │  (SharedState +    │
                  │   常量定义)         │
                  └──────┬────────────┘
                         │
           ┌─────────────┼──────────────┐
           │             │              │
           ▼             ▼              ▼
  ┌────────────┐ ┌────────────┐ ┌─────────────┐
  │LidarTracker│ │ DirectCtrl │ │ WebComm /   │
  │ (避障核心)  │ │ (直接控制)  │ │ AndroidComm │
  └─────┬──────┘ └─────┬──────┘ └──────┬──────┘
        │              │               │
        └──────────────┼───────────────┘
                       │
                       ▼
              ┌─────────────────┐
              │ robot_nexus.cpp │  (ROS2 节点)
              └─────────────────┘
```

---

## 5. 详细设计

### 5.1 文件: `common_types.hpp`

路径: `robot_follow/include/common_types.hpp`

#### 5.1.1 现有常量修改

```cpp
// [修改前] constexpr double APF_REPULSE_GAIN = 0.01;
// [修改后] 重命名并提值
constexpr double APF_BASE_REPULSE_GAIN = 0.08; // 排斥力基础增益 (原0.01, 提升8倍)

// [修改前] constexpr double APF_SLOWDOWN_DIST = 0.25;
// [修改后] 扩大减速预警距离
constexpr double APF_SLOWDOWN_DIST = 0.35;    // 减速距离 (原0.25, 提前预警)
```

#### 5.1.2 新增常量

```cpp
constexpr double APF_MIN_REPULSE_GAIN  = 0.04;  // 动态增益下限 (静止时最小值)
constexpr double APF_MAX_REPULSE_GAIN  = 0.20;  // 动态增益上限 (全速时最大值)
constexpr double APF_VORTEX_THRESHOLD  = 3;     // 连续卡住帧数阈值 (3帧 ≈ 0.3s @10Hz)
constexpr double APF_STUCK_SPEED_THRESHOLD = 0.03; // "卡住"判定速度阈值 (m/s)
```

**选值依据**：

- `APF_MIN_REPULSE_GAIN = 0.04`：原值 0.01 的 4 倍，保证即使静止时也有足够排斥力
- `APF_MAX_REPULSE_GAIN = 0.20`：全速时增益 = 0.08×(1+1.0) = 0.16，上限 0.20 留有安全余量
- `APF_VORTEX_THRESHOLD = 3`：避开短暂减速的误触发，3 帧（@10Hz = 0.3s）确认真正卡住
- `APF_STUCK_SPEED_THRESHOLD = 0.03`：低于最小输出速度 0.06m/s 的一半，区分"慢行"与"卡死"

#### 5.1.3 SharedState 新增字段

在 `std::atomic<int> control_mode{MODE_FOLLOW};` 之后新增：

```cpp
// 涡旋场避障状态追踪
std::atomic<int> stuck_frame_count{0};    // 连续卡住帧计数 (单调递增, 正常时归零)
std::atomic<bool> vortex_active{false};   // 涡旋场激活标志 (true = 当前处于涡旋绕行模式)
std::atomic<int> vortex_direction{0};     // 涡旋方向: +1=CCW(右绕), -1=CW(左绕), 0=未激活
```

**字段生命周期**：

```
  stuck_frame_count:  每帧卡住→+1,  正常→→0
  vortex_active:      卡住≥3帧→true, 正常→false
  vortex_direction:   涡旋激活时→+1/-1, 重置时→0
```

---

### 5.2 文件: `lidar_tracker.hpp`

路径: `robot_follow/include/lidar_tracker.hpp`

#### 5.2.1 新增辅助方法（4 个）

**方法 1: `getCurrentSpeed()`**

```cpp
double getCurrentSpeed() {
    double vx, vy, wz;
    state_.getVelocity(vx, vy, wz);
    return std::sqrt(vx * vx + vy * vy);  // 仅考虑平移速度，忽略 wz
}
```

- 输入：无（从共享状态读取缓存速度）
- 输出：合成平移速率 (m/s)
- 用途：速度自适应增益计算 + 卡住检测

**方法 2: `computeDynamicGain(double current_speed)`**

```cpp
double computeDynamicGain(double current_speed) {
    double gain = APF_BASE_REPULSE_GAIN * (1.0 + current_speed / MAX_LINEAR_SPEED);
    return std::clamp(gain, APF_MIN_REPULSE_GAIN, APF_MAX_REPULSE_GAIN);
}
```

- 输入：当前合成速度 (m/s)
- 输出：自适应排斥力增益
- 映射关系：

| 速度 (m/s) | 增益值 | 说明 |
|-----------|--------|------|
| 0.0 | 0.08 | 静止，基础增益 |
| 0.3 | 0.104 | 慢速 |
| 0.5 | 0.12 | 中速 |
| 0.8 | 0.144 | 快速 |
| 1.0 | 0.16 | 全速（未触及上限 0.20） |

**方法 3: `isRobotStuck(double current_speed, double target_x, double target_y)`**

```cpp
bool isRobotStuck(double current_speed, double target_x, double target_y) {
    double target_dist = std::sqrt(target_x * target_x + target_y * target_y);
    bool target_reached = (target_dist <= FOLLOW_DIST + 0.05);
    bool is_moving = state_.is_moving_enabled.load();
    return is_moving && !target_reached && (current_speed < APF_STUCK_SPEED_THRESHOLD);
}
```

**卡住判定三条件**：

1. 运动模式开启（`is_moving_enabled == true`）
2. 目标未到达（`target_dist > FOLLOW_DIST + 0.05 = 0.65m`）
3. 当前几乎静止（`current_speed < 0.03 m/s`）

**设计考虑**：条件 2 避免机器人到达目标后正常停车被误判为卡住；条件 1 确保停止模式不会被触发涡旋。

**方法 4: `computeVortexDirection(const vector<pair<double,double>>& points)`**

```cpp
int computeVortexDirection(const std::vector<std::pair<double, double>>& points) {
    double left_weight = 0.0;   // py > 0 侧 (机器人左侧)
    double right_weight = 0.0;  // py <= 0 侧 (机器人右侧)
    for (const auto& [px, py] : points) {
        double dist = std::sqrt(px * px + py * py);
        // 只考虑影响范围内的前端/侧端障碍
        if (dist < APF_INFLUENCE_DIST && px > -0.1 && dist > 0.01) {
            double weight = 1.0 / (dist * dist);  // 越近权重越大
            if (py > 0.0) left_weight += weight;
            else right_weight += weight;
        }
    }
    return (left_weight > right_weight) ? 1 : -1;  // 绕向障碍少的一侧
}
```

**绕行方向决策逻辑**：

- 左侧障碍权重大 → 返回 `+1`(CCW) → 排斥力旋转 +90° → **向右绕行**
- 右侧障碍权重大或相等 → 返回 `-1`(CW) → 排斥力旋转 -90° → **向左绕行**

#### 5.2.2 processScan() 修改

**修改点 A：卡住检测与涡旋激活**（位于读取 target 之后、点云遍历之前）

```cpp
// 动态增益与卡住检测
double current_speed = getCurrentSpeed();
double dynamic_gain = computeDynamicGain(current_speed);

if (isRobotStuck(current_speed, target_x, target_y)) {
    int stuck_count = state_.stuck_frame_count.load() + 1;
    state_.stuck_frame_count.store(stuck_count);
    if (stuck_count >= APF_VORTEX_THRESHOLD && !state_.vortex_active.load()) {
        // 达到阈值 → 分析障碍不对称性 → 激活涡旋
        std::vector<std::pair<double, double>> pts = state_.getPoints();
        int vdir = computeVortexDirection(pts);
        state_.vortex_direction.store(vdir);
        state_.vortex_active.store(true);
    }
} else {
    // 未卡住或已恢复 → 重置所有涡旋状态
    state_.stuck_frame_count.store(0);
    state_.vortex_active.store(false);
    state_.vortex_direction.store(0);
}
```

**修改点 B：动态增益替换静态增益**（位于排斥力计算行）

```cpp
// [修改前]
// double force = APF_REPULSE_GAIN * (1.0/dist - 1.0/APF_INFLUENCE_DIST) / (dist*dist);

// [修改后]
double force = dynamic_gain * (1.0 / dist_to_robot - 1.0 / APF_INFLUENCE_DIST)
               / (dist_to_robot * dist_to_robot);
```

#### 5.2.3 calculateFollowVelocity() 修改

**修改点 C：涡旋调制**（位于排斥力融入速度之前）

```cpp
// 涡旋场调制：卡住时将排斥力旋转90度绕开障碍
if (state_.vortex_active.load()) {
    int vdir = state_.vortex_direction.load();
    double rx = repulse_x;
    double ry = repulse_y;
    if (vdir > 0) {
        // CCW: 排斥力旋转 +90°
        repulse_x =  ry;
        repulse_y = -rx;
    } else {
        // CW: 排斥力旋转 -90°
        repulse_x = -ry;
        repulse_y =  rx;
    }
}

// 融合势场排斥力 (原有代码不变)
cmd.linear.x += repulse_x;
cmd.linear.y += repulse_y;
```

**旋转几何意义**：

```
正常 APF 排斥力:  F_repulse → 指向远离障碍方向
涡旋 CCW (+90°):  F_vortex   → 沿障碍边界向右切向力
涡旋 CW  (-90°):  F_vortex   → 沿障碍边界向左切向力

              ↑ (forward)
         障碍物
           /|\
    ← ← ←  |  → → →  (vortex forces)
          机器狗
```

---

## 6. 算法设计

### 6.1 主流程伪代码

```
Algorithm: enhancedAPF_ProcessScan
Input: LaserScan (ranges[], angle_min, angle_increment)
Output: cmd_vel (vx, vy, wz) → published to /cmd_vel

1.  If state_.active == false: return

2.  target ← state_.getTarget()

3.  // ===== [新增] 动态增益与卡住检测 =====
    current_speed ← sqrt(cached_vx² + cached_vy²)
    dynamic_gain ← clamp(
        APF_BASE_REPULSE_GAIN * (1.0 + current_speed / MAX_LINEAR_SPEED),
        APF_MIN_REPULSE_GAIN,
        APF_MAX_REPULSE_GAIN
    )

    target_dist ← sqrt(target.x² + target.y²)
    target_reached ← (target_dist ≤ FOLLOW_DIST + 0.05)
    is_stuck ← (is_moving_enabled AND NOT target_reached
                 AND current_speed < 0.03)

    IF is_stuck THEN
        stuck_frame_count ← stuck_frame_count + 1
        IF stuck_frame_count ≥ 3 AND NOT vortex_active THEN
            // 分析障碍分布不对称性
            left_weight, right_weight ← analyzeAsymmetry(cached_points)
            vortex_direction ← (left_weight > right_weight) ? +1 : -1
            vortex_active ← true
        END IF
    ELSE
        stuck_frame_count ← 0
        vortex_active ← false
        vortex_direction ← 0
    END IF

4.  // ===== 遍历激光点 =====
    repulse_x ← 0, repulse_y ← 0
    min_obstacle_dist ← ∞

    FOR each valid point (px, py) in scan_ranges DO
        dist ← sqrt(px² + py²)

        // 机器人自身框架过滤
        in_frame ← (px ∈ [-0.35, 0.15]) AND (py ∈ [-0.15, 0.15])
        IF NOT in_frame THEN
            min_obstacle_dist ← min(min_obstacle_dist, dist)
        END IF

        // [修改] 排斥力计算 (使用动态增益)
        IF NOT in_frame AND dist < 0.25 AND px > -0.1 THEN
            force ← dynamic_gain * (1.0/dist - 1.0/0.25) / dist²
            repulse_x ← repulse_x - force * px / dist
            repulse_y ← repulse_y - force * py / dist
        END IF

        // 路径通道计算 (原有逻辑)
        ...
    END FOR

    // 排斥力限幅 (原有逻辑)
    IF sqrt(repulse_x² + repulse_y²) > 1.0 THEN
        归一化到幅值 1.0
    END IF

5.  // ===== 速度合成 =====
    calculateFollowVelocity(cmd, target, left_y_min, right_y_min,
                             repulse_x, repulse_y, min_obstacle_dist)

6.  // 发布速度
    state_.setVelocity(cmd.vx, cmd.vy, cmd.wz)
    publishVelocity(cmd, mode)
```

### 6.2 calculateFollowVelocity() 流程

```
Algorithm: calculateFollowVelocity
Input: cmd (out), target_x, target_y, left_y_min, right_y_min,
       repulse_x, repulse_y, min_obstacle_dist

1.  // 紧急停止 (最高优先级, 不变)
    IF min_obstacle_dist < 0.2 THEN
        (cmd.vx, cmd.vy, cmd.wz) ← (0, 0, 0); return
    END IF

2.  // 前后运动 (不变)
    dist_error ← target_x - 0.6
    cmd.vx ← (|dist_error| < 0.05) ? 0 : dist_error * 0.5
    IF cmd.vx < 0 THEN cmd.vx ← cmd.vx * 0.8 END IF

3.  // 旋转运动 (不变)
    angle_error ← atan2(target_y, target_x)
    cmd.wz ← (|angle_error| < 0.1) ? 0 : angle_error * 1.0

4.  // 横向运动 (不变)
    lateral_error ← -(left_y_min + right_y_min)
    cmd.vy ← (|lateral_error| < 0.03) ? 0 : lateral_error * 1.0

5.  // ===== [新增] 涡旋调制 =====
    IF vortex_active THEN
        IF vortex_direction == +1 THEN  // CCW = 右绕
            (repulse_x, repulse_y) ← (repulse_y, -repulse_x)
        ELSE                              // CW = 左绕
            (repulse_x, repulse_y) ← (-repulse_y, repulse_x)
        END IF
    END IF

6.  // 排斥力融合 (不变)
    cmd.vx ← cmd.vx + repulse_x
    cmd.vy ← cmd.vy + repulse_y

7.  // [修改参数] 减速 (APF_SLOWDOWN_DIST: 0.25→0.35)
    IF min_obstacle_dist < 0.35 THEN
        factor ← (min_obstacle_dist - 0.2) / (0.35 - 0.2)
        factor ← clamp(factor, 0.1, 1.0)
        cmd.vx ← cmd.vx * factor
    END IF

8.  // 速度限幅 (不变)
    cmd.vx ← clamp(cmd.vx, -1.0, 1.0)
    cmd.vy ← clamp(cmd.vy, -1.0, 1.0)
    cmd.wz ← clamp(cmd.wz, -1.0, 1.0)
```

### 6.3 障碍不对称性分析伪代码

```
Algorithm: computeVortexDirection
Input: points[] (障碍点云, 机器人坐标系)
Output: +1 (CCW/右绕) 或 -1 (CW/左绕)

    left_weight ← 0.0
    right_weight ← 0.0

    FOR each (px, py) in points DO
        dist ← sqrt(px² + py²)
        // 条件：在影响范围内 + 前端/侧端 + 非零距
        IF dist < 0.25 AND px > -0.1 AND dist > 0.01 THEN
            weight ← 1.0 / dist²  // 距离越近权重越大
            IF py > 0.0 THEN
                left_weight ← left_weight + weight
            ELSE
                right_weight ← right_weight + weight
            END IF
        END IF
    END FOR

    // 绕向障碍较少的一侧
    RETURN (left_weight > right_weight) ? +1 : -1
```

---

## 7. 参数设计

### 7.1 完整参数表

| 参数名 | 值 | 单位 | 原值 | 说明 |
|--------|------|------|------|------|
| `APF_INFLUENCE_DIST` | 0.25 | m | 0.25 | 障碍物影响距离（不变） |
| `APF_EMERGENCY_DIST` | 0.20 | m | 0.20 | 紧急停止距离（不变） |
| `APF_BASE_REPULSE_GAIN` | 0.08 | — | 0.01 | 基础排斥力增益（8x） |
| `APF_MIN_REPULSE_GAIN` | 0.04 | — | — | 动态增益下限 |
| `APF_MAX_REPULSE_GAIN` | 0.20 | — | — | 动态增益上限 |
| `APF_SLOWDOWN_DIST` | 0.35 | m | 0.25 | 减速预警距离 |
| `APF_VORTEX_THRESHOLD` | 3 | 帧 | — | 涡旋激活所需连续卡住帧数 |
| `APF_STUCK_SPEED_THRESHOLD` | 0.03 | m/s | — | 卡住判定速度阈值 |
| `FOLLOW_DIST` | 0.6 | m | 0.6 | 目标跟随预设距离（不变） |
| `MAX_LINEAR_SPEED` | 1.0 | m/s | 1.0 | 最大平移速度（不变） |

### 7.2 参数调优建议

| 场景 | 建议调整 |
|------|---------|
| 避障过于激进（绕行过早、距离太远） | 降低 `APF_BASE_REPULSE_GAIN` 至 0.06 |
| 仍然被卡住不触发涡旋 | 降低 `APF_VORTEX_THRESHOLD` 至 2，降低 `APF_STUCK_SPEED_THRESHOLD` 至 0.02 |
| 涡旋误触发（正常行驶中激活） | 提高 `APF_VORTEX_THRESHOLD` 至 5 |
| 绕行方向选择不合理 | 检查 `computeVortexDirection()` 的权重计算条件 |

---

## 8. 状态机设计

### 8.1 避障状态机

```
                        ┌─────────────┐
                        │   NORMAL    │
                        │  (正常APF)   │
                        └──────┬──────┘
                               │
                     isStuck() == true
                     stuck_count = 1,2
                               │
                               ▼
                        ┌─────────────┐
                        │  PRE_VORTEX │
                        │ (卡住计数中)  │
                        └──────┬──────┘
                               │
                     stuck_count >= 3
                               │
                               ▼
                        ┌─────────────┐
                        │   VORTEX    │
                        │  (涡旋绕行)  │
                        └──────┬──────┘
                               │
                  isStuck() == false
         (速度恢复 或 目标到达)
                               │
                               ▼
                        ┌─────────────┐
                        │   NORMAL    │
                        │ (重置状态)   │
                        └─────────────┘
```

### 8.2 状态转移表

| 当前状态 | 条件 | 动作 | 下一状态 |
|----------|------|------|----------|
| NORMAL | `isStuck() == true`, count=1 | count++ | PRE_VORTEX |
| PRE_VORTEX | `isStuck() == true`, count<3 | count++ | PRE_VORTEX |
| PRE_VORTEX | `isStuck() == false` | count=0 | NORMAL |
| PRE_VORTEX | count ≥ 3 | 激活涡旋, 选方向 | VORTEX |
| VORTEX | `isStuck() == true` | 保持涡旋 | VORTEX |
| VORTEX | `isStuck() == false` | 重置所有状态 | NORMAL |

### 8.3 状态转移时序

```
帧序列:    N  N  N  S  S  S  V  V  V  V  N  N
stuck:     0  0  0  1  2  3  4  5  6  7  0  0
vortex:    -  -  -  -  -  *  A  A  A  A  -  -
          (正常行驶) (卡住累积) (涡旋激活)(恢复)

N = NORMAL, S = PRE_VORTEX, V = VORTEX
* = 涡旋激活时刻, A = 涡旋保持
```

---

## 9. 数据结构设计

### 9.1 SharedState 字段完整定义

```cpp
struct SharedState {
    // ===== 互斥保护数据 =====
    std::mutex target_mutex;
    double target_x = FOLLOW_DIST;     // 目标 X 坐标 (机器人坐标系)
    double target_y = 0.0;             // 目标 Y 坐标

    std::mutex velocity_mutex;
    double cached_vx = 0.0;            // 上一帧速度 Vx
    double cached_vy = 0.0;            // 上一帧速度 Vy
    double cached_wz = 0.0;            // 上一帧角速度 Wz

    std::mutex scan_data_mutex;
    std::vector<std::pair<double, double>> cached_points; // 点云缓存

    std::mutex direct_cmd_mutex;
    double direct_vx, direct_vy, direct_wz; // 直接控制指令

    // ===== 原子状态 (无锁访问) =====
    std::atomic<bool> active{false};           // 系统激活标志
    std::atomic<bool> is_moving_enabled{false}; // 运动使能标志
    std::atomic<int> control_mode{MODE_FOLLOW}; // 控制模式

    // ===== [新增] 涡旋场避障状态 =====
    std::atomic<int> stuck_frame_count{0};     // 连续卡住帧计数
    std::atomic<bool> vortex_active{false};    // 涡旋激活标志
    std::atomic<int> vortex_direction{0};      // 涡旋方向 (+1 / -1 / 0)
};
```

### 9.2 线程安全分析

| 字段 | 读写者 | 安全机制 |
|------|--------|---------|
| `stuck_frame_count` | 写: `processScan()` (单线程回调), 读: `processScan()` 自身 | atomic：自增 + 自读，无竞争 |
| `vortex_active` | 写: `processScan()`, 读: `calculateFollowVelocity()` (同调用链) | 同一调用栈内，无竞争 |
| `vortex_direction` | 写: `processScan()`, 读: `calculateFollowVelocity()` | 同一调用栈内，无竞争 |

三个新字段均为原子类型，写操作在 `processScan()` 中（单线程 ROS 回调），读操作在 `calculateFollowVelocity()` 中（同一调用链），无跨线程竞争风险。

---

## 10. 接口定义

### 10.1 本模块对外接口

本模块不新增或修改任何对外接口。现有的速度发布接口保持不变：

```cpp
// 速度发布回调 (由 robot_nexus.cpp 注入)
using VelocityCallback = std::function<void(const geometry_msgs::msg::Twist&)>;
```

### 10.2 本模块内部接口

```cpp
// === 新增内部方法 ===

// 获取当前合成速度
// 返回: sqrt(vx² + vy²) (m/s)
double getCurrentSpeed();

// 计算动态排斥力增益
// 参数: current_speed - 当前合成速度 (m/s)
// 返回: 自适应增益值, 范围 [APF_MIN_REPULSE_GAIN, APF_MAX_REPULSE_GAIN]
double computeDynamicGain(double current_speed);

// 判断机器人是否卡住
// 参数: current_speed - 当前合成速度, target_x/y - 目标位置
// 返回: true = 卡住 (运动中、未达目标、速度接近零)
bool isRobotStuck(double current_speed, double target_x, double target_y);

// 分析障碍物空间分布不对称性
// 参数: points - 点云数据 (机器人坐标系, 已过滤自身框架)
// 返回: +1 = 右侧障碍较多, 绕行方向 CCW (向右绕)
//       -1 = 左侧障碍较多, 绕行方向 CW  (向左绕)
int computeVortexDirection(const std::vector<std::pair<double, double>>& points);
```

---

## 11. 验证方案

### 11.1 编译验证

```bash
colcon build --packages-select robot_follow
```

预期：编译成功，无错误无警告。✅ 已通过（提交 `156ab99`）

### 11.2 单元级验证

| 编号 | 测试项 | 输入条件 | 预期输出 |
|------|--------|---------|---------|
| UT-01 | 动态增益-静止 | current_speed = 0.0 | gain = 0.08 |
| UT-02 | 动态增益-中速 | current_speed = 0.5 | gain = 0.12 |
| UT-03 | 动态增益-全速 | current_speed = 1.0 | gain = 0.16 |
| UT-04 | 动态增益-超速 | current_speed = 2.0 | gain = 0.20 (clamped) |
| UT-05 | 卡住判定-真 | moving=true, dist=1.0, speed=0.01 | true |
| UT-06 | 卡住判定-目标已达 | moving=true, dist=0.62, speed=0.0 | false |
| UT-07 | 卡住判定-速度正常 | moving=true, dist=1.0, speed=0.5 | false |
| UT-08 | 涡旋方向-左偏 | points 集中在 py>0 侧 | +1 (CCW) |
| UT-09 | 涡旋方向-右偏 | points 集中在 py<0 侧 | -1 (CW) |
| UT-10 | 涡旋方向-对称 | left_weight == right_weight | -1 (默认 CW) |

### 11.3 场景级验证

| 编号 | 场景 | 操作 | 预期 |
|------|------|------|------|
| ST-01 | 正面接近墙 | 目标在墙后，机器人向前跟随 | 距离 < 0.35m 开始减速；< 0.2m 紧急停止 |
| ST-02 | 狭窄通道卡住 | 目标在窄通道中，机器人抵近后无法前进 | 连续 3 帧卡住 → 涡旋激活 → 产生横向速度 |
| ST-03 | 涡旋退出 | 涡旋绕行一段后无阻挡 | 速度恢复 → 卡住判定 false → 涡旋失活 |
| ST-04 | 正常无障碍跟随 | 目标前方空旷 | 排斥力 = 0 → 不触发涡旋 → 正常跟随 |
| ST-05 | MODE_DIRECT | 切换到直接控制模式 | calculateFollowVelocity() 不执行 → 无影响 |

### 11.4 回归验证

| 编号 | 检查项 | 预期结果 |
|------|--------|---------|
| RG-01 | 紧急停止 (dist < 0.2m) | 所有速度置零，优先级不变 |
| RG-02 | 直接控制模式 | 不受任何影响 |
| RG-03 | 运动关闭 | is_moving_enabled=false 时不触发涡旋 |
| RG-04 | target 已到达 | 静止不触发卡住检测 |

---

## 12. 约束与局限

### 12.1 已知局限

| 编号 | 局限 | 说明 | 改进方向 |
|------|------|------|---------|
| L-01 | 仅考虑前方障碍 | `point_x > -0.1` 过滤条件排除后方 | 后退场景需单独处理 |
| L-02 | 涡旋依赖速度缓存 | `getCurrentSpeed()` 读上一帧缓存，有一帧延迟 | 可接受（卡住检测本身需要多帧） |
| L-03 | 无动态障碍物预测 | 对移动中的障碍物无速度估计 | 后续引入 DWA 或 MPC |
| L-04 | 对称障碍场默认左绕 | 无偏好的情况默认 CW | 可加入历史方向偏好 |
| L-05 | 未利用 angular velocity | 卡住检测仅考虑平移速度 | 旋转避障场景可改进 |

### 12.2 适用范围

- 室内低速场景（v ≤ 1.0 m/s）
- 静态或准静态障碍物
- 跟随模式下使用
- 激光雷达需正常工作（10Hz 扫描）

---

## 附录

### A. 修改文件清单

| 文件 | 改动类型 | 行数变化 |
|------|---------|---------|
| `robot_follow/include/common_types.hpp` | 常量修改 + 新增 | +6 -2 |
| `robot_follow/include/lidar_tracker.hpp` | 逻辑增强 + 新增方法 | +82 净增 |

### B. 参考文献

- Khatib, O. (1986). Real-Time Obstacle Avoidance for Manipulators and Mobile Robots.
- Borenstein, J., & Koren, Y. (1991). The Vector Field Histogram - Fast Obstacle Avoidance for Mobile Robots.
- Fox, D., Burgard, W., & Thrun, S. (1997). The Dynamic Window Approach to Collision Avoidance.

### C. 术语对照

| 中文 | English | 备注 |
|------|---------|------|
| 人工势场法 | Artificial Potential Field (APF) | — |
| 涡旋场 | Vortex Field | 局部极小值逃逸机制 |
| 排斥力 | Repulsive Force | 障碍物产生的排斥力 |
| 动态增益 | Dynamic Gain | 速度自适应增益 |
| 卡住检测 | Stuck Detection | 连续低速判定 |
| 局部极小值 | Local Minima | 合力为零的位姿 |
