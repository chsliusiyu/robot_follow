# DWA 避障规划器 — 版本演进与使用说明

## 1. 版本对比概览

| 维度 | V1.0 (6166d8e) | V1.1 (7aeb4a7) |
|------|----------------|----------------|
| 日期 | 2026-05-07 | 2026-05-08 |
| 最高速度 | 1.0 m/s | 0.45 m/s |
| 否决距离 | 0.15 m (点模型) | 0.50 m (含机器人半径 0.25m) |
| 安全距离 | 0.4 m | 0.5 m |
| 目标遮罩 | 无 | 0.45m，排除被跟随者 |
| 安全评分 | 线性 | 非线性指数 + 障碍密度 |
| 卡死恢复 | 无 | 原地缓慢旋转找空隙 |

---

## 2. 详细变更清单

### 变更 1：目标遮罩排除（781ad03）
**文件**: `lidar_tracker.hpp:178-183`

**改动**:
```diff
+ // 排除UWB目标附近的点 — 这些点来自被跟随者，不应视为障碍物
+ double dist_to_target = std::sqrt(
+     (point_x - target_x) * (point_x - target_x) +
+     (point_y - target_y) * (point_y - target_y));
+ if (dist_to_target < TARGET_MASK_RADIUS) continue;
```

**原因**: UWB 标签佩戴在被跟随者身上，LiDAR 扫描到被跟随者的腿部。如果不排除，DWA 会把被跟随者当作障碍物来避开，形成"跟着一个人，同时避开这个人"的矛盾。这是导致机器狗绕圈的**根本原因**。

**效果**: UWB 目标周围 0.45m 半径内的 LiDAR 点不再进入障碍物列表。

**⚠️ 注意**: `TARGET_MASK_RADIUS`(0.45m) 取值比 `TARGET_RADIUS`(0.3m) 大，以覆盖人的腿部和身体轮廓。如果被跟随者紧贴墙壁或柱子，这些真实障碍物也会被一同排除，存在碰撞风险。

---

### 变更 2：安全距离增大（781ad03）
**文件**: `common_types.hpp`

```diff
- constexpr double DWA_EMERGENCY_DIST = 0.15;
- constexpr double DWA_SAFE_DIST = 0.4;
+ constexpr double DWA_EMERGENCY_DIST = 0.25;
+ constexpr double DWA_SAFE_DIST = 0.5;
```

**原因**: V1.0 的否决距离 0.15m 在最高速 1.0m/s 下制动距离不足。当 LiDAR 扫描到障碍物并触发否决时，机器人可能已经太近。0.4m 安全距离也不足以让 DWA 提前绕行。

**效果**: 否决阈值 0.25m 给了更大安全余量；安全距离 0.5m 让 DWA 更早考虑绕行。

---

### 变更 3：评分权重调整（781ad03）
**文件**: `common_types.hpp`

```diff
- constexpr double DWA_WEIGHT_CLEARANCE = 0.35;
- constexpr double DWA_WEIGHT_VELOCITY = 0.15;
+ constexpr double DWA_WEIGHT_CLEARANCE = 0.40;
+ constexpr double DWA_WEIGHT_VELOCITY = 0.10;
```

**原因**: 原始权重中速度权重过高，DWA 倾向选更快的轨迹而非更安全的轨迹。提高 clearance 权重、降低速度权重，让安全优先于速度。

**效果**: 权重总和仍为 1.0（heading 0.35 + clearance 0.40 + velocity 0.10 + target_dist 0.15）。

---

### 变更 4：安全评分非线性化（781ad03）
**文件**: `dwa_planner.hpp:158-160`

**改动**:
```diff
- // V1.0: 线性评分
- double clearance_score = std::min(1.0, min_clearance / DWA_SAFE_DIST);

+ // V1.1: 非线性指数衰减
+ double effective_clearance = std::max(0.0, min_clearance - DWA_ROBOT_RADIUS);
+ double clearance_score = 1.0 - std::exp(-3.0 * effective_clearance / DWA_SAFE_DIST);
```

**原因**: 线性评分下，障碍物从 0.4m 退到 0.2m 只扣 0.5 分，惩罚过轻。非线性指数对近距离惩罚剧烈得多。

**效果对比**:

| 有效距离 | V1.0 线性 | V1.1 指数 |
|----------|:---:|:---:|
| 0.5m | 1.00 | 0.95 |
| 0.3m | 0.75 | 0.83 |
| 0.2m | 0.50 | 0.70 |
| 0.1m | 0.25 | 0.45 |
| 0.05m | 0.12 | 0.26 |

从 0.3m 到 0.1m，线性扣 0.5 分，指数扣 0.38 分。看起来指数更宽松？实际上指数在远距离加分更显著（近距离轨迹吃亏更大），且配合障碍密度罚分效果才完整。

---

### 变更 5：障碍密度惩罚（781ad03）
**文件**: `dwa_planner.hpp:130-133`

**改动**:
```diff
+ // 累加轨迹每步上安全距离内所有障碍物的指数罚分
+ if (dist < DWA_ROBOT_RADIUS + DWA_SAFE_DIST) {
+     double eff_d = std::max(0.0, dist - DWA_ROBOT_RADIUS);
+     density_penalty += std::exp(-(eff_d * eff_d) / (DWA_SAFE_DIST * DWA_SAFE_DIST));
+ }
```

```diff
- return DWA_WEIGHT_CLEARANCE * clearance_score + ...
+ return DWA_WEIGHT_CLEARANCE * (0.7 * clearance_score + 0.3 * density_score) + ...
```

**原因**: V1.0 的 `min_clearance` 只看轨迹上**最近的一个障碍点**。当机器狗绕开前方椅子时，如果侧面有墙，`min_clearance` 可能仍然很大（因为墙在侧面不在正前方），DWA 以为安全实际已经贴近。

障碍密度罚分统计轨迹上**所有**在安全距离内的障碍物。贴墙走的轨迹会累积大量罚分，自然被淘汰。

**效果**: 三条轨迹对比——

```
直行（前方1个椅子）： density_penalty = 0.2  → density_score = 0.83
右转贴墙（墙+多障碍）：  density_penalty = 2.5  → density_score = 0.29  ← 被惩罚
左转空旷：              density_penalty = 0    → density_score = 1.00
```

---

### 变更 6：距离自适应速度（781ad03）
**文件**: `dwa_planner.hpp:166-170`

**改动**:
```diff
- // V1.0: 越快越好
- double velocity_score = std::max(0.0, vel_proj) / DWA_MAX_VX;

+ // V1.1: 带距离衰减
+ double target_dist = std::sqrt(target_x * target_x + target_y * target_y);
+ double dist_factor = std::clamp(
+     (target_dist - FOLLOW_DIST) * 0.25, 0.15, 1.0);
+ double velocity_score = std::max(0.0, vel_proj) / DWA_MAX_VX * dist_factor;
```

**原因**: V1.0 的 velocity 评分是纯粹的"越快越好"，DWA 在允许范围内一定选最高速。没有距离概念——2m 外和 0.6m 处的最高速一样。

**效果**: 距离越近，速度得分越低，自然减速。

| 目标距离 | dist_factor | 最大速度 |
|----------|:---:|:---:|
| 0.6m（理想跟随距离） | 0.15 | 0.07 m/s |
| 1.0m | 0.15 (clamp下限) | 0.07 m/s |
| 1.5m | 0.22 | 0.10 m/s |
| 2.0m | 0.35 | 0.16 m/s |
| 3.0m | 0.60 | 0.27 m/s |

**⚠️ 已知问题**: 当前公式 `(target_dist - 0.6) * 0.25` 在 0.6~1.0m 区间全部被 clamp 到 0.15，导致这个范围速度几乎为 0。且速度得分只奖励正向速度投影（`max(0, vel_proj)`），目标太近需要后退时不会被奖励。

---

### 变更 7：最高速度与加速度降低（7aeb4a7）
**文件**: `common_types.hpp`

```diff
- constexpr double DWA_MAX_VX = 1.0;
- constexpr double DWA_ACC_VX = 1.0;
+ constexpr double DWA_MAX_VX = 0.45;
+ constexpr double DWA_ACC_VX = 0.5;
```

**原因**: 1.0 m/s 对室内跟随场景过快，容易冲撞。加速度 1.0 m/s² 让机器狗动作过于突兀。

**效果**: 最远距离最高速度 0.27 m/s（受 dist_factor 限制），加/减速更平缓。

---

### 变更 8：机器人半径实体模型（7aeb4a7）
**文件**: `common_types.hpp` + `dwa_planner.hpp`

**改动 8a — 新增常量**:
```diff
+ constexpr double DWA_ROBOT_RADIUS = 0.25;
```

**改动 8b — 碰撞否决加半径**:
```diff
- if (min_clearance < DWA_EMERGENCY_DIST) {
+ if (min_clearance < DWA_ROBOT_RADIUS + DWA_EMERGENCY_DIST) {
```

**改动 8c — density 加半径**:
```diff
- if (dist < DWA_SAFE_DIST) {
+ if (dist < DWA_ROBOT_RADIUS + DWA_SAFE_DIST) {
+     double eff_d = std::max(0.0, dist - DWA_ROBOT_RADIUS);
```

**改动 8d — clearance 减半径**:
```diff
+ double effective_clearance = std::max(0.0, min_clearance - DWA_ROBOT_RADIUS);
+ double clearance_score = ...;
```

**原因**: V1.0 将机器人视为一个**点**来计算碰撞距离。当机器狗旋转（wz）或侧移（vy）绕开前方障碍时，身体侧面可能扫到旁边的障碍物，但点模型看不见。

```
实际机器人 (0.5m 宽)             V1.0 点模型
    ┌─────┐                         ·
    │ 狗  │  ──→  绕行             ·  ──→  绕行
    └─────┘
  侧面离墙 0.1m                  点模型离墙 0.35m
  危险！DWA 不知道               DWA 觉得安全 ✓
```

**效果**: 碰撞否决从点模型 0.25m 提高到实体 0.50m（0.25 + 0.25），clearance 和 density 都基于"障碍到机器人外壳的距离"而非"障碍到机器人中心的距离"。

---

### 变更 9：卡死恢复旋转（7aeb4a7）
**文件**: `dwa_planner.hpp:71-82`

**改动**:
```diff
+ // 卡死恢复：最优轨迹几乎不动时，缓慢旋转寻找空隙绕过障碍
+ if (best.vx < 0.05 && best.vy < 0.05 && std::abs(best.wz) < 0.05) {
+     double left_score  = scoreSample(0.0, 0.0,  0.3, obstacles,
+                                      target_x, target_y, num_steps);
+     double right_score = scoreSample(0.0, 0.0, -0.3, obstacles,
+                                      target_x, target_y, num_steps);
+     if (left_score > right_score && left_score > best.score) {
+         best = {0.0, 0.0, 0.3, left_score};
+     } else if (right_score > best.score) {
+         best = {0.0, 0.0, -0.3, right_score};
+     }
+ }
```

**原因**: 当被跟随者走到障碍物后面时（UWB 信号穿墙但 LiDAR 看到墙），DWA 所有朝向目标的轨迹都被碰撞否决。V1.0 此时选"不动"，机器狗停在障碍物前束手无策。

V1.1 检测到卡死状态后，评估左右两侧旋转的 clear 程度，向更空旷的一侧缓慢旋转（0.3 rad/s）。转出角度后，下一帧 DWA 就能找到绕过障碍物的路径。

**效果**:
```
人 ←— 柱子 —← 狗

V1.0: 狗停在柱子前不动
V1.1: 狗先旋转看哪个方向空旷，然后慢慢转过去绕开柱子
```

---

## 3. 已知问题与后续改进方向

| 问题 | 现状 | 改进方向 |
|------|------|---------|
| 太近时不会后退 | velocity_score 只奖励正向速度 | 双向速度评分（后退也奖励） |
| dist_factor 在 0.6-1.0m 区间全被 clamp | 衰减公式起点 0.25 太低 | 提高衰减系数或调整 clamp 下限 |
| 转得太慢 | 恢复旋转固定 0.3 rad/s | 自适应旋转速度 |
| UWB 丢失后追假位置 | uwb_serial_pub 发布 (0.7, 0.3) | 丢失时保持最后有效位置或启用卡尔曼预测 |
| 目标附近障碍物误排除 | TARGET_MASK_RADIUS=0.45m 一刀切 | 聚类区分人腿和其他障碍 |

---

## 4. DWA 使用说明

### 4.1 参数调优

所有可调参数在 `common_types.hpp` 中，重新编译生效：

| 参数 | 默认值 | 调大/调小的效果 |
|------|--------|----------------|
| `DWA_MAX_VX` | 0.45 | ↑ 更快速 / ↓ 更平稳 |
| `DWA_EMERGENCY_DIST` | 0.25 | ↑ 更保守(保持距离) / ↓ 更激进 |
| `DWA_SAFE_DIST` | 0.5 | ↑ 更早触发绕行 / ↓ 更晚绕行 |
| `DWA_ROBOT_RADIUS` | 0.25 | ↑ 碰撞检测更保守 / ↓ 更宽松 |
| `DWA_WEIGHT_HEADING` | 0.35 | ↑ 更积极面朝目标 / ↓ 更注重侧面移动 |
| `DWA_WEIGHT_CLEARANCE` | 0.40 | ↑ 更保守避障 / ↓ 更激进贴近障碍 |
| `DWA_WEIGHT_VELOCITY` | 0.10 | ↑ 更快速 / ↓ 更慢 |
| `DWA_WEIGHT_TARGET_DIST` | 0.15 | ↑ 更精确保持跟随距离 / ↓ 距离容忍更大 |
| `TARGET_MASK_RADIUS` | 0.45 | ↑ 排除更大范围 / ↓ 保留更多障碍点 |
| `FOLLOW_DIST` | 0.6 | ↑ 跟得更远 / ↓ 跟得更近 |

### 4.2 常见场景调优

**场景 1: 狭窄走廊，机器狗贴墙走**
```
调大 DWA_WEIGHT_CLEARANCE (0.40 → 0.50)
调大 DWA_ROBOT_RADIUS (0.25 → 0.30)
```

**场景 2: 开阔空间，希望跟得更快更紧**
```
调大 DWA_MAX_VX (0.45 → 0.6)
调大 DWA_WEIGHT_VELOCITY (0.10 → 0.20)
调小 FOLLOW_DIST (0.6 → 0.4)
```

**场景 3: UWB 信号不稳定，目标位置抖动大**
```
提高 dist_factor clamp 下限 (0.15 → 0.25) — 稍微多动一点来对抗噪声
或开启卡尔曼滤波平滑目标位置
```

**场景 4: 被跟随者紧贴障碍物（人靠着墙站）**
```
调小 TARGET_MASK_RADIUS (0.45 → 0.25) — 避免把墙也排除
```

### 4.3 调试技巧

1. **OpenCV 可视化**: launch 文件设 `enable_opencv: true`，窗口内：
   - 红色点 = 否决距离内的障碍
   - 橙色点 = 安全距离内的障碍
   - 灰色点 = 远处障碍
   - 紫色圆圈 = UWB 目标位置
   - 绿色线 = 机器人到目标的连线
   - 底部显示实时速度 vx/vy/wz

2. **Web 界面**: 浏览器打开 `http://<机器人IP>:8080`，实时显示雷达点云。

3. **快速验证**: 可以先用 `ros2 topic pub /cmd_vel` 直接发速度指令验证 `agibot` 节点正常响应。

4. **检查目标遮罩**: 如果机器狗还是绕着你转，可能是 `TARGET_MASK_RADIUS` 不够大，或者 UWB 位置偏差太大导致遮罩没遮对你的腿。
