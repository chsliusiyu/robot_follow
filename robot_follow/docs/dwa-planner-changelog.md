# DWA 避障规划器 — 版本演进与使用说明

## 1. 版本对比概览

| 维度 | V1.0 (6166d8e) | V1.1 (7aeb4a7) | V1.2 (051f85d) | V1.3 | V1.4 (当前) |
|------|----------------|----------------|----------------|------|-------------|
| 日期 | 2026-05-07 | 2026-05-08 | 2026-05-09 | 2026-05-13 | 2026-05-19 |
| 最高速度 | 1.0 m/s | 0.45 m/s | 0.45 m/s | 0.45 m/s | 0.45 m/s |
| 否决距离 | 0.15 m | 0.50 m (含半径) | 0.35 m (点模型) | 0.35 m (点模型) | 0.35 m (点模型) |
| 安全距离 | 0.4 m | 0.5 m | 0.5 m | 0.5 m | 0.5 m |
| 目标遮罩 | 无 | 0.45m | 0.25m | 0.30m | 占据栅格动态分离 |
| 安全评分 | 线性 | 非线性指数 + 障碍密度 | 非线性指数 + 障碍密度 | 非线性指数 + 障碍密度 | 非线性指数 + 障碍密度 |
| 卡死恢复 | 无 | 原地旋转 | 横向平移 vy=±0.15 | 横向平移 vy=±0.25 + 趋势否决 | 横向平移 vy=±0.25 + 趋势否决 |
| UWB 丢失 | 追假位置 | 追假位置 | 停止→超时旋转搜索 | 三态(正常/陈旧/超时) + RSSI=-79 | 三态(正常/陈旧/超时) + RSSI=-79 |
| 朝向评分 | 线性 1-angle/π | 线性 1-angle/π | 线性 1-angle/π | cos(angle) 终点 | cos²(angle) 每步平均 |
| 速度对齐 | 无 | 无 | 无 | |cos(target_angle)| 惩罚斜行 | |cos(target_angle)| 惩罚斜行 |
| wz 加速度 | 2.0 | 2.0 | 2.0 | 4.0 rad/s² | 2.5 rad/s² |
| 距离衰减 | 0.8× + 0.2 | 0.25× + 0.15 | 0.5× + 0.15 | 0.5× + 0.15 | 0.5× + 0.15 |
| 控制模式 | 纯 DWA | 纯 DWA | 纯 DWA | 纯 DWA | 两阶段(PD直走/DWA避障) |

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

---

## 3. V1.2 变更清单（2026-05-09）

### 变更 10：回退机器人半径 + 旋转恢复（bb3e8bd）

**文件**: `common_types.hpp` + `dwa_planner.hpp`

V1.1 的 DWA_ROBOT_RADIUS（变更 8）和卡死旋转恢复（变更 9）在实际测试中使机器人行为过于保守，整体回退。

**保留**: DWA_MAX_VX=0.45 的速度降低。

**效果**: 回到点模型碰撞检测（`min_clearance < DWA_EMERGENCY_DIST`），卡死时返回零速。

---

### 变更 11：UWB 信号丢失搜索 + 停止假位置（bdd9f2f, 051f85d）

**文件**: `uwb_serial_pub.cpp` + `common_types.hpp` + `lidar_tracker.hpp`

**改动 11a — 停止发布假位置**:
```diff
- else
-     publish_point(0.7,0.3);
+ // 信号丢失时不发布，由上层超时检测触发搜索旋转
```

**改动 11b — 新增常量**:
```diff
+ constexpr double UWB_TIMEOUT_S = 0.2;   // UWB 超时进入搜索 (秒)
+ constexpr double UWB_SEARCH_WZ = 0.5;   // 搜索旋转速度 (rad/s)
```

**改动 11c — lidar_tracker 超时检测**:
```diff
+ auto now = std::chrono::steady_clock::now();
+ double uwb_elapsed = std::chrono::duration<double>(now - last_uwb_time_).count();
+ if (uwb_elapsed > UWB_TIMEOUT_S) {
+     cmd_vel_msg = {0.0, 0.0, UWB_SEARCH_WZ};  // 原地旋转搜索
+ } else {
+     // 正常 DWA 跟随
+ }
```

**原因**: V1.0/V1.1 在 UWB 信号丢失时发布假位置 (0.7, 0.3)，导致机器狗追不存在的目标。改为超时后原地旋转扫描，信号恢复后自动回归跟随。

**效果**: UWB 丢失 0.2s 后原地以 0.5 rad/s 旋转搜索，信号回来立即恢复。

---

### 变更 12：否决距离增大（417f3e0）

**文件**: `common_types.hpp`

```diff
- constexpr double DWA_EMERGENCY_DIST = 0.25;
+ constexpr double DWA_EMERGENCY_DIST = 0.35;
```

**原因**: 0.25m 否决距离在仍然偏激进，0.35m 给 DWA 更多提前量规划绕行。

**效果**: 障碍物在 0.35m 处即触发轨迹否决，机器人更早开始绕行。

---

### 变更 13：横向平移卡死恢复（417f3e0）

**文件**: `dwa_planner.hpp:71-82`

```diff
+ // 卡死恢复：最优轨迹被否决时，尝试横向移动找空隙
+ if (best.vx < 0.05 && best.vy < 0.05 && std::abs(best.wz) < 0.05) {
+     double left_score  = scoreSample(0.05,  0.15, 0.0, obstacles, ...);
+     double right_score = scoreSample(0.05, -0.15, 0.0, obstacles, ...);
+     if (left_score > best.score)  best = {0.05,  0.15, 0.0, left_score};
+     if (right_score > best.score) best = {0.05, -0.15, 0.0, right_score};
+ }
```

**原因**: 当 DWA 所有轨迹被否决时（前方堵死），V1.0 返回零速不动，V1.1 用旋转找路但效果不好。横向平移保持面向目标方向，更可能找到侧面空隙。

**效果**: 前方被堵时尝试左右侧移，找到缝隙后 DWA 继续规划前进路径。

---

### 变更 14：目标遮罩缩小（d2ee4cc）

**文件**: `common_types.hpp`

```diff
- constexpr double TARGET_MASK_RADIUS = 0.45;
+ constexpr double TARGET_MASK_RADIUS = 0.25;
```

**原因**: 0.45m 遮罩半径过大，当被跟随者紧贴墙壁或柱子时，会将真实障碍物误排除，导致碰撞风险。

**效果**: TARGET_MASK_RADIUS (0.25m) 现在小于 TARGET_RADIUS (0.3m)，只排除人腿部核心区域，贴墙/柱子的障碍物不再被误排。

---

### 变更 15：中远距离速度降低（051f85d）

**文件**: `dwa_planner.hpp:166-167`

```diff
- double dist_factor = std::clamp(
-     (target_dist - FOLLOW_DIST) / (DWA_MAX_TARGET_RANGE - FOLLOW_DIST) * 0.8 + 0.2,
-     0.2, 1.0);
+ double dist_factor = std::clamp(
+     (target_dist - FOLLOW_DIST) / (DWA_MAX_TARGET_RANGE - FOLLOW_DIST) * 0.5 + 0.15,
+     0.15, 1.0);
```

**原因**: 中远距离速度仍然偏快，减速系数从 0.8 降到 0.5，下限从 0.2 降到 0.15。

**效果对比**:

| 目标距离 | V1.1 速度 | V1.2 速度 |
|----------|:---:|:---:|
| 0.6m | 0.09 m/s | 0.07 m/s |
| 1.0m | 0.15 m/s | 0.10 m/s |
| 1.5m | 0.22 m/s | 0.15 m/s |
| 2.0m | 0.30 m/s | 0.20 m/s |
| 3.0m | 0.45 m/s | 0.29 m/s |

---

### 变更 16：UWB 丢失立即停止（当前）

**文件**: `common_types.hpp` + `lidar_tracker.hpp`

**改动 16a — 新增常量**:
```diff
+ constexpr double UWB_STALE_S = 0.4;   // UWB 数据陈旧，停止等待 (秒)
```

**改动 16b — lidar_tracker 三态检测**:
```diff
- if (uwb_elapsed > UWB_TIMEOUT_S) {
-     // 旋转搜索
- } else {
-     // DWA 跟随
- }
+ if (uwb_elapsed > UWB_TIMEOUT_S) {
+     // 超时：旋转搜索
+ } else if (uwb_elapsed > UWB_STALE_S) {
+     // 数据陈旧：停止等待，不追冻结目标
+ } else {
+     // 正常 DWA 跟随
+ }
```

**原因**: V1.2 在 UWB 丢失后 0-0.2s 窗口内，DWA 仍在追冻结在最后位置的目标。如果人已走开，狗会冲向错误方向。增加中间态：UWB 数据陈旧（>0.1s 未更新）时立即停止等待，不再追冻结目标。

**效果**: 
```
UWB 更新 → DWA 跟随 → 0.4s 无数据 → 停止等待 → 0.8s 超时 → 旋转搜索
```

三种状态的 `uwb_elapsed` 阈值：
| 状态 | uwb_elapsed | 行为 |
|------|-------------|------|
| 正常 | ≤ 0.4s | DWA 跟随 |
| 陈旧 | 0.4s ~ 0.8s | 零速等待 |
| 超时 | > 0.8s | 旋转搜索 |

---

---

## 4. V1.3 变更清单（2026-05-13）

### 变更 17：零速豁免碰撞否决 + VETOED 格式化（435a692）

**文件**: `dwa_planner.hpp`

**改动 17a — 零速豁免**:
```diff
+ // 硬否决：碰撞（零速不否决，不动不会撞）
+ bool zero_vel = (std::abs(vx) < 1e-6 && std::abs(vy) < 1e-6 && std::abs(wz) < 1e-6);
+ if (!zero_vel && min_clearance < DWA_EMERGENCY_DIST) {
```

**改动 17b — 全否决检测**:
```diff
+ bool all_vetoed = (best.score < -1e100);
+ if (all_vetoed) {
+     fprintf(stderr, "\n[DWA] ALL SAMPLES VETOED — all %d trajectories blocked\n", ...);
+ }
```

**改动 17c — fmtScore() 辅助函数**:
```diff
+ static std::string fmtScore(double s) {
+     if (s < -1e100) return "VETOED";
+     char buf[32];
+     snprintf(buf, sizeof(buf), "%.4f", s);
+     return std::string(buf);
+ }
```

**原因**: 速度为 0 的轨迹不会产生碰撞，之前的代码把"不动"也否决了，导致 all_vetoed 时连零速都不可选。同时用 `fmtScore()` 统一输出格式，避免 `-DBL_MAX` 显示为巨大数字。

---

### 变更 18：DWA 卡死调试日志（91535cd）

**文件**: `dwa_planner.hpp` + `common_types.hpp`

**改动 18a — scoreSample 增加 debug 模式**:
```diff
+ double scoreSample(double vx, double vy, double wz, ...,
+                    bool debug = false)
```
debug=true 时打印完整评分分解（heading/clearance/density/velocity/target_dist）及否决原因。

**改动 18b — 恢复结果日志**: 左右横向恢复的结果和选中方向打印到 stderr。

**改动 18c — 参数微调**:
- `UWB_STALE_S`: 0.4 → 0.3s
- `UWB_TIMEOUT_S`: 0.8 → 0.5s
- `TARGET_MASK_RADIUS`: 0.25 → 0.3m

**原因**: 卡死时缺乏诊断信息，不知道是哪个评分项导致最优轨迹接近零速。参数微调：UWB 阈值缩短加快丢失响应；目标遮罩扩大减少被跟随者被误判为障碍。

---

### 变更 19：死区过滤 + 后退恢复（03f6311）

**文件**: `dwa_planner.hpp` + `lidar_tracker.hpp`

**改动 19a — agibot 死区过滤**（`lidar_tracker.hpp`）:
```diff
+ // agibot 死区过滤：小指令会被拒，直接置零避免执行打折扣
+ if (std::abs(cmd_vel_msg.linear.x) < 0.05)  cmd_vel_msg.linear.x = 0.0;
+ if (std::abs(cmd_vel_msg.linear.y) < 0.10)  cmd_vel_msg.linear.y = 0.0;
+ if (std::abs(cmd_vel_msg.angular.z) < 0.05) cmd_vel_msg.angular.z = 0.0;
```

**改动 19b — 后退恢复**:
```diff
+ double back_score = scoreSample(-0.15, 0.0, 0.0, obstacles, ...);
+ if (back_ok && back_score > best.score) {
+     best = {-0.15, 0.0, 0.0, back_score};
+ }
```

**原因**: agibot 底盘对小速度指令响应不佳，部分执行导致轨迹变形。死区过滤将低于阈值的指令直接置零。后退恢复在左右都堵死时提供撤退选项。

---

### 变更 20：回退后退恢复（f4b1ee9）

**文件**: `dwa_planner.hpp`

移除变更 19b 的后退恢复，仅保留左右横向恢复。

**原因**: 后退恢复在测试中效果不佳——后退时看不到障碍物更容易撞，且目标在正前方时后退会拉大距离。

---

### 变更 21：多项修复（b02decc）

**文件**: `dwa_planner.hpp` + `lidar_tracker.hpp` + `uwb_serial_pub.cpp`

**改动 21a — 卡死检测修复**:
```diff
- if (best.vx < 0.05 && best.vy < 0.05 && std::abs(best.wz) < 0.05) {
+ if (std::abs(best.vx) < 0.05 && std::abs(best.vy) < 0.05 && std::abs(best.wz) < 0.05) {
```
负向速度（如 DWA 选出 vx=-0.08 后退）之前不被识别为"接近零速"，漏掉卡死检测。

**改动 21b — velocity_score 允许负值**:
```diff
- double velocity_score = std::max(0.0, vel_proj) / DWA_MAX_VX * dist_factor;
+ double velocity_score = vel_proj / DWA_MAX_VX * dist_factor;
```
原先 `max(0.0, vel_proj)` 把后退速度投影评为 0（和静止同分），DWA 无法区分"后退"和"不动"。

**改动 21c — 死区缓存顺序修复**（`lidar_tracker.hpp`）:
```diff
+ // 缓存原始 DWA 速度，供下一帧动态窗口计算（不受死区过滤影响）
+ state_.setVelocity(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);
+
  // agibot 死区过滤
  if (std::abs(cmd_vel_msg.linear.x) < 0.05)  cmd_vel_msg.linear.x = 0.0;
  ...
- // 缓存速度
- state_.setVelocity(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);
```
原先先过滤再缓存，DWA 内部永远收不到小幅加速度，导致速度窗口无法突破死区。改为先缓存再过滤，DWA 内部可以累积速度。

**改动 21d — RSSI 阈值下调**（`uwb_serial_pub.cpp`）:
```diff
- if(RSSI < -77)
+ if(RSSI < -79)
```
信号较弱时仍保留 UWB 数据，减少不必要的丢失。

---

### 变更 22：all_vetoed 覆盖修复（429a4e6）

**文件**: `dwa_planner.hpp`

**改动**:
```diff
+ bool recovery_selected = false;
+
  if (left_ok && left_score > best.score) {
      best = {0.05, 0.15, 0.0, left_score};
+     recovery_selected = true;
  }
  if (right_ok && right_score > best.score) {
      best = {0.05, -0.15, 0.0, right_score};
+     recovery_selected = true;
  }

- // 全部否决时确保返回零速
- if (all_vetoed) {
+ // 全部否决且恢复也失败时才归零
+ if (all_vetoed && !recovery_selected) {
      best = {0.0, 0.0, 0.0, 0.0};
  }
```

**原因**: 所有 525 条轨迹被否决时，横向恢复可能找到有效方向（如右侧 score=0.5514）。但原代码的 `if (all_vetoed)` 无条件将 best 归零，完全覆盖了恢复的选择。这是导致"明明有路但狗不动"的直接 bug。

---

### 变更 23：横向恢复加速 + 趋势否决（7587b71）

**文件**: `dwa_planner.hpp`

**改动 23a — 恢复速度提升**:
```diff
- double left_score  = scoreSample(0.05,  0.15, 0.0, obstacles, ...);
- double right_score = scoreSample(0.05, -0.15, 0.0, obstacles, ...);
+ double left_score  = scoreSample(0.05,  0.25, 0.0, obstacles, ..., false, true);
+ double right_score = scoreSample(0.05, -0.25, 0.0, obstacles, ..., false, true);
```
横向速度从 ±0.15 提升到 ±0.25 m/s，每步位移 0.025m → 0.0375m，1.5s 总横移 0.225m → 0.375m。

**改动 23b — 趋势否决**:
```diff
+ // 恢复轨迹需要趋势判断：记录起点 clearance，只否决持续恶化的轨迹
+ double start_clearance = std::numeric_limits<double>::max();
+ if (recovery) {
+     for (const auto& obs : obstacles) {
+         double d2 = obs.first * obs.first + obs.second * obs.second;
+         double dist = std::sqrt(d2);
+         if (dist < start_clearance) start_clearance = dist;
+     }
+ }
```
```diff
  if (!zero_vel && min_clearance < DWA_EMERGENCY_DIST) {
+     // 恢复轨迹使用趋势否决：clearance 比起点恶化超过 3cm 才否决
+     if (recovery && min_clearance >= start_clearance - 0.03) {
+         continue; // 保持距离或远离中，不否决
+     }
      return -std::numeric_limits<double>::max();
  }
```

**原因**: 机器人本身就在 EMERGENCY_DIST 内（如紧贴墙壁 0.33m < 0.35m）时，任何非零速度的横移轨迹第一步就可能触发否决。趋势否决只否决"离障碍越来越近"的轨迹，允许"保持距离或远离"的侧移。这是解决 BUG 2（被困在 EMERGENCY_DIST 内无法动弹）的关键。

---

### 变更 24：非线性朝向 + 速度对齐 + wz 加速（0cb52de）

**文件**: `dwa_planner.hpp` + `common_types.hpp`

**改动 24a — 朝向评分 cos 非线性化**:
```diff
- double heading_score = 1.0 - angle_error / M_PI;
+ double heading_score = std::cos(angle_error);
```
线性评分对小偏差和大偏差区分度不够。cos 函数在 0° 附近平坦（小偏差无伤大雅），大偏差时陡峭（严重惩罚）。

对比效果：30° → cos=0.866 vs 旧版 0.833（接近）；60° → cos=0.500 vs 旧版 0.667（显著惩罚）；90° → cos=0.0 vs 旧版 0.5（完全否决）。

**改动 24b — 速度对齐因子**:
```diff
+ double heading_alignment = std::abs(std::cos(target_angle));
- double velocity_score = vel_proj / DWA_MAX_VX * dist_factor;
+ double velocity_score = vel_proj / DWA_MAX_VX * dist_factor * heading_alignment;
```
旧版 velocity_score 只考察 vx/vy 在目标方向上的投影，不管机器人面朝哪里。即使机器人侧身对目标（target_angle=90°），vx 照样得满分。乘上 `|cos(target_angle)|` 后，侧身时速度奖励打折，DWA 会优先转身再前进。

**改动 24c — wz 加速度翻倍**:
```diff
- constexpr double DWA_ACC_WZ = 2.0;
+ constexpr double DWA_ACC_WZ = 4.0;             // wz 角加速度 (rad/s²)
```
旧版每帧最多旋转 0.2 rad/s（约 11°/帧），1.5s 后最多转 17°。翻倍后每帧最多 0.4 rad/s，1.5s 最多转 34°，轨迹终点朝向大幅改善。

三项改动协同效果：机器人不再斜着走，而是先转身面朝目标再前进。

---

## 5. V1.4 变更清单（2026-05-19）

### 变更 25：每步朝向评分 + cos² 陡峭化（38a5a6b）

**文件**: `dwa_planner.hpp` + `common_types.hpp`

**改动 25a — 仿真每步累积朝向评分**:
```diff
+ double heading_sum = 0.0;
  for (int k = 0; k < num_steps; ++k) {
      x += ...; y += ...; theta += ...;
+     // 每步朝向评分累积
+     double step_tx = (target_x - x) * cos(theta) + (target_y - y) * sin(theta);
+     double step_ty = -(target_x - x) * sin(theta) + (target_y - y) * cos(theta);
+     double step_angle = abs(atan2(step_ty, step_tx));
+     heading_sum += cos(step_angle) * cos(step_angle);
  }
- double heading_score = cos(angle_error);  // 只看终点
+ double heading_score = heading_sum / num_steps;  // 全程平均
```

**改动 25b — cos² 代替 cos**:
```diff
- double heading_score = std::cos(angle_error);
+ double step_cos = std::cos(step_angle);
+ heading_sum += step_cos * step_cos;
```

**改动 25c — 权重再平衡**:
```diff
- constexpr double DWA_WEIGHT_HEADING = 0.35;
- constexpr double DWA_WEIGHT_VELOCITY = 0.10;
+ constexpr double DWA_WEIGHT_HEADING = 0.44;
+ constexpr double DWA_WEIGHT_VELOCITY = 0.01;
```

**原因**: 旧版朝向评分只看 15 步仿真终点处机器人与目标的角度。DWA 可以"作弊"：选一条前 14 步斜着走、第 15 步才转正的轨迹，评分很高。但实际只执行第 1 步，所以机器人在每一帧都"承诺"下一个 1.5s 会转正，却从不兑现。这就是**斜着走的根本原因**。

改为每步累积平均后，中途斜着走的每一步都会被记录下来，真正面朝目标的轨迹才能得高分。cos² 比 cos 更陡峭：30° → 0.75 vs 0.866，区分度更大。

速度权重从 0.10 降到 0.01：cos² + 每步平均已提供足够区分度，速度分不再需要主导。

---

### 变更 26：局部占据栅格替换目标遮罩（89540c0）

**文件**: `local_occupancy_grid.hpp`（新建） + `lidar_tracker.hpp` + `common_types.hpp`

**改动 26a — 新建占据栅格类**（`local_occupancy_grid.hpp`）:
- 5cm 分辨率，40×40 格 = 2m×2m 覆盖
- 每次占据 +2，每帧衰减 -1
- 同一世界坐标连续出现 3 帧 → 判定为静态障碍
- 基于里程计积分定位栅格原点

**改动 26b — 动静分离逻辑**（`lidar_tracker.hpp`）:
```diff
- // 固定半径遮罩：目标周围 TARGET_MASK_RADIUS 内的点全部排除
- if (dist_to_target < TARGET_MASK_RADIUS) continue;

+ // 占据栅格分类：持续出现的点是静态障碍，短暂出现的是人腿
+ uint8_t cell_count = grid_.getCell(wx, wy);
+ if (cell_count >= STATIC_THRESH) {
+     points.push_back({point_x, point_y});  // 静态障碍 → 送入 DWA
+ } else if (dist_to_target < 0.5) {
+     // 新点靠近目标 → 人腿 → 跳过
+ } else {
+     points.push_back({point_x, point_y});  // 其他新障碍
+ }
+ grid_.occupy(wx, wy);
```

**改动 26c — 里程计积分**:
```diff
+ // 从速度指令积分世界位姿
+ robot_yaw_ += cur_wz * dt;
+ robot_x_ += (cur_vx * cos(robot_yaw_) - cur_vy * sin(robot_yaw_)) * dt;
+ robot_y_ += (cur_vx * sin(robot_yaw_) + cur_vy * cos(robot_yaw_)) * dt;
+ grid_.decay();
+ grid_.setOrigin(robot_x_, robot_y_);
```

**原因**: 固定半径遮罩（TARGET_MASK_RADIUS=0.3m）有盲区——当被跟随者贴着墙壁或柱子时，墙/柱的 LiDAR 点也落在遮罩内被排除，导致 DWA 看不见真实障碍物。这是**方案 D**（时间维度动静分离）的实现。

占据栅格利用"人的腿在动、墙/柱不动"的特性：在同一个世界坐标持续出现的点判定为静态障碍物，短暂出现的点（如人腿经过）判定为动态目标。彻底解决了遮罩盲区问题。

**参数**:
| 参数 | 值 | 说明 |
|------|-----|------|
| RESOLUTION | 0.05m | 栅格分辨率 |
| SIZE | 40 | 40×40 格 = 2m×2m |
| OCCUPY_INCREMENT | 2 | 每次占据 +2 |
| STATIC_THRESH | 3 | 连续出现 3 帧判定为静态 |
| 衰减 | -1/帧 | 不再出现的点逐渐归零 |

---

### 变更 27：两阶段控制 — PD 直走 + DWA 避障（95796d4）

**文件**: `dwa_planner.hpp` + `common_types.hpp`

**改动 27a — 前方走廊检查**:
```diff
+ // 检查目标方向矩形走廊 (0.5m宽 × 1.2m长) 内是否有障碍物
+ bool isCorridorClear(obstacles, target_x, target_y, target_dist) {
+     for (auto& obs : obstacles) {
+         double proj = (ox*tx + oy*ty) / target_dist;
+         if (proj < 0 || proj > min(target_dist, 1.2)) continue;
+         double perp = abs(ox*ty - oy*tx) / target_dist;
+         if (perp < 0.25) return false;
+     }
+     return true;
+ }
```

**改动 27b — PD 朝向控制器**:
```diff
+ if (isCorridorClear(...)) {
+     double heading_error = atan2(target_y, target_x);
+     double diff = heading_error - prev_heading_error_;  // 归一化 ±π
+     double heading_error_rate = diff / 0.1;
+     prev_heading_error_ = heading_error;
+
+     // 死区 3°: 小角度不转
+     if (abs(heading_error) < 0.05) wz = 0;
+     else wz = clamp(1.0 * heading_error + 0.3 * heading_error_rate, ...);
+
+     vx = DWA_MAX_VX * cos(heading_error) * dist_factor;
+     return {vx, 0.0, wz, 1.0};  // vy=0, 不横移
+ }
```

**改动 27c — DWA 参数微调**:
```diff
- constexpr double DWA_ACC_WZ = 4.0;
- constexpr double DWA_WEIGHT_HEADING = 0.44;
- constexpr double DWA_WEIGHT_TARGET_DIST = 0.15;
+ constexpr double DWA_ACC_WZ = 2.5;
+ constexpr double DWA_WEIGHT_HEADING = 0.40;
+ constexpr double DWA_WEIGHT_TARGET_DIST = 0.19;
```

**两阶段控制逻辑**:
```
每帧:
  if (目标方向 0.5m×1.2m 走廊内无障碍):
    → PD 控制器: wz = Kp*err + Kd*err_rate, vy=0
    → 直直地朝目标走，不斜行
  else:
    → 完整 DWA 采样评分
    → 避障模式，允许 vy/wz 绕行
```

**原因**: 之前的纯 DWA 加权求和评分中，各评分项可以互相"买分"——朝向不好用 vy 快速侧移补回来，导致狗斜着走。即使反复调整权重也无法根除（20° 偏差下 heading 扣 0.25 分，但 vy 速度分可以补回 0.3 分）。

两阶段控制从根本上分离"直走"和"避障"：无障碍时不需要 DWA 做多目标权衡——直接 P 控制转正 + 直走即可。障碍出现时才启用 DWA 绕行。

**PD 控制器参数**:
| 参数 | 值 | 说明 |
|------|-----|------|
| KP_HEADING | 1.0 | P 增益，决定转正速度 |
| KD_HEADING | 0.3 | D 增益，阻尼振荡（"刹车"效果） |
| HEADING_DEADBAND | 0.05 rad (3°) | 死区，防止小误差抖动 |
| CORRIDOR_HALF_WIDTH | 0.25m | 走廊半宽 |
| CORRIDOR_MAX_LENGTH | 1.2m | 走廊最大检查距离 |

**D 项的作用**: 当机器人快速转正时，heading_error 在减小（负变化率），D 项产生反向力矩提前减速。例如 heading_error 从 10°→5° 时，P=0.087, D=-0.22，wz 在到达 0 之前就开始减小，不再过冲振荡。

---

## 6. 已知问题与后续改进方向

| 问题 | 现状 | 改进方向 |
|------|------|---------|
| 太近时不会后退 | velocity_score 允许负值但不激励后退 | 目标过近时增加后退奖励 |
| 点模型碰撞检测 | 从中心算距离，忽略机器人宽度 | 考虑椭圆模型（机身 0.7m×0.35m） |
| 动态窗口受加速度限制 | 恢复轨迹 vy 受限于 DWA_MAX_VY=0.3 | 恢复时可放宽速度限制 |
| 无 AOA 角度区分 | UWB 只给距离，不知道人在左/右 | 利用 AOA 提前调整朝向 |
| 里程计漂移 | 速度积分位姿无外部校正 | 长时间运行栅格原点可能偏移 |

---

## 7. DWA 使用说明

### 7.1 参数调优

所有可调参数在 `common_types.hpp` 和 `dwa_planner.hpp` 中，重新编译生效：

| 参数 | 默认值 | 调大/调小的效果 |
|------|--------|----------------|
| `DWA_MAX_VX` | 0.45 | ↑ 更快速 / ↓ 更平稳 |
| `DWA_EMERGENCY_DIST` | 0.35 | ↑ 更保守(保持距离) / ↓ 更激进 |
| `DWA_SAFE_DIST` | 0.5 | ↑ 更早触发绕行 / ↓ 更晚绕行 |
| `DWA_WEIGHT_HEADING` | 0.40 | ↑ 更积极面朝目标 / ↓ 更注重侧面移动 |
| `DWA_WEIGHT_CLEARANCE` | 0.40 | ↑ 更保守避障 / ↓ 更激进贴近障碍 |
| `DWA_WEIGHT_VELOCITY` | 0.01 | ↑ 更快速 / ↓ 更慢 |
| `DWA_WEIGHT_TARGET_DIST` | 0.19 | ↑ 更精确保持跟随距离 / ↓ 距离容忍更大 |
| `FOLLOW_DIST` | 0.6 | ↑ 跟得更远 / ↓ 跟得更近 |
| `DWA_ACC_WZ` | 2.5 | ↑ 转弯更快 / ↓ 转弯更平缓 |
| `KP_HEADING` | 1.0 | ↑ 直走模式转正更快 / ↓ 更平缓 |
| `KD_HEADING` | 0.3 | ↑ 阻尼更强 / ↓ 可能过冲 |
| `CORRIDOR_MAX_LENGTH` | 1.2 | ↑ 更早切避障 / ↓ 更晚切避障 |

### 7.2 常见场景调优

**场景 1: 狭窄走廊，机器狗贴墙走**
```
调大 DWA_WEIGHT_CLEARANCE (0.40 → 0.50)
调大 DWA_EMERGENCY_DIST (0.35 → 0.45)
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
调小 TARGET_MASK_RADIUS (0.30 → 0.20) — 避免把墙也排除
```

### 7.3 调试技巧

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

5. **DWA 调试日志**: 卡死时会自动输出评分分解到 stderr，包含 heading/clearance/density/velocity/target_dist 各项得分和否决原因。出现异常时查看终端输出定位问题。
