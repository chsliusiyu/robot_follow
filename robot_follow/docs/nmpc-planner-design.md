# NMPC (Nonlinear Model Predictive Control) 局部避障规划器 — 软件设计文档

## 1. 概述

### 1.1 目的

用**梯度下降非线性模型预测控制（NMPC）**完全替换原始 APF（人工势场法）避障模块。通过在线优化控制序列，使机器狗在跟随目标的同时平滑绕开障碍物。

### 1.2 设计目标

| 目标 | 指标 |
|------|------|
| 避障成功率 | 障碍物密度 ≤ 0.5/m² 时 ≥ 95% |
| 轨迹平滑性 | 速度跳变 ≤ 0.3 m/s² |
| 实时性 | 单次求解 ≤ 30 ms |
| 最小安全距离 | 与障碍物 ≥ 0.15 m |
| 代码自包含 | 0 外部依赖 |

### 1.3 与原始 APF 的关键区别

| 维度 | APF | NMPC |
|------|-----|------|
| 原理 | 力矢量合成（引力+斥力） | 控制序列数值优化 |
| 前视能力 | 0（仅当前时刻） | 1.5s 闭环优化 |
| 障碍建模 | 点斥力（易局部极小） | 光滑指数惩罚场 |
| 轨迹平滑 | 差（可能跳变） | 好（显式平滑代价） |
| 控制约束 | 后验 clamp | 优化内投影 |

---

## 2. 系统模型

### 2.1 运动学模型

以当前机器人位置为原点，状态 z = [x, y, θ]^T 为机器人系中的相对位姿，控制 u = [vx, vy, wz]^T 为全向速度指令。

```
x_{k+1} = x_k + (vx_k · cos(θ_k) - vy_k · sin(θ_k)) · dt
y_{k+1} = y_k + (vx_k · sin(θ_k) + vy_k · cos(θ_k)) · dt
θ_{k+1} = θ_k + wz_k · dt
```

参数：dt = 0.1s, N = 15 步（1.5s 前视时域）

### 2.2 雅可比矩阵

**∂f/∂z**（3×3）：

```
∂f/∂z = [1, 0, -(vx·sinθ + vy·cosθ)·dt]
         [0, 1,  (vx·cosθ - vy·sinθ)·dt]
         [0, 0,   1                     ]
```

**∂f/∂u**（3×3）：

```
∂f/∂u = [cosθ·dt, -sinθ·dt, 0]
         [sinθ·dt,  cosθ·dt, 0]
         [0,        0,       dt]
```

---

## 3. 优化问题建模

### 3.1 目标函数

```
min_U  J(U) = φ(z_N) + Σ_{k=0}^{N-1} L(z_k, u_k)

s.t.  z_{k+1} = f(z_k, u_k),  z_0 = [0, 0, 0]^T
      u_k ∈ U_feasible
```

### 3.2 终端代价 φ(z_N)

评估轨迹终点与目标的偏差（在终点姿态坐标系下表达）：

```
target_end_x = (target_x - x_N)·cos(θ_N) + (target_y - y_N)·sin(θ_N)
target_end_y = -(target_x - x_N)·sin(θ_N) + (target_y - y_N)·cos(θ_N)

dist_error = √(target_end_x² + target_end_y²) - FOLLOW_DIST
angle_error = |atan2(target_end_y, target_end_x)|

φ = w_track · dist_error² + w_head · angle_error²
```

- w_track = 5.0：位置跟踪权重
- w_head = 3.0：朝向收敛权重

### 3.3 阶段代价 L(z_k, u_k)

#### 障碍物排斥（光滑指数惩罚场）

```
d_i(z_k) = √((x_k - ox_i)² + (y_k - oy_i)²)

L_obs(z_k) = Σ_i exp(-d_i(z_k)² / σ²)
```

- σ = 0.15m：指数衰减宽度（控制障碍影响范围平滑度）
- 影响范围截断：d_i > 0.5m 的点不参与计算
- 权重 w_obs = 10.0

**梯度**（解析）：

```
∂L_obs/∂x_k = Σ_i (-2/σ²)·(x_k - ox_i)·exp(-d_i²/σ²)
∂L_obs/∂y_k = Σ_i (-2/σ²)·(y_k - oy_i)·exp(-d_i²/σ²)
∂L_obs/∂θ_k = 0
```

#### 控制平滑

```
L_smooth = ||u_k - u_{k-1}||²  （u_{-1} = 当前实际速度）
```

权重 w_smooth = 2.0，相邻步控制变化惩罚。

#### 控制代价

```
L_ctrl = ||u_k||²
```

权重 w_ctrl = 0.1（小量，仅避免不必要的大速度）。

### 3.4 约束

| 约束 | 最小值 | 最大值 |
|------|--------|--------|
| vx | -0.3 m/s | 1.0 m/s |
| vy | -0.3 m/s | 0.3 m/s |
| wz | -1.0 rad/s | 1.0 rad/s |
| ax | - | 1.0 m/s² |
| ay | - | 0.5 m/s² |
| αz | - | 2.0 rad/s² |

紧急否决：若任意轨迹点距离障碍物 < 0.12m，该控制序列代价 = ∞。

---

## 4. 优化算法

### 4.1 梯度计算：伴随法

**前向传播**：从 z_0 = [0,0,0]^T 仿真 N 步，记录全部状态 z_k。

**反向传播**：

```
λ_N = ∂φ/∂z_N  （终端代价对终态的梯度）

For k = N-1 → 0:
    λ_k = ∂L/∂z_k + (∂f/∂z_k)^T · λ_{k+1}   （伴随状态递推）
    g_k = ∂L/∂u_k + (∂f/∂u_k)^T · λ_{k+1}   （控制梯度）
```

终端状态梯度 ∂φ/∂z_N 通过链式法则从 target_end 坐标系计算。

平滑代价梯度：∂L_smooth/∂u_k 耦合相邻步：
- k = 0: ∂/∂u₀ = 2·(u₀ - u_{-1})
- 0 < k < N-1: ∂/∂u_k = 2·(u_k - u_{k-1}) - 2·(u_{k+1} - u_k)
- k = N-1: ∂/∂u_{N-1} = 2·(u_{N-1} - u_{N-2})

### 4.2 优化循环

```
1. Warm Start:
   - 有历史解：prev_U 左移一步，尾部重复
   - 无历史解：全部初始化为当前速度

2. For iter = 1..30:
   a. 前向仿真 → 轨迹代价
   b. 检测紧急距离（否决不可行解）
   c. 存储最优解
   d. 伴随反向传播 → 控制梯度 G
   e. U_new = U - step · G
   f. 投影到可行域（盒约束 + 加速度约束）
   g. 梯度范数 < 1e-3 → 收敛退出
   h. 步长衰减：step = max(min_step, step · 0.98)

3. 保存 warm start，返回 u₀
```

### 4.3 约束投影

**盒投影**：逐元素 clamp 到速度上下界。

**加速度投影**（递归）：

```
u₀ = clamp(u₀, current - acc·dt, current + acc·dt)
For k = 1..N-1:
    u_k = clamp(u_k, u_{k-1} - acc·dt, u_{k-1} + acc·dt)
```

---

## 5. 参数表

| 参数 | 符号 | 值 | 说明 |
|------|------|-----|------|
| 预测步数 | N | 15 | 1.5s 前视 |
| 仿真步长 | dt | 0.1 s | |
| 最大迭代 | MAX_ITER | 30 | |
| 收敛容差 | ε | 1e-3 | 梯度范数 |
| 初始步长 | α₀ | 0.02 | |
| 最小步长 | α_min | 1e-4 | |
| 跟踪权重 | w_track | 5.0 | 终端位置误差 |
| 朝向权重 | w_head | 3.0 | 终端朝向误差 |
| 障碍权重 | w_obs | 10.0 | 阶段障碍惩罚 |
| 平滑权重 | w_smooth | 2.0 | 控制序列平滑 |
| 控制权重 | w_ctrl | 0.1 | 控制代价 |
| 障碍 sigma | σ | 0.15 m | 指数衰减宽度 |
| 障碍截断 | r_cut | 0.5 m | 影响范围 |
| 紧急距离 | d_em | 0.12 m | 否决距离 |

---

## 6. 模块接口

### 6.1 NMPCPlanner 类

```cpp
class NMPCPlanner {
public:
    struct Control { double vx, vy, wz; };

    Control solve(const std::vector<std::pair<double, double>>& obstacles,
                  double target_x, double target_y,
                  double cur_vx, double cur_vy, double cur_wz);
};
```

| 参数 | 说明 |
|------|------|
| obstacles | 障碍点云（机器人坐标系，已过滤自身框架） |
| target_x, target_y | 目标位置（机器人坐标系） |
| cur_vx, cur_vy, cur_wz | 当前机器人速度 |
| 返回值 | 最优第一步速度指令 |

### 6.2 LidarTracker 集成

```cpp
// 跟随模式速度计算
double cur_vx, cur_vy, cur_wz;
state_.getVelocity(cur_vx, cur_vy, cur_wz);

auto obstacles = state_.getPoints();
auto result = nmpc_planner_.solve(obstacles, target_x, target_y,
                                   cur_vx, cur_vy, cur_wz);
cmd_vel_msg.linear.x = result.vx;
cmd_vel_msg.linear.y = result.vy;
cmd_vel_msg.angular.z = result.wz;
```

### 6.3 数据流

```
LaserScan → LidarTracker::processScan()
  ├─ 过滤 inf/nan 点
  ├─ 排除机器人框架区域
  ├─ 存储点云到 SharedState
  ├─ NMPCPlanner::solve() → 最优速度
  ├─ 发布 /cmd_vel
  └─ 广播点云 (Web 可视化)
```

---

## 7. 参数调优指南

| 场景 | 调整方向 |
|------|---------|
| 避障过于激进（远离障碍物过多） | 降低 w_obs 或增大 σ |
| 避障不足（靠近障碍物） | 提高 w_obs 或降低 σ |
| 速度震荡 | 提高 w_smooth 或降低步长 |
| 跟不紧目标 | 提高 w_track |
| 收敛慢 | 提高初始步长或降低 N |
| 计算超时 | 降低 N 或 MAX_ITER |

---

## 8. 局限性与后续改进

1. **局部最优**：梯度下降可能陷入局部极小点，Warm start 策略缓解但非完全解决
2. **动态障碍**：未显式建模障碍物速度
3. **一次规划**：仅取 u₀ 执行，不进行滚动时域闭环
4. **参数敏感**：20 个参数需针对场景调优
5. **后续方向**：可探索 SQP 方法提升约束处理精度；引入障碍物运动预测
