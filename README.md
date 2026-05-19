# robot_follow — 机器狗 UWB 跟随系统

基于 ROS2 Humble 的四足机器人自主跟随系统。通过 UWB 定位目标位置，LiDAR 感知障碍物，DWA (Dynamic Window Approach) 局部规划器实时避障跟随。

## 目录结构

```
├── robot_follow/          # 核心跟随包
│   ├── include/           # 头文件
│   │   ├── common_types.hpp        # 公共常量和共享状态
│   │   ├── dwa_planner.hpp        # DWA 局部避障规划器（两阶段控制）
│   │   ├── lidar_tracker.hpp      # LiDAR 追踪 + 目标融合
│   │   ├── local_occupancy_grid.hpp # 局部占据栅格（动静分离）
│   │   ├── kalman_filter.hpp      # 卡尔曼滤波器
│   │   ├── direct_control.hpp     # 直接控制模式
│   │   ├── android_comm.hpp       # Android 通信
│   │   ├── web_comm.hpp           # Web 通信
│   │   ├── web_server.hpp         # Web 服务器
│   │   └── httplib.h              # 第三方 HTTP/WebSocket 库
│   ├── src/               # 源文件
│   │   ├── robot_nexus.cpp        # 主入口（控制模式调度）
│   │   ├── android_comm.cpp       # Android 通信节点
│   │   ├── web_comm.cpp           # Web 通信节点
│   │   └── keyboard_cmd.cpp       # 键盘控制节点
│   ├── launch/            # 启动文件
│   │   └── start.launch.py        # 主启动文件
│   ├── docs/              # 设计文档
│   │   ├── dwa-planner-changelog.md  # DWA 版本变更日志
│   │   ├── dwa-planner-design.md     # DWA 设计文档
│   │   └── apf-vortex-avoidance-design.md  # APF 避障设计（历史）
│   ├── msg/               # 自定义 ROS 消息
│   ├── CMakeLists.txt
│   └── package.xml
├── agibot/                # 机器人底盘驱动
├── lidar_pkg/             # LiDAR 驱动
├── uwb_serial_pub/        # UWB 串口发布节点
├── CLAUDE.md              # Claude Code 开发指引
└── README.md
```

## 依赖

- ROS2 Humble
- OpenCV (可视化调试)
- 硬件: LD19 LiDAR, UWB 串口模块, agibot 四足底盘

## 编译

```bash
# 初始化 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译所有包
colcon build --symlink-install

# 或只编译核心包
colcon build --symlink-install --packages-select robot_follow

# 加载编译产物
source install/setup.bash
```

`--symlink-install` 让安装目录指向源码，修改解释型文件（Python/launch）无需重新编译。

## 运行

```bash
source install/setup.bash
ros2 launch robot_follow start.launch.py
```

启动后系统进入 **跟随模式** (MODE_FOLLOW)，机器狗自动追踪 UWB 标签。

### 控制模式

| 模式 | 说明 |
|------|------|
| MODE_DIRECT (0) | 直接遥控 |
| MODE_FOLLOW (1) | UWB 跟随（默认） |
| MODE_NAV (2) | 导航模式 |

Web 界面 `http://<机器人IP>:8080` 可切换模式和查看雷达点云。

## 调试

### OpenCV 可视化

launch 文件中设 `enable_opencv: true` 打开可视化窗口：

- 红色点 — 否决距离内障碍
- 橙色点 — 安全距离内障碍
- 灰色点 — 远处障碍
- 紫色圆圈 — UWB 目标
- 绿色线 — 机器人到目标连线
- 底部 — 实时速度 vx/vy/wz

### DWA 调试日志

机器人卡死时自动输出评分分解到 stderr：

```
========== DWA Stuck Debug ==========
Target: dist=1.50m angle=10.5° | Obstacles: 12
Sample: vx=0.100 vy=0.050 wz=0.200
--- Score breakdown (raw -> weighted) ---
heading:       0.850 ->  0.340  (w=0.40)
clearance:     0.920 ->  0.258  (w=0.40*0.7)
density:       0.780 ->  0.094  (w=0.40*0.3)
velocity:      0.150 ->  0.002  (w=0.01)
target_dist:   0.600 ->  0.114  (w=0.19)
TOTAL SCORE: 0.8078
======================================
```

### 查看实时速度指令

```bash
ros2 topic echo /cmd_vel
```

### 查看 UWB 数据

```bash
ros2 topic echo /uwb_data
```

## DWA 避障简介

V1.4 采用两阶段控制：

1. **无障碍走廊** → PD 控制器直走（vy=0, 不斜行）
2. **走廊有障碍** → 完整 DWA 采样评分绕行

走廊范围：目标方向 0.5m × 1.2m 矩形区域。参数定义在 `common_types.hpp` 和 `dwa_planner.hpp`。

## 已知问题

| 问题 | 改进方向 |
|------|---------|
| 太近时不会后退 | 目标过近时增加后退激励 |
| 点模型碰撞 | 考虑椭圆机身模型 |
| 里程计漂移 | 栅格原点无外部校正 |

详细版本演进参见 `docs/dwa-planner-changelog.md`。
