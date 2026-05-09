/**
 * @file common_types.hpp
 * @brief 公共类型定义和常量
 */

#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <functional>

// ----- 常量定义 -----
constexpr double FOLLOW_DIST = 0.6;           // 机器人与目标的预设距离 (米)
constexpr double TARGET_RADIUS = 0.3;         // 目标搜索半径 (米)
constexpr double LINEAR_SCALE_FACTOR = 0.5;   // 前后运动速度比例系数
constexpr double ANGULAR_SCALE_FACTOR = 1.0;  // 旋转运动速度比例系数
constexpr double LINEAR_Y_SCALE_FACTOR = 1.0; // 左右运动速度比例系数
constexpr double RECTANGLE_WIDTH = 0.35;      // 矩形宽度 (米)

// 目标遮罩 — UWB目标周围的LiDAR点视为被跟随者，不作为障碍物
constexpr double TARGET_MASK_RADIUS = 0.25;    // 目标周围排除半径 (米)

// UWB 信号丢失搜索
constexpr double UWB_TIMEOUT_S = 0.2;           // UWB 超时进入搜索 (秒)
constexpr double UWB_SEARCH_WZ = 0.5;           // 搜索旋转速度 (rad/s)

// 速度限制
constexpr double MAX_LINEAR_SPEED = 1.0;
constexpr double MAX_ANGULAR_SPEED = 1.0;

// DWA (Dynamic Window Approach) 避障参数
constexpr double DWA_SIM_TIME = 1.5;           // 前向仿真时长 (s)
constexpr double DWA_DT = 0.1;                 // 仿真步长 (s)
constexpr int DWA_VX_SAMPLES = 15;             // vx 采样数
constexpr int DWA_VY_SAMPLES = 5;              // vy 采样数
constexpr int DWA_WZ_SAMPLES = 7;              // wz 采样数

// DWA 速度约束
constexpr double DWA_MIN_VX = -0.3;            // vx 下限 (m/s)
constexpr double DWA_MAX_VX = 0.45;            // vx 上限 (m/s)
constexpr double DWA_MIN_VY = -0.3;            // vy 下限 (m/s)
constexpr double DWA_MAX_VY = 0.3;             // vy 上限 (m/s)
constexpr double DWA_MIN_WZ = -1.0;            // wz 下限 (rad/s)
constexpr double DWA_MAX_WZ = 1.0;             // wz 上限 (rad/s)

// DWA 加速度约束
constexpr double DWA_ACC_VX = 0.5;             // vx 加速度 (m/s²)
constexpr double DWA_ACC_VY = 0.5;             // vy 加速度 (m/s²)
constexpr double DWA_ACC_WZ = 2.0;             // wz 角加速度 (rad/s²)

// DWA 评分权重
constexpr double DWA_WEIGHT_HEADING = 0.35;     // 朝向权重
constexpr double DWA_WEIGHT_CLEARANCE = 0.40;   // 安全距离权重
constexpr double DWA_WEIGHT_VELOCITY = 0.10;    // 速度权重
constexpr double DWA_WEIGHT_TARGET_DIST = 0.15; // 目标距离权重

// DWA 安全阈值
constexpr double DWA_EMERGENCY_DIST = 0.35;     // 否决距离 (m)
constexpr double DWA_SAFE_DIST = 0.5;           // 安全距离 (m)
constexpr double DWA_MAX_TARGET_RANGE = 3.0;    // 最大目标距离 (m)

// 机器人框架排除区域（雷达可能扫描到的内部支架）
constexpr double ROBOT_FRAME_FRONT = 0.15;    // 前方排除范围 (米)
constexpr double ROBOT_FRAME_BACK = 0.35;     // 后方排除范围 (米)
constexpr double ROBOT_FRAME_LEFT = 0.15;     // 左侧排除范围 (米)
constexpr double ROBOT_FRAME_RIGHT = 0.15;    // 右侧排除范围 (米)

// OpenCV可视化参数（调试模式）
constexpr int WINDOW_SIZE = 800;
constexpr int SPEED_DISPLAY_HEIGHT = 150;
constexpr int TOTAL_WINDOW_HEIGHT = WINDOW_SIZE + SPEED_DISPLAY_HEIGHT;
constexpr double DISPLAY_WORLD_SIZE_M = 4.0;
constexpr double METERS_TO_PIXELS = WINDOW_SIZE / DISPLAY_WORLD_SIZE_M;
constexpr double ROBOT_Y_OFFSET_M = 0.5;

constexpr int SPEED_BAR_LENGTH = 50;
constexpr int SPEED_ARC_RADIUS = 40;

// 网络端口
constexpr int UDP_SEND_PORT = 8888;
constexpr int UDP_RECV_PORT = 8889;
constexpr int HTTP_PORT = 8080;
constexpr int WS_PORT = 8890;

/**
 * @brief 控制模式枚举
 */
enum ControlMode {
    MODE_DIRECT = 0,  // 直接控制模式
    MODE_FOLLOW = 1,  // 跟随模式
    MODE_NAV = 2      // 导航模式
};

/**
 * @brief 速度指令结构体
 */
struct VelocityCmd {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
};

/**
 * @brief 共享状态 - 各模块间共享的数据
 */
struct SharedState {
    // 目标位置 (需要保护)
    std::mutex target_mutex;
    double target_x = FOLLOW_DIST;
    double target_y = 0.0;
    
    // 速度指令缓存 (需要保护)
    std::mutex velocity_mutex;
    double cached_vx = 0.0;
    double cached_vy = 0.0;
    double cached_wz = 0.0;
    
    // 雷达点云缓存 (需要保护)
    std::mutex scan_data_mutex;
    std::vector<std::pair<double, double>> cached_points;
    
    // 直接控制指令 (需要保护)
    std::mutex direct_cmd_mutex;
    double direct_vx = 0.0;
    double direct_vy = 0.0;
    double direct_wz = 0.0;
    
    // 原子状态
    std::atomic<bool> active{false};
    std::atomic<bool> is_moving_enabled{false};
    std::atomic<int> control_mode{MODE_FOLLOW};
    
    // 获取目标位置
    void getTarget(double& x, double& y) {
        std::lock_guard<std::mutex> lock(target_mutex);
        x = target_x;
        y = target_y;
    }
    
    // 设置目标位置
    void setTarget(double x, double y) {
        std::lock_guard<std::mutex> lock(target_mutex);
        target_x = x;
        target_y = y;
    }
    
    // 获取速度缓存
    void getVelocity(double& vx, double& vy, double& wz) {
        std::lock_guard<std::mutex> lock(velocity_mutex);
        vx = cached_vx;
        vy = cached_vy;
        wz = cached_wz;
    }
    
    // 设置速度缓存
    void setVelocity(double vx, double vy, double wz) {
        std::lock_guard<std::mutex> lock(velocity_mutex);
        cached_vx = vx;
        cached_vy = vy;
        cached_wz = wz;
    }
    
    // 获取直接控制指令
    void getDirectCmd(double& vx, double& vy, double& wz) {
        std::lock_guard<std::mutex> lock(direct_cmd_mutex);
        vx = direct_vx;
        vy = direct_vy;
        wz = direct_wz;
    }
    
    // 设置直接控制指令
    void setDirectCmd(double vx, double vy, double wz) {
        std::lock_guard<std::mutex> lock(direct_cmd_mutex);
        direct_vx = vx;
        direct_vy = vy;
        direct_wz = wz;
    }
    
    // 获取点云数据
    std::vector<std::pair<double, double>> getPoints() {
        std::lock_guard<std::mutex> lock(scan_data_mutex);
        return cached_points;
    }
    
    // 设置点云数据
    void setPoints(std::vector<std::pair<double, double>>&& points) {
        std::lock_guard<std::mutex> lock(scan_data_mutex);
        cached_points = std::move(points);
    }
};

/**
 * @brief 简易JSON解析工具
 */
class JsonParser {
public:
    // 从JSON中提取数值
    static double extractNumber(const std::string& str, const std::string& key) {
        size_t pos = str.find("\"" + key + "\"");
        if (pos == std::string::npos) return 0;
        pos = str.find(":", pos);
        if (pos == std::string::npos) return 0;
        pos++;
        while (pos < str.size() && (str[pos] == ' ' || str[pos] == '\t')) pos++;
        size_t end = pos;
        while (end < str.size() && (std::isdigit(str[end]) || str[end] == '.' || str[end] == '-')) end++;
        if (end > pos) {
            return std::stod(str.substr(pos, end - pos));
        }
        return 0;
    }
    
    // 解析x, y坐标
    static bool parseXY(const std::string& json, double& x, double& y) {
        x = extractNumber(json, "x");
        y = extractNumber(json, "y");
        return true;
    }
    
    // 检查消息类型
    static bool hasType(const std::string& json, const std::string& type) {
        return json.find("\"type\":\"" + type + "\"") != std::string::npos ||
               json.find("\"type\": \"" + type + "\"") != std::string::npos;
    }
    
    // 检查布尔值
    static bool getBool(const std::string& json, const std::string& key) {
        return json.find("\"" + key + "\":true") != std::string::npos ||
               json.find("\"" + key + "\": true") != std::string::npos;
    }
    // 提取字符串值
    static std::string extractString(const std::string& str, const std::string& key) {
        size_t pos = str.find("\"" + key + "\"");
        if (pos == std::string::npos) return "";
        pos = str.find(":", pos);
        if (pos == std::string::npos) return "";
        pos = str.find("\"", pos);
        if (pos == std::string::npos) return "";
        size_t start = pos + 1;
        size_t end = str.find("\"", start);
        if (end == std::string::npos) return "";
        return str.substr(start, end - start);
    }
};

#endif // COMMON_TYPES_HPP
