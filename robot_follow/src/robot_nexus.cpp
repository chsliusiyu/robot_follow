/**
 * @file robot_nexus.cpp
 * @brief ROS2机器人中枢控制节点
 * 
 * 功能：
 * - 订阅激光雷达数据，追踪目标
 * - 通过UDP发送雷达数据到Android App
 * - 接收Android App的控制指令
 * - Web可视化显示
 * - 可选的OpenCV可视化显示（调试用）
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <chrono>

#include "common_types.hpp"
#include "lidar_tracker.hpp"
#include "direct_control.hpp"
#include "web_comm.hpp"
#include "android_comm.hpp"

/**
 * @class RobotNexusNode
 * @brief 机器人中枢控制ROS2节点
 */
class RobotNexusNode : public rclcpp::Node
{
public:
    RobotNexusNode() 
        : Node("robot_nexus"),
          lidar_tracker_(shared_state_),
          direct_controller_(shared_state_),
          web_comm_(shared_state_),
          android_comm_(shared_state_)
    {
        // 声明参数
        this->declare_parameter<bool>("active", false);
        this->declare_parameter<bool>("enable_opencv", false);
        this->declare_parameter<bool>("enable_web", true);
        this->declare_parameter<bool>("enable_kalman", false);
        this->declare_parameter<std::string>("web_root", "");
        
        // 获取参数
        shared_state_.active.store(this->get_parameter("active").as_bool());
        bool enable_opencv = this->get_parameter("enable_opencv").as_bool();
        bool enable_web = this->get_parameter("enable_web").as_bool();
        bool enable_kalman = this->get_parameter("enable_kalman").as_bool();
        std::string web_root = this->get_parameter("web_root").as_string();
        
        RCLCPP_INFO(this->get_logger(), "节点启动 - 跟随: %s, OpenCV: %s, Web: %s, 卡尔曼: %s",
                    shared_state_.active.load() ? "开启" : "关闭",
                    enable_opencv ? "开启" : "关闭",
                    enable_web ? "开启" : "关闭",
                    enable_kalman ? "开启" : "关闭");
        
        // 初始化发布者和订阅者
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>("topic", 10, 
            std::bind(&RobotNexusNode::topic_callback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        action_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/d1_cmd", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1,
            std::bind(&RobotNexusNode::scanCallback, this, std::placeholders::_1)
        );
        
        // 设置日志回调
        auto log_cb = [this](const std::string& msg) {
            RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        };
        
        // 设置速度发布回调
        auto vel_cb = [this](const geometry_msgs::msg::Twist& cmd) {
            cmd_vel_pub_->publish(cmd);
        };
        
        // 配置雷达追踪器
        lidar_tracker_.setOpenCVEnabled(enable_opencv);
        lidar_tracker_.setKalmanEnabled(enable_kalman);
        lidar_tracker_.setVelocityCallback(vel_cb);
        lidar_tracker_.setDataBroadcastCallback([this]() {
            android_comm_.sendScanData();
            web_comm_.broadcastData();
        });
        
        // 配置直接控制器
        direct_controller_.setVelocityCallback(vel_cb);
        
        // 配置Web通讯
        if (enable_web) {
            web_comm_.setLogCallback(log_cb);
            web_comm_.setWebRoot(web_root);
            web_comm_.autoDetectWebRoot({
                "./web",
                "../share/robot_follow/web"
            });
            web_comm_.setDirectCmdCallback([this](double x, double y, double z) {
                direct_controller_.processDirectCmd(x, y, z);
                RCLCPP_INFO(this->get_logger(), "收到direct_cmd: vx=%.2f, vy=%.2f, wz=%.2f", x, y, z);
            });
            web_comm_.setActionCmdCallback([this](const std::string& action) {
                std_msgs::msg::String msg;
                msg.data = action;
                action_cmd_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "发布动作指令: %s", action.c_str());
                
                if (action == "liedown") {
                    std::thread([this]() {
                        std::this_thread::sleep_for(std::chrono::seconds(5));
                        std_msgs::msg::String passive_msg;
                        passive_msg.data = "passive";
                        if (rclcpp::ok()) {
                            action_cmd_pub_->publish(passive_msg);
                            RCLCPP_INFO(this->get_logger(), "延迟发布动作指令: passive");
                        }
                    }).detach();
                }
            });
            web_comm_.start();
            
            std::string local_ip = web_comm_.getLocalIP();
            RCLCPP_INFO(this->get_logger(), "Web界面: http://%s:%d", local_ip.c_str(), HTTP_PORT);
        }
        
        // 配置Android通讯
        android_comm_.setLogCallback(log_cb);
        android_comm_.start();
        
        // 创建直接控制定时器（10Hz）
        direct_control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DirectController::timerCallback, &direct_controller_)
        );
        
        RCLCPP_INFO(this->get_logger(), "机器人中枢节点 'robot_nexus' 已启动");
    }
    
    ~RobotNexusNode()
    {
        lidar_tracker_.destroyWindows();
    }

private:
    // 共享状态
    SharedState shared_state_;
    
    // 功能模块
    LidarTracker lidar_tracker_;
    DirectController direct_controller_;
    WebCommManager web_comm_;
    AndroidCommManager android_comm_;
    
    // ROS2成员
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr direct_control_timer_;
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        lidar_tracker_.processScan(scan_msg);
    }

    void topic_callback(const geometry_msgs::msg::Point  msg) 
    {
  
        lidar_tracker_.processtalker(msg);
    }

};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobotNexusNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
