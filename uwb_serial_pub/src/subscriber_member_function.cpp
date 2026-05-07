// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
  
    // ----- 常量定义 -----
 double FOLLOW_DIST = 0.4;           // 机器人与目标的预设距离 (米)
 double TARGET_RADIUS = 0.3;         // 目标搜索半径 (米)
 double LINEAR_SCALE_FACTOR = 0.5;   // 前后运动速度比例系数
 double ANGULAR_SCALE_FACTOR = 1.0;  // 旋转运动速度比例系数
 double LINEAR_Y_SCALE_FACTOR = 1.0; // 左右运动速度比例系数
 double RECTANGLE_WIDTH = 0.35;      // 矩形宽度 (米)

// 速度限制
 double MAX_LINEAR_SPEED = 1.0;
 double MAX_ANGULAR_SPEED = 1.0;
  
  
 // 势场法避障参数
 double APF_INFLUENCE_DIST = 0.25;   // 障碍物影响距离 (米)
 double APF_REPULSE_GAIN = 0.01;      // 排斥力增益
 double APF_EMERGENCY_DIST = 0.2;    // 紧急停止距离 (米)
 double APF_SLOWDOWN_DIST = 0.25;    // 减速距离 (米)
  
  void calculateFollowVelocity(geometry_msgs::msg::Twist& cmd, double target_x, double target_y,
                                  double left_y_min, double right_y_min,
                                  double repulse_x, double repulse_y, double min_obstacle_dist) {
        // 紧急停止检查
        if (min_obstacle_dist < APF_EMERGENCY_DIST) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            return;
        }
        
        // 前后运动控制（带死区）
        double dist_error = target_x - FOLLOW_DIST;
        if (std::abs(dist_error) < 0.05) {
            cmd.linear.x = 0.0;
        } else {
            cmd.linear.x = dist_error * LINEAR_SCALE_FACTOR;
            if (cmd.linear.x < 0) cmd.linear.x *= 0.8;
            
            double min_speed = 0.06;
            if (std::abs(cmd.linear.x) < min_speed) {
                cmd.linear.x = (cmd.linear.x > 0) ? min_speed : -min_speed;
            }
        }
        
        // 旋转运动控制（带死区）
        double angle_error = atan2(target_y, target_x);
        if (std::abs(angle_error) < 0.1) {
            cmd.angular.z = 0.0;
        } else {
            cmd.angular.z = angle_error * ANGULAR_SCALE_FACTOR;
        }
        
        // 横向运动控制（带死区）
        double lateral_error = -(left_y_min + right_y_min);
        if (std::abs(lateral_error) > 1.0) lateral_error = 0.0;
        if (std::abs(lateral_error) < 0.03) {
            cmd.linear.y = 0.0;
        } else {
            cmd.linear.y = lateral_error * LINEAR_Y_SCALE_FACTOR;
        }
        
        // 融合势场排斥力
        cmd.linear.x += repulse_x;
        cmd.linear.y += repulse_y;
        
        // 接近障碍时减速
        if (min_obstacle_dist < APF_SLOWDOWN_DIST) {
            double slowdown_factor = (min_obstacle_dist - APF_EMERGENCY_DIST) 
                                   / (APF_SLOWDOWN_DIST - APF_EMERGENCY_DIST);
            slowdown_factor = std::clamp(slowdown_factor, 0.1, 1.0);
            cmd.linear.x *= slowdown_factor;
        }
        
        // 限制速度
        cmd.linear.x = std::clamp(cmd.linear.x, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        cmd.linear.y = std::clamp(cmd.linear.y, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        cmd.angular.z = std::clamp(cmd.angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    }
  
  
  

private:
  void topic_callback(const geometry_msgs::msg::Point  msg) 
  {
  
    double left_y_min = -RECTANGLE_WIDTH / 2;
    double right_y_min = RECTANGLE_WIDTH / 2;
    
    // 计算速度
    geometry_msgs::msg::Twist cmd_vel_msg;
    
    calculateFollowVelocity(cmd_vel_msg, msg.x, msg.y, 
                                    left_y_min, right_y_min,0,0,1);
  
  
    RCLCPP_INFO(this->get_logger(), "FollowVelocity: '%lf' '%lf' '%lf'", cmd_vel_msg.linear.x , cmd_vel_msg.linear.y  , cmd_vel_msg.angular.z );
  }
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
