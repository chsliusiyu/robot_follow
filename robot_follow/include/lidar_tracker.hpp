/**
 * @file lidar_tracker.hpp
 * @brief 雷达跟随模块 - 处理激光雷达数据并计算跟随速度
 */

#ifndef LIDAR_TRACKER_HPP
#define LIDAR_TRACKER_HPP

#include "common_types.hpp"
#include "kalman_filter.hpp"
#include "nmpc_planner.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <functional>
#include <chrono>

/**
 * @class LidarTracker
 * @brief 雷达跟随处理器
 */
class LidarTracker {
public:
    using VelocityCallback = std::function<void(const geometry_msgs::msg::Twist&)>;
    using DataBroadcastCallback = std::function<void()>;
    
    LidarTracker(SharedState& state) : state_(state) {
        // 计算机器人在窗口中的像素坐标
        robot_center_pixel_ = cv::Point(
            WINDOW_SIZE / 2,
            WINDOW_SIZE - static_cast<int>(ROBOT_Y_OFFSET_M * METERS_TO_PIXELS)
        );
    }
    
    // 设置速度发布回调
    void setVelocityCallback(VelocityCallback cb) {
        velocity_callback_ = std::move(cb);
    }
    
    // 设置数据广播回调
    void setDataBroadcastCallback(DataBroadcastCallback cb) {
        data_broadcast_callback_ = std::move(cb);
    }
    
    // 设置OpenCV可视化开关
    void setOpenCVEnabled(bool enabled) {
        enable_opencv_ = enabled;
        if (enabled) {
            cv::namedWindow("Follow", cv::WINDOW_AUTOSIZE);
        }
    }
    
    // 设置卡尔曼滤波开关
    void setKalmanEnabled(bool enabled) {
        enable_kalman_ = enabled;
        if (enabled) {
            kalman_.reset();
        }
    }
    
    // 设置卡尔曼滤波参数
    void setKalmanParams(double process_noise, double measurement_noise) {
        kalman_.setProcessNoise(process_noise);
        kalman_.setMeasurementNoise(measurement_noise);
    }
    
    // 销毁OpenCV窗口
    void destroyWindows() {
        if (enable_opencv_) {
            cv::destroyAllWindows();
        }
    }

    void processtalker(const geometry_msgs::msg::Point msg)
    {
        last_uwb_time_ = std::chrono::steady_clock::now();

        if (!state_.active.load()) {
            return;
        }

        if (enable_kalman_) {
                double filtered_x, filtered_y;
                kalman_.update(msg.x, msg.y, filtered_x, filtered_y);
                state_.setTarget(filtered_x, filtered_y);
        } else {
                state_.setTarget(msg.x, msg.y);
        }

        // double left_y_min = -RECTANGLE_WIDTH / 2;
        // double right_y_min = RECTANGLE_WIDTH / 2;
    
        // // 计算速度
        // geometry_msgs::msg::Twist cmd_vel_msg;
        // int mode = state_.control_mode.load();
    
        // if (mode == MODE_DIRECT) {
        //     double vx, vy, wz;
        //     state_.getDirectCmd(vx, vy, wz);
        //     cmd_vel_msg.linear.x = vx;
        //     cmd_vel_msg.linear.y = vy;
        //     cmd_vel_msg.angular.z = wz;
        // }
        // else if (mode == MODE_FOLLOW) {
        //     calculateFollowVelocity(cmd_vel_msg, msg.x, msg.y, 
        //                             left_y_min, right_y_min , 0 , 0, 1);
        // }

        // // 缓存速度
        // state_.setVelocity(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);
        
        // // 发布速度
        // publishVelocity(cmd_vel_msg, mode);

        
        // // 广播数据
        // if (data_broadcast_callback_) {
        //     data_broadcast_callback_();
        // }

    }
    
    
    /**
     * @brief 处理激光扫描数据
     */
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        if (!state_.active.load()) {
            return;
        }
        
        double target_x, target_y;
        state_.getTarget(target_x, target_y);
        
        cv::Mat image;
        if (enable_opencv_) {
            image = cv::Mat::zeros(TOTAL_WINDOW_HEIGHT, WINDOW_SIZE, CV_8UC3);
            drawBackground(image, target_x, target_y);
        }
        
        if (scan_msg->ranges.empty()) {
            return;
        }
        
        // 处理扫描点
        std::vector<std::pair<double, double>> points;
        points.reserve(scan_msg->ranges.size());
        
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float range = scan_msg->ranges[i];
            if (std::isinf(range) || std::isnan(range)) continue;
            
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double point_x = -range * cos(angle);
            double point_y = -range * sin(angle);
            
            // 计算到机器人的距离
            double dist_to_robot = std::sqrt(point_x * point_x + point_y * point_y);
            
            // 可视化：根据距离着色
            if (enable_opencv_) {
                cv::Scalar color;
                if (dist_to_robot < NMPC_EMERGENCY_DIST) {
                    color = cv::Scalar(0, 0, 255);      // 红色：危险
                } else if (dist_to_robot < NMPC_OBS_CUTOFF) {
                    color = cv::Scalar(0, 165, 255);    // 橙色：障碍物影响范围
                } else {
                    color = cv::Scalar(100, 100, 100);  // 灰色：安全
                }
                cv::circle(image, toPixel(point_x, point_y), 2, color, -1, cv::LINE_AA);
            }
            
            // 排除机器人框架内的点
            bool in_robot_frame = (point_x > -ROBOT_FRAME_BACK && point_x < ROBOT_FRAME_FRONT &&
                                   point_y > -ROBOT_FRAME_RIGHT && point_y < ROBOT_FRAME_LEFT);

            if (in_robot_frame) continue;

            // 排除UWB目标附近的点 — 这些点来自被跟随者，不应视为障碍物
            double dist_to_target = std::sqrt(
                (point_x - target_x) * (point_x - target_x) +
                (point_y - target_y) * (point_y - target_y));
            if (dist_to_target < TARGET_MASK_RADIUS) continue;

            points.emplace_back(point_x, point_y);
        }
        
        // 更新点云缓存
        state_.setPoints(std::move(points));

        // 计算速度
        geometry_msgs::msg::Twist cmd_vel_msg;
        int mode = state_.control_mode.load();

        if (mode == MODE_DIRECT) {
            double vx, vy, wz;
            state_.getDirectCmd(vx, vy, wz);
            cmd_vel_msg.linear.x = vx;
            cmd_vel_msg.linear.y = vy;
            cmd_vel_msg.angular.z = wz;
        }
        else if (mode == MODE_FOLLOW) {
            // UWB 信号丢失超时检测：原地旋转搜索信号
            auto now = std::chrono::steady_clock::now();
            double uwb_elapsed = std::chrono::duration<double>(now - last_uwb_time_).count();
            if (uwb_elapsed > UWB_TIMEOUT_S) {
                nmpc_planner_.resetWarmstart();
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.linear.y = 0.0;
                cmd_vel_msg.angular.z = UWB_SEARCH_WZ;
            } else {
                double cur_vx, cur_vy, cur_wz;
                state_.getVelocity(cur_vx, cur_vy, cur_wz);

                auto obstacles = state_.getPoints();
                auto result = nmpc_planner_.solve(obstacles, target_x, target_y, cur_vx, cur_vy, cur_wz);
                cmd_vel_msg.linear.x = result.vx;
                cmd_vel_msg.linear.y = result.vy;
                cmd_vel_msg.angular.z = result.wz;
            }
        }

        // 到达目标处：停止移动，只做朝向对齐
        double target_dist = std::sqrt(target_x * target_x + target_y * target_y);
        if (std::abs(target_dist - FOLLOW_DIST) < 0.1) {
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.linear.y = 0.0;
            double angle_to_target = std::atan2(target_y, target_x);
            cmd_vel_msg.angular.z = std::clamp(angle_to_target * 0.6, -0.4, 0.4);
        }

        // 缓存速度
        state_.setVelocity(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);

        // 发布速度
        publishVelocity(cmd_vel_msg, mode);
        
        // OpenCV可视化
        if (enable_opencv_) {
            drawSpeedDisplay(image, cmd_vel_msg);
            std::string status = state_.is_moving_enabled.load() ? "MOVING" : "STOPPED";
            cv::putText(image, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       state_.is_moving_enabled.load() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
            cv::imshow("Follow", image);
            cv::waitKey(1);
        }
        
        // 广播数据
        if (data_broadcast_callback_) {
            data_broadcast_callback_();
        }
    }

private:
    SharedState& state_;
    cv::Point robot_center_pixel_;
    bool enable_opencv_ = false;
    bool enable_kalman_ = false;
    KalmanFilter2D kalman_;
    NMPCPlanner nmpc_planner_;
    std::chrono::steady_clock::time_point last_uwb_time_ = std::chrono::steady_clock::now();
    VelocityCallback velocity_callback_;
    DataBroadcastCallback data_broadcast_callback_;
    
    // 坐标转换
    cv::Point toPixel(double robot_x, double robot_y) const {
        int px = robot_center_pixel_.x - static_cast<int>(robot_y * METERS_TO_PIXELS);
        int py = robot_center_pixel_.y - static_cast<int>(robot_x * METERS_TO_PIXELS);
        return cv::Point(px, py);
    }
    
    // 绘制背景
    void drawBackground(cv::Mat& image, double target_x, double target_y) {
        double dist_to_target = std::sqrt(target_x * target_x + target_y * target_y);
        if (dist_to_target > 1e-6) {
            double angle_to_target = atan2(target_y, target_x);
            double half_width = RECTANGLE_WIDTH / 2.0;
            
            cv::Point2f corners_robot[4];
            corners_robot[0] = cv::Point2f(0 - half_width * sin(angle_to_target), 0 + half_width * cos(angle_to_target));
            corners_robot[1] = cv::Point2f(0 + half_width * sin(angle_to_target), 0 - half_width * cos(angle_to_target));
            corners_robot[2] = cv::Point2f(target_x + half_width * sin(angle_to_target), target_y - half_width * cos(angle_to_target));
            corners_robot[3] = cv::Point2f(target_x - half_width * sin(angle_to_target), target_y + half_width * cos(angle_to_target));
            
            std::vector<cv::Point> corners_pixel;
            for (int i = 0; i < 4; ++i) {
                corners_pixel.push_back(toPixel(corners_robot[i].x, corners_robot[i].y));
            }
            cv::fillConvexPoly(image, corners_pixel, cv::Scalar(50, 50, 50), cv::LINE_AA);
        }
        
        cv::circle(image, robot_center_pixel_, 8, cv::Scalar(255, 200, 200), -1, cv::LINE_AA);
        const std::vector<double> scales = {1.0, 2.0, 3.0, 4.0};
        for (double dist : scales) {
            int radius_px = static_cast<int>(dist * METERS_TO_PIXELS);
            cv::circle(image, robot_center_pixel_, radius_px, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);
        }
        
        cv::Point target_center_px = toPixel(target_x, target_y);
        int target_radius_px = static_cast<int>(TARGET_RADIUS * METERS_TO_PIXELS);
        cv::circle(image, target_center_px, target_radius_px, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
        cv::line(image, robot_center_pixel_, target_center_px, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    
    // 发布速度
    void publishVelocity(const geometry_msgs::msg::Twist& cmd, int mode) {
        if (!velocity_callback_) return;
        
        if (state_.is_moving_enabled.load() || mode == MODE_DIRECT) {
            if (mode == MODE_DIRECT) {
                velocity_callback_(cmd);
            } else if (state_.is_moving_enabled.load()) {
                velocity_callback_(cmd);
            } else {
                geometry_msgs::msg::Twist zero_vel;
                velocity_callback_(zero_vel);
            }
        } else {
            geometry_msgs::msg::Twist zero_vel;
            velocity_callback_(zero_vel);
        }
    }
    
    // 绘制速度显示
    void drawSpeedDisplay(cv::Mat& image, const geometry_msgs::msg::Twist& cmd_vel) {
        int center_x = WINDOW_SIZE / 2;
        int center_y = WINDOW_SIZE + SPEED_DISPLAY_HEIGHT / 2;
        
        cv::line(image, cv::Point(0, WINDOW_SIZE), cv::Point(WINDOW_SIZE, WINDOW_SIZE),
                 cv::Scalar(80, 80, 80), 1);
        
        cv::line(image, cv::Point(center_x - SPEED_BAR_LENGTH - 10, center_y),
                 cv::Point(center_x + SPEED_BAR_LENGTH + 10, center_y),
                 cv::Scalar(60, 60, 60), 1);
        cv::line(image, cv::Point(center_x, center_y - SPEED_BAR_LENGTH - 10),
                 cv::Point(center_x, center_y + SPEED_BAR_LENGTH + 10),
                 cv::Scalar(60, 60, 60), 1);
        
        double vx = std::clamp(cmd_vel.linear.x, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        double vy = std::clamp(cmd_vel.linear.y, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        double wz = std::clamp(cmd_vel.angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
        
        int bar_x = static_cast<int>((vx / MAX_LINEAR_SPEED) * SPEED_BAR_LENGTH * 4);
        int bar_y = static_cast<int>((vy / MAX_LINEAR_SPEED) * SPEED_BAR_LENGTH * 4);
        
        if (std::abs(bar_x) > 1) {
            cv::line(image, cv::Point(center_x, center_y),
                     cv::Point(center_x, center_y - bar_x),
                     cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
        }
        
        if (std::abs(bar_y) > 1) {
            cv::line(image, cv::Point(center_x, center_y),
                     cv::Point(center_x - bar_y, center_y),
                     cv::Scalar(0, 255, 0), 4, cv::LINE_AA);
        }
        
        cv::circle(image, cv::Point(center_x, center_y), 5, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        
        int arc_center_x = center_x + 120;
        cv::circle(image, cv::Point(arc_center_x, center_y), SPEED_ARC_RADIUS,
                   cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
        
        if (std::abs(wz) > 0.01) {
            double start_angle = -90;
            double arc_angle = -(wz / MAX_ANGULAR_SPEED) * 180;
            double end_angle = start_angle + arc_angle;
            
            double draw_start = start_angle;
            double draw_end = end_angle;
            if (arc_angle < 0) {
                std::swap(draw_start, draw_end);
            }
            
            cv::ellipse(image, cv::Point(arc_center_x, center_y), 
                       cv::Size(SPEED_ARC_RADIUS, SPEED_ARC_RADIUS),
                       0, draw_start, draw_end,
                       cv::Scalar(255, 150, 0), 4, cv::LINE_AA);
        }
        
        char buf[64];
        snprintf(buf, sizeof(buf), "Vx:%.2f", cmd_vel.linear.x);
        cv::putText(image, buf, cv::Point(10, WINDOW_SIZE + 25),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        
        snprintf(buf, sizeof(buf), "Vy:%.2f", cmd_vel.linear.y);
        cv::putText(image, buf, cv::Point(10, WINDOW_SIZE + 50),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        
        snprintf(buf, sizeof(buf), "Wz:%.2f", cmd_vel.angular.z);
        cv::putText(image, buf, cv::Point(10, WINDOW_SIZE + 75),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 150, 0), 1);
    }
};

#endif // LIDAR_TRACKER_HPP
