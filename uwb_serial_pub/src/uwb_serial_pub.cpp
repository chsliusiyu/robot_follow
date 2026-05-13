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
#include <chrono>
#include <memory>
#include <thread>
#include <string>
#include <sstream>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <atomic>
#include <functional>
#include <iostream> 


#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher")
  {
  
    // 声明参数
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 115200);

    // 获取参数
    std::string serial_port = this->get_parameter("serial_port").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();
    
    
    // 打开串口
    serial_fd_ = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", serial_port.c_str(), strerror(errno));
      rclcpp::shutdown();
      return;
    }

    // 配置串口
    if (!configure_serial(baud_rate)) {
      close(serial_fd_);
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud", serial_port.c_str(), baud_rate);
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("topic", 10);
    

    RCLCPP_INFO(this->get_logger(), "Publisher started. Publishing immediately upon serial data.");
    running_ = true;
    
    set_serial_state(1);

  }
  
   ~MinimalPublisher()
  {
    set_serial_state(0);

    // 通知线程退出并等待
    running_ = false;
    // 关闭串口
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }
  
  
    // 串口读取线程函数
  void serial_read_loop()
  {
    std::string buffer;
    char ch[255];
    while (running_ && rclcpp::ok()) {
      int n = read(serial_fd_, &ch, 255);
      if (n > 0) {
      
            buffer.append(ch, n);
            
            RCLCPP_INFO(this->get_logger(), "Serial read :   %d" ,n);
            RCLCPP_INFO(this->get_logger(), "Serial read :   %s" , buffer.c_str());
            
            parse_and_publish(buffer);
            
            buffer.clear();
        
      } else if (n < 0) {
        RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
        std::this_thread::sleep_for(100ms);
      } else {
        std::this_thread::sleep_for(50ms);
      }
    }
  }
  


private:



  // 配置串口参数
  bool configure_serial(int baud_rate)
  {
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
      return false;
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                          // disable break processing
    tty.c_lflag = 0;                                 // no signaling, echo, etc.
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;                             // read doesn't block
    tty.c_cc[VTIME] = 5;                             // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // shut off flow control
    tty.c_cflag |= (CLOCAL | CREAD);                  // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);                // no parity
    tty.c_cflag &= ~CSTOPB;                            // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                           // no hardware flowcontrol

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
      return false;
    }
    return true;
  }

  void set_serial_state(char flag)
  {
    if(flag == 1)
    {
  	std::string start = "initf\r\n"; 
        std::string diag = "diag 1\r\n";

        int n = write(serial_fd_, diag.c_str(), diag.size());
        if (n  == (int)diag.size()) {
           RCLCPP_INFO(this->get_logger(), "serial_fd_ write diag.");
        }

        sleep(3);

    	 n = write(serial_fd_, start.c_str(), start.size());
        if (n  == (int)start.size()) {
           RCLCPP_INFO(this->get_logger(), "serial_fd_ write start.");
        }
    }
    else
    {
    	std::string stop = "stop\r\n"; 
    
    	int n = write(serial_fd_, stop.c_str(), stop.size());
        if (n  == (int)stop.size()) {
           RCLCPP_INFO(this->get_logger(), "serial_fd_ write stop.");
        }
    }
  
  }

  void parse_and_publish(const std::string& buffer)
  {
  
    double distance, loc_az_aoa, RSSI;
  
    size_t distPos = buffer.find("distance[cm]=");
    if (distPos == std::string::npos) {
        return ;
    }

    const char* distStart = buffer.c_str() + distPos + strlen("distance[cm]=");
    distance = strtod(distStart, nullptr);
    
    distance = distance/100;


    size_t azPos = buffer.find("loc_az=");
    if (azPos == std::string::npos) {
        return ;
    }

    const char* azStart = buffer.c_str() + azPos + strlen("loc_az=");
    loc_az_aoa = strtod(azStart, nullptr);


    size_t rssiPos = buffer.find("RSSI[dBm]=");
    if (rssiPos == std::string::npos) {
        return ;
    }

    const char* rssiStart = buffer.c_str() + rssiPos + strlen("RSSI[dBm]=");
    RSSI = strtod(rssiStart, nullptr);
    
    RCLCPP_INFO(this->get_logger(), "distance: %.2f   loc_az_aoa : %.2f   RSSI : %.2f", distance , loc_az_aoa ,RSSI);
    
    if(RSSI < -79)
    {
      RSSI_enum++;
      if(RSSI_enum > 3)
      {
        RSSI_rnum = 0;
        RSSIFLAG = false;
      }
 
    }
    else{
      RSSI_rnum++;
      if(RSSI_rnum > 3)
      {
        RSSI_enum = 0;
        RSSIFLAG = true;
      }
    }
    
    double a_rad = loc_az_aoa * M_PI / 180.0;

    
    double x = distance * std::cos(a_rad);
    double y = distance * std::sin(a_rad);
    
    //RCLCPP_INFO(this->get_logger(), " x : %.2f   y : %.2f ", x ,y);
    if(RSSIFLAG)
      publish_point(x,y);
    // 信号丢失时不发布，由上层超时检测触发搜索旋转
  }
  

  // 发布Point消息
  void publish_point(double x, double y)
  {
    auto message = geometry_msgs::msg::Point();
    message.x = x;
    message.y = y;
    message.z = 0.0;

    //RCLCPP_INFO(this->get_logger(), "Publishing: targetx=%.2f, targety=%.2f", message.x, message.y);
    publisher_->publish(message);
  }

  // 成员变量
  int serial_fd_;
  std::atomic<bool> running_;
  int RSSI_rnum = 0;
  int RSSI_enum = 0;
  bool RSSIFLAG = true;
  
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
    try {
        node->serial_read_loop();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
  rclcpp::shutdown();
  return 0;
}
