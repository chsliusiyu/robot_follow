#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include "highlevel.h"
using mc_sdk::zsl_1::HighLevel;
class NavSubscriber : public rclcpp::Node
{

public:

NavSubscriber() : Node("agibot_move_sub")

{


    // 在NavSubscriber构造函数中添加参数读取逻辑
this->declare_parameter("local_ip", "192.168.1.136");
this->declare_parameter("local_port", 43988);
this->declare_parameter("robot_ip", "192.168.1.136");

std::string local_ip;
int local_port;
std::string robot_ip;

this->get_parameter("local_ip", local_ip);
this->get_parameter("local_port", local_port);
this->get_parameter("robot_ip", robot_ip);

robot_.initRobot(local_ip, local_port, robot_ip);

//std::string local_ip = "192.168.1.136";
//int local_port = 43988;
//std::string robot_ip = "192.168.1.136";
//robot_.initRobot(local_ip, local_port, robot_ip);

robot_.standUp();

sleep(5);

movesubscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&NavSubscriber::move_topic_callback, this, std::placeholders::_1));

actionsubscription_ = this->create_subscription<std_msgs::msg::String>("/d1_cmd", 10, std::bind(&NavSubscriber::action_topic_callback, this, std::placeholders::_1));

}

private:

void move_topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%lf' '%lf' '%lf'", msg->linear.x, msg->linear.y, msg->angular.z);
 
    robot_.move(msg->linear.x, msg->linear.y, msg->angular.z);

}

void action_topic_callback(const std_msgs::msg::String & msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  
    if(msg.data == "standup")
    {
        int stand=1;
	stand=robot_.standUp();
	if(!stand)RCLCPP_INFO(this->get_logger(),"正常");else RCLCPP_INFO(this->get_logger(),"不正常");
    }
    
    else if(msg.data == "liedown")
    {
    	int stand=1;
	stand=robot_.lieDown();
	if(!stand)RCLCPP_INFO(this->get_logger(),"正常");else RCLCPP_INFO(this->get_logger(),"不正常");
      
    }
    else if(msg.data == "passive")
    {
        int stand=1;
	    stand=robot_.passive();
	    if(!stand)RCLCPP_INFO(this->get_logger(),"正常");else RCLCPP_INFO(this->get_logger(),"不正常");
      
    }
    
}

rclcpp::Subscription<std_msgs::msg::String>::SharedPtr actionsubscription_;

rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr movesubscription_;

HighLevel robot_;

};



int main(int argc, char * argv[])

{

rclcpp::init(argc, argv);

rclcpp::spin(std::make_shared<NavSubscriber>());

rclcpp::shutdown();

return 0;

}
