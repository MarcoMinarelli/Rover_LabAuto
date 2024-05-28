// Standard library includes
#include <vector>

//ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>

//DART includes
#include "dart_interfaces/msg/commands.hpp"

//custom include
//#include "writecsv.cpp"
#include"PID.cpp"
#include "complementaryfilter.cpp"


using namespace std::chrono_literals;


class StopNode:public rclcpp::Node{
	private:
	int count = 0;
    	rclcpp::TimerBase::SharedPtr timer;
		
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
	
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
		
		rclcpp::Publisher<dart_interfaces::msg::Commands>::SharedPtr commands_pub;
	public: 
    		StopNode() : Node("StopNode"){
				rclcpp::QoS custom_qos(10);
				
				auto sub_opt = rclcpp::SubscriptionOptions();

         		
				timer = this->create_wall_timer(30ms, std::bind(&StopNode::timerCallback, this));

           	
        		commands_pub = this->create_publisher<dart_interfaces::msg::Commands>("/dart/commands", 10);

      			RCLCPP_INFO(this->get_logger(), "Stop iniziato");

		}
		void sendMessage(float v, float steering){
			auto msg = new dart_interfaces::msg::Commands();
			msg->header.stamp  = now();
			msg->steering.data = steering;
			msg->throttle.data = v; 
			commands_pub->publish(*msg); 
		}

		void timerCallback(){
				if(count < 20){
					sendMessage(0, 0);
					count++ ;
				}
			RCLCPP_INFO(this->get_logger(), "Finito");
		}
};


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StopNode>());
    rclcpp::shutdown();
    return 0;
}
