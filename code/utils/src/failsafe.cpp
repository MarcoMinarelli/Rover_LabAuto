//ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>

//DART includes
#include "dart_interfaces/msg/commands.hpp"


/** Failsafe node that gets the depth image from ZED and measurements from LiDAr, and a minimum distance value. If one of the measurements (or a pixel inside the depth image) is less or equal 
		than given distance, it sends a commend that stops the robot.
 **/
class Failsafe : public rclcpp::Node{

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub;
		rclcpp::Publisher<dart_interfaces::msg::Commands>::SharedPtr commandsPub;
		
		double minDist;
		
		/** Function that send a command message with 0 steering angle and 0 throttle **/
		void sendStop(){
			auto msg = new dart_interfaces::msg::Commands();
		 	msg->header.stamp  = now();
		 	msg->steering.data = 0;
		 	msg->throttle.data = 0;
		 	commandsPub->publish(*msg);
		}
		
	public:
	
		Failsafe() : Node("failsafe"){
			rclcpp::QoS custom_qos(10);

			declare_parameter("min_dist", 0.20);
			
			get_parameter("min_dist", minDist);

			auto sub_opt = rclcpp::SubscriptionOptions();

        		imgSub = create_subscription<sensor_msgs::msg::Image>( "/zed/zed_node/depth/depth_registered", custom_qos, std::bind(&Failsafe::imageCallback, this, std::placeholders::_1), sub_opt);
        		laserSub = create_subscription<sensor_msgs::msg::LaserScan>( "/scan", custom_qos, std::bind(&Failsafe::laserCallback, this, std::placeholders::_1), sub_opt);
        	
        		commandsPub = this->create_publisher<dart_interfaces::msg::Commands>("dart/commands", 10);
      		
      			RCLCPP_INFO(this->get_logger(), "Failsafe node started");
		}
		
	
		/** Callback for depth image. If a depth computed is less or equal than given minimum distance, it stops **/
		void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
			float * depths = reinterpret_cast<float *>(&msg->data[0]);
			float min = 1000;
			for(int i = 0; i < msg->width; i++){
				for(int j = 0; j < msg->height; j++){
					if(depths[i + msg->width * j] < min)
						min = depths[i + msg->width * j];
				}
			}
			if(min <= minDist){
				sendStop();
				RCLCPP_INFO(this->get_logger(), "Rover stopped by depth image");
			}
		}
		
		/** Callback for LiDar. If a LiDar measurement is less or equal than given minimum distance, it stops **/
		void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
			RCLCPP_INFO(this->get_logger(), "Got new message");
			for (int i=0; i<360; i++){
				RCLCPP_INFO(this->get_logger(), "Ranges %d", msg->ranges[i]);
				if(msg->ranges[i]<= minDist){
					sendStop();
					RCLCPP_INFO(this->get_logger(), "Rover stopped by LiDar");
				}
			}
		}
			
		
		
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Failsafe>());
    	rclcpp::shutdown();
    	return 0;
}
    
