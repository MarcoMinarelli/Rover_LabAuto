// Standard library includes
#include <vector>
#include"PID.cpp"

//ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <chrono>

//DART includes
#include "dart_interfaces/msg/commands.hpp"
#include "dart_interfaces/msg/imu.hpp"

using namespace std::chrono_literals;


/**	Node that gets desired angular and linear velocities, computes right values for steering angle and throttle, then sends them to the robot

**/
class Converter : public rclcpp::Node{

	private:
	
		PID p;
		
		double desVel;
		double desAng;
		
		double steering_current;
		double estVel;// estimated linear velocity along x axis
		
		double deltaT = 10; //ms
		
		rclcpp::TimerBase::SharedPtr timer;
		
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub;
		

		rclcpp::Subscription<dart_interfaces::msg::Imu>::SharedPtr imuSub;
		
		rclcpp::Publisher<dart_interfaces::msg::Commands>::SharedPtr commands_pub;
		
	public:
	
		Converter() : Node("converter"), p(1,1,1,10,1,7) {
			rclcpp::QoS custom_qos(10);
			
			desVel = 0;
			desAng = 0;
			estVel = 0;
			steering_current=0;
			
			auto sub_opt = rclcpp::SubscriptionOptions();

		
        		twistSub = create_subscription<geometry_msgs::msg::Twist>( "/cmd_vel", custom_qos, std::bind(&Converter::twistCallback, this, std::placeholders::_1), sub_opt);


        		imuSub = create_subscription<dart_interfaces::msg::Imu>( "dart/telemetry/imu", custom_qos, std::bind(&Converter::imuCallback, this, std::placeholders::_1), sub_opt);
			timer = this->create_wall_timer(10ms, std::bind(&Converter::timerCallback, this));
        	
        		commands_pub = this->create_publisher<dart_interfaces::msg::Commands>("dart/commands", 10);
      			RCLCPP_INFO(this->get_logger(), "Converter node started");
		}
		
	
		/** Callback for Twist message. Just copies the value we're interested in (e.g.linear velocity along x axis and angular velocity around z axis) for further use **/
		void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
			desVel = msg->linear.x;
			desAng = msg->angular.z;
		}
		
	
	
		void imuCallback(const dart_interfaces::msg::Imu::SharedPtr msg){
			estVel += msg->acc.x * deltaT;
		}
	
		/** Callback for Timer. Computes from desired angular ad linear velocity the correct steering and throttle values **/
		void timerCallback(){
			double steering = steering_current + deltaT*desAng;
			steering_current=steering;
			double throttle = 0;
			double error = desVel - estVel;
			
			throttle = p.compute(error);
			
			auto msg = new dart_interfaces::msg::Commands();
		 	msg->header.stamp  = now();
		 	msg->steering.data = steering;
		 	msg->throttle.data = throttle;
		 	commands_pub->publish(*msg);
			
		}
			
		
		
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Converter>());
    	rclcpp::shutdown();
    	return 0;
}
    
