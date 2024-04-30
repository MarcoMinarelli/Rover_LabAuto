// Standard library includes
#include <vector>
#include"PID.cpp"
#include "complementaryfilter.cpp"

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

using namespace std::chrono_literals;


/**	Node that gets desired angular and linear velocities, computes right values for steering angle and throttle, then sends them to DART interface

**/
class Converter : public rclcpp::Node{

	private:
	
		PID p;
		ComplementaryFilter cf;
		
		double desVel;
		double desAng;
		
		double steering_current;
		double estVel_acc;// estimated linear velocity along x axis
		double estVel_pos;
		double old_yaw;
		std::vector<double> pose; //pose=[x_r, y_r] 
		double deltaT = 0.01; //s
		double acc_bias = 0.09;
		
		bool ok; 
		rclcpp::TimerBase::SharedPtr timer;
		
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub;
		

		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
		
		rclcpp::Publisher<dart_interfaces::msg::Commands>::SharedPtr commands_pub;
		
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
		
	public:
	
		Converter() : Node("converter"), p(1,1,1,10,1,7), cf(0.985), pose(2, 0) {
			rclcpp::QoS custom_qos(10);
			
			desVel = 0;
			desAng = 0;
			estVel_acc = 0;
			estVel_pos=0;
			old_yaw=0;
			steering_current=0;
			
			auto sub_opt = rclcpp::SubscriptionOptions();

		
        		twistSub = create_subscription<geometry_msgs::msg::Twist>( "/cmd_vel", custom_qos, std::bind(&Converter::twistCallback, this, std::placeholders::_1), sub_opt);


        		imuSub = create_subscription<sensor_msgs::msg::Imu>( "/zed/zed_node/imu/data", custom_qos, std::bind(&Converter::imuCallback, this, std::placeholders::_1), sub_opt);
			timer = this->create_wall_timer(10ms, std::bind(&Converter::timerCallback, this));
			poseSub = create_subscription<geometry_msgs::msg::PoseStamped>("/zed/zed_node/pose", custom_qos, std::bind(&Converter::poseCallback, this, std::placeholders::_1), sub_opt);
        	
        		commands_pub = this->create_publisher<dart_interfaces::msg::Commands>("dart/commands", 10);
      			RCLCPP_INFO(this->get_logger(), "Converter node started");
		}
		
	
		/** Callback for Twist message. Just copies the value we're interested in (e.g.linear velocity along x axis and angular velocity around z axis) for further use **/
		void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
			desVel = msg->linear.x;
			desAng = msg->angular.z;
		}
		
	
		/** Callback for Imu message. Removes accelerometer bias, then computes instant velocity and adds it to the computed speed (estimated from accelerometer) **/
		void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
			estVel_acc += (msg->linear_acceleration.x - acc_bias) * deltaT ;
		}
	
		/** Callback for Timer. Computes from desired angular ad linear velocity the correct steering and throttle values **/
		void timerCallback(){
			if(ok){
				double steering = steering_current + deltaT*desAng;
				steering_current=steering;
				double throttle = 0;
				double error = desVel - cf.update(estVel_pos, estVel_acc);
				
				throttle = p.compute(error);
				
				
				//RCLCPP_INFO(this->get_logger(), "Throttle calcolato", throttle);
				auto msg = new dart_interfaces::msg::Commands();
			 	msg->header.stamp  = now();
			 	msg->steering.data = steering;
			 	msg->throttle.data = throttle;
			 	commands_pub->publish(*msg);
		 	}
		}
		
		/** Callback for Pose message. Estimtes velocity along x-axis from pose and relative derivative**/
		void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
			tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,  msg->pose.orientation.w);

			// 3x3 Rotation matrix from quaternion
			tf2::Matrix3x3 m(q);

			// Roll Pitch and Yaw from rotation matrix
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			double x_dot=(msg->pose.position.x-pose[0])/deltaT;
			double psi_dot=(yaw-old_yaw)/deltaT;
			estVel_pos=x_dot-psi_dot*msg->pose.position.y;
			

			
		    	pose[0] =  msg->pose.position.x;
		    	pose[1] =  msg->pose.position.y;
			old_yaw=yaw;

		 	ok = true;
		}
			
		
		
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Converter>());
    	rclcpp::shutdown();
    	return 0;
}
    
