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
	
		PID p_v;
		PID p_theta;
		ComplementaryFilter cf;
		
		double desVel;
		double desAng;
		
		double steering_current;
		double estVel_acc;// estimated linear velocity along x axis
		double estVel_pos;
		double old_yaw;
		std::vector<double> pose; //pose=[x_r, y_r] 
		double deltaT = 0.03; //s
		
		
		double acc_bias = 0;
		double x_bias = 0;
		double y_bias = 0;
		double yaw_bias = 0;
		
		int count = 0;
		int count_acc = 0;
		int count_pose = 0;
		
		bool ok; 
		rclcpp::TimerBase::SharedPtr timer;
		
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub;
		

		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
		
		rclcpp::Publisher<dart_interfaces::msg::Commands>::SharedPtr commands_pub;
		
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
		
	public:
	
		Converter() : Node("converter"), p_v(0.8, 0.85 , 1 , 0.03, 0.65, 0), p_theta (10, 0 , 0 , 0.03, 28, -28), cf(0.985), pose(2, 0) {
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
			timer = this->create_wall_timer(30ms, std::bind(&Converter::timerCallback, this));
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
		// IMU rate: 200 Hz
		void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
			if(count_acc < 120){
				acc_bias += msg->linear_acceleration.x;
				count_acc++;
			}else{
				if(count_acc == 120) acc_bias = acc_bias/count_acc;
				estVel_acc += (msg->linear_acceleration.x - acc_bias) * 0.005 ; //deltaT imu=0.005 s
				//RCLCPP_INFO(this->get_logger(), " acc %f", msg->linear_acceleration.x - acc_bias);
				
			}
		}
	
		/** Callback for Timer. Computes from desired angular ad linear velocity the correct steering and throttle values **/
		void timerCallback(){
			if(count <  60){
				auto msg = new dart_interfaces::msg::Commands();
				msg->steering.data =0;
				msg->throttle.data = 0;
				msg->header.stamp = now();
				commands_pub->publish(*msg);
				count++;
				//RCLCPP_INFO(this->get_logger(), "steering %f", msg->steering.data);
			}else{
			if(ok){
				double des_yaw = old_yaw + deltaT * (desAng * 180 /3.14); 
				double throttle = 0;
				double error = desVel - cf.update(estVel_pos, estVel_acc) * 0.01;
				 //RCLCPP_INFO(this->get_logger(), "theta %f omega %f ",old_yaw, deltaT * (desAng * 180 /3.14));
				//RCLCPP_INFO(this->get_logger(), "Steering %f", steering);
				throttle = p_v.compute(error);
				double steering=p_theta.compute(des_yaw-old_yaw)+13; //13 experimental values
				//RCLCPP_INFO(this->get_logger(), "Steering %f omega %f", steering, desAng);
			    	//RCLCPP_INFO(this->get_logger(), "filtro %f PID %f",cf.update(estVel_pos, estVel_acc)*0.01, throttle);
				auto msg = new dart_interfaces::msg::Commands();
			 	msg->header.stamp  = now();
			 	msg->steering.data = steering;
			 	msg->throttle.data = throttle;
			 	commands_pub->publish(*msg);
		 	}
			}
		}
		
		/** Callback for Pose message. Estimates velocity along x-axis from pose and relative derivative**/
		// Pose rate: 100 Hz
		void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
			tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,  msg->pose.orientation.w);

			// 3x3 Rotation matrix from quaternion
			tf2::Matrix3x3 m(q);

			// Roll Pitch and Yaw from rotation matrix
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			
			if(count_pose < 60){
				x_bias += msg->pose.position.x;
				y_bias += msg->pose.position.y;
				yaw_bias += yaw;
				count_pose++;
			}else{
				if(count_pose == 60){
					x_bias = x_bias/count_pose;
					y_bias = y_bias/count_pose;
					yaw_bias = yaw_bias/count_pose;
				}		
						
				double x_dot=( (msg->pose.position.x -x_bias)  -pose[0])/0.01; //deltaT pose=0.01 s
				double psi_dot=((yaw - yaw_bias) - old_yaw)/0.01;
				estVel_pos=x_dot-psi_dot*(msg->pose.position.y - y_bias);
				

						
				pose[0] =  msg->pose.position.x - x_bias;
				pose[1] =  msg->pose.position.y - y_bias;
				old_yaw= yaw - yaw_bias;
				
				ok = true;
				//RCLCPP_INFO(this->get_logger(), " %f %f %f ",  msg->pose.position.x - x_bias,  msg->pose.position.y - y_bias, yaw - yaw_bias);
			}
		}
			
		
		
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Converter>());
    	rclcpp::shutdown();
    	return 0;
}
    
