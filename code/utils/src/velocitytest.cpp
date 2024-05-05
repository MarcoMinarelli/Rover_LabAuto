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
#include "writecsv.cpp"
#include"PID.cpp"
#include "complementaryfilter.cpp"


using namespace std::chrono_literals;


class VelocityTest:public rclcpp::Node{
	private:
		WriteCSV file_vel{"vel.csv"};
		ComplementaryFilter cf;
    		double estVel_acc;// estimated linear velocity along x axis
		double estVel_pos;
		double old_yaw;
		std::vector<double> pose; //pose=[x_r, y_r] 
		double deltaT = 0.01; //s 
		double inputVel = 0.2;
		double acc_bias = 0.09;

		int count = 0;
    		rclcpp::TimerBase::SharedPtr timer;
		
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
	
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
		
		rclcpp::Publisher<dart_interfaces::msg::Commands>::SharedPtr commands_pub;
	public: 
    		VelocityTest() : Node("tv"), cf(0.985), pose(2, 0){
			rclcpp::QoS custom_qos(10);
			
			auto sub_opt = rclcpp::SubscriptionOptions();

           		estVel_acc=0;
            		estVel_pos=0;
            		old_yaw=0;
		

         		imuSub = create_subscription<sensor_msgs::msg::Imu>( "/zed/zed_node/imu/data", custom_qos, std::bind(&VelocityTest::imuCallback, this, std::placeholders::_1), sub_opt);

			timer = this->create_wall_timer(10ms, std::bind(&VelocityTest::timerCallback, this));

           		poseSub = create_subscription<geometry_msgs::msg::PoseStamped>("/zed/zed_node/pose", custom_qos, std::bind(&VelocityTest::poseCallback, this, std::placeholders::_1), sub_opt);

        		commands_pub = this->create_publisher<dart_interfaces::msg::Commands>("dart/commands", 10);

      			RCLCPP_INFO(this->get_logger(), "Test velocity node started");

		}

    
		/** Callback for Imu message. Removes accelerometer bias, then computes instant velocity and adds it to the computed speed (estimated from accelerometer) **/
		void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
			estVel_acc += (msg->linear_acceleration.x - acc_bias) * deltaT ;
		}


		void timerCallback(){
			if(count < 600){
					double estvel=cf.update(estVel_pos,estVel_acc);
				  	std::vector<double> v(2,0);
				 	v[0] = inputVel;
				 	v[1] = estvel;
				  	file_vel.writeData (v);
					RCLCPP_INFO(this->get_logger(), "acc %f pos %f est %f", estVel_acc, estVel_pos, estvel);
				count++;
				if(count>200 && count<400)
					inputVel=0.4;
				if(count>400)
					inputVel=0.6;	
			}else{
				inputVel = 0;
			}
			auto msg = new dart_interfaces::msg::Commands();
			msg->header.stamp  = now();
			msg->steering.data = 0;
			msg->throttle.data = inputVel; 
			commands_pub->publish(*msg);
			
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

				 	//ok = true;
				}

};


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocityTest>());
    	rclcpp::shutdown();
    	return 0;
}
