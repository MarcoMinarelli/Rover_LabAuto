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


class PIDTest:public rclcpp::Node{
	private:
		WriteCSV file_vel{"PID_vel.csv"};
		ComplementaryFilter cf;
    	double estVel_acc;// estimated linear velocity along x axis
		double estVel_pos;
		double old_yaw;
		std::vector<double> pose; //pose=[x_r, y_r] 
		double deltaT = 0.03; //s 
		double desvel = 0.2; //[m/s]
		
		double acc_bias = 0;
		double x_bias = 0;
		double y_bias = 0;
		double yaw_bias = 0;
		
		PID p;

		int count = 0;
		int count_acc = 0;
		int count_pose = 0;
    	rclcpp::TimerBase::SharedPtr timer;
		
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
	
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
		
		rclcpp::Publisher<dart_interfaces::msg::Commands>::SharedPtr commands_pub;
	public: 
    		PIDTest() : Node("PIDTest"), cf(0.985), pose(2, 0), p(0.158, 4.5, 0.0012, deltaT,0.8, 0.65, 0.2){
				rclcpp::QoS custom_qos(10);
				
				auto sub_opt = rclcpp::SubscriptionOptions();

		       	estVel_acc=0;
		        estVel_pos=0;
		        old_yaw=0;
		

         		imuSub = create_subscription<sensor_msgs::msg::Imu>( "/zed/zed_node/imu/data", custom_qos, std::bind(&PIDTest::imuCallback, this, std::placeholders::_1), sub_opt);

				timer = this->create_wall_timer(30ms, std::bind(&PIDTest::timerCallback, this));

           		poseSub = create_subscription<geometry_msgs::msg::PoseStamped>("/zed/zed_node/pose", custom_qos, std::bind(&PIDTest::poseCallback, this, std::placeholders::_1), sub_opt);

        		commands_pub = this->create_publisher<dart_interfaces::msg::Commands>("/dart/commands", 10);

      			RCLCPP_INFO(this->get_logger(), "Test velocity node started");

		}

    
		/** Callback for Imu message. Removes accelerometer bias, then computes instant velocity and adds it to the computed speed (estimated from accelerometer) **/
		// IMU rate: 200 Hz
		void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
			if(count_acc < 120){
				acc_bias += msg->linear_acceleration.x;
				count_acc++;
			}else{
				if(count_acc == 120) acc_bias = acc_bias/count_acc;
				estVel_acc += (msg->linear_acceleration.x - acc_bias) * deltaT ;
				//RCLCPP_INFO(this->get_logger(), " acc %f", msg->linear_acceleration.x - acc_bias);
				
			}
		}


		void sendMessage(float v, float steering){
			auto msg = new dart_interfaces::msg::Commands();
			msg->header.stamp  = now();
			msg->steering.data = steering;
			msg->throttle.data = v; 
			commands_pub->publish(*msg); 
		}

		void timerCallback(){
			if(count < 180){
				if(count < 20){
					sendMessage(0, 0);
				} else{ 
					double estvel=cf.update(estVel_pos, estVel_acc);
					
				  	std::vector<double> v(2,0);
				 	v[0] = desvel;
				 	v[1] = estvel;
				  	file_vel.writeData (v);
					//RCLCPP_INFO(this->get_logger(), " %f,  %f",v[0],  v[1]);
					
					/*if(count>30 && count <230) inputVel=0.2;
					if(count>230 && count<430) inputVel=0.4;
					if(count>430 && count<630) inputVel=0.6;
					if (count>630) inputVel=0.8;*/
					
					sendMessage(p.compute(desvel - estvel), 15);
					RCLCPP_INFO(this->get_logger(), "risultato PID %f", p.compute(desvel-estvel));
				}	 
			}else{
				sendMessage(0, 0);
			}
			count++ ;
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
				if(count == 60){
					x_bias = x_bias/count_pose;
					y_bias = y_bias/count_pose;
					yaw_bias = yaw_bias/count_pose;
				}		
						
				double x_dot=( (msg->pose.position.x -x_bias)  -pose[0])/deltaT;
				double psi_dot=((yaw - yaw_bias) - old_yaw)/deltaT;
				estVel_pos=x_dot-psi_dot*(msg->pose.position.y -y_bias  );
				

						
				pose[0] =  msg->pose.position.x - x_bias;
				pose[1] =  msg->pose.position.y - y_bias;
				old_yaw= yaw - yaw_bias;
				
				
				//RCLCPP_INFO(this->get_logger(), " %f %f %f ",  msg->pose.position.x - x_bias,  msg->pose.position.y - y_bias, yaw - yaw_bias);
			}
		}

};


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PIDTest>());
    rclcpp::shutdown();
    return 0;
}
