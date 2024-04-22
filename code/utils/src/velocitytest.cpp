// Standard library includes
#include <vector>
#include"PID.cpp"
#include "complementaryFilter.cpp"

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


#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>

class WriteCSV {

public:
	std::string fileName;
	std::ofstream csv_file;

	WriteCSV(std::string filename) : fileName(filename) { this->csv_file.open(this->fileName); }


	void writeData(std::vector<double> list) {
		for (int i = 0; i < list.size()-1; ++i) {
        		this->csv_file << std::setprecision(11) << list[i]<<",";
		};
		this->csv_file << std::setprecision(11) << list[list.size()]<<"\n";
	}

	void close_csv() { this->csv_file.close(); }
};	



using namespace std::chrono_literals;
class VelocityTest:public rclcpp::Node{
  private:

    WriteCSV file_vel{"vel.csv"};

		PID p;
		ComplementaryFilter cf;
    double estVel_acc;// estimated linear velocity along x axis
		double estVel_pos;
		double old_yaw;
		std::vector<double> pose; //pose=[x_r, y_r] 
		double deltaT = 10; //ms 

    rclcpp::TimerBase::SharedPtr timer;
		
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub;
	
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
		
		rclcpp::Publisher<dart_interfaces::msg::Commands>::SharedPtr commands_pub;
	public: 
    VelocityTest : Node("tv"), cf(4.74,62.6,10){
      rclcpp::QoS custom_qos(10);
			
			auto sub_opt = rclcpp::SubscriptionOptions();

            estVel_acc=0;
            estVel_pos=0;
            old_yaw=0;
		
        		twistSub = create_subscription<geometry_msgs::msg::Twist>( "/cmd_vel", custom_qos, std::bind(&Converter::twistCallback, this, std::placeholders::_1), sub_opt);

         		imuSub = create_subscription<sensor_msgs::msg::Imu>( "/zed/zed_node/imu/data", custom_qos, std::bind(&Converter::imuCallback, this, std::placeholders::_1), sub_opt);

			      timer = this->create_wall_timer(10ms, std::bind(&Converter::timerCallback, this));

            poseSub = create_subscription<geometry_msgs::msg::PoseStamped>("/zed/zed_node/pose", custom_qos, std::bind(&Converter::poseCallback, this, std::placeholders::_1), sub_opt);

        		commands_pub = this->create_publisher<dart_interfaces::msg::Commands>("dart/commands", 10);

      			RCLCPP_INFO(this->get_logger(), "Test velocity node started");

    }

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
			estVel_acc += msg->linear_acceleration.x * deltaT;
		}
void timerCallback(){
      double estvel=cf.update(estVel_pos,estVel_acc);
      std::vector<double> v(1,0);
      v[0] = estvel;
      file_vel.writeData (v);
    //RCLCPP_INFO(this->get_logger(), "Throttle calcolato", throttle);
				auto msg = new dart_interfaces::msg::Commands();
			 	msg->header.stamp  = now();
			 	msg->steering.data = 0;
			 	msg->throttle.data = 50; //Or 0.5?
			 	commands_pub->publish(*msg);
}
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
