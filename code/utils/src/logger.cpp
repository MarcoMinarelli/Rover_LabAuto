
// standard library includes 
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

//custom include
#include "bbm_interfaces/msg/lineparams.hpp"


	
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
	
		
class Logger : public rclcpp::Node{
	private:
		// subscriber 
		rclcpp::Subscription<bbm_interfaces::msg::Lineparams>::SharedPtr lpSub;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
		WriteCSV file_robot{"robot_pose.csv"};
		WriteCSV file_params{"lines_params.csv"};
		
		
	public:
		Logger() : Node("logger") {

			rclcpp::QoS custom_qos(10);
			auto sub_opt = rclcpp::SubscriptionOptions();
        		lpSub = create_subscription<bbm_interfaces::msg::Lineparams>( "/bbm/line_params", custom_qos, std::bind(&Logger::lineParamsCallback, this, std::placeholders::_1), sub_opt);
        		poseSub = create_subscription<geometry_msgs::msg::PoseStamped>("/zed/zed_node/pose", custom_qos, std::bind(&Logger::poseCallback, this, std::placeholders::_1), sub_opt);
      			RCLCPP_INFO(this->get_logger(), "Logger_Node started");
		}
		
	
		void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
			std::vector<double> pose(2,0);
		    	pose[0] =  msg->pose.position.x;
		    	pose[1] =  msg->pose.position.y;
			file_robot.writeData (pose);
		}
		
		
		void lineParamsCallback(const bbm_interfaces::msg::Lineparams::SharedPtr msg){
			std::vector<double> line(2,0);
		    	line[0] =  msg->x;
		    	line[1] =  msg->y;
			file_params.writeData (line);
		}
		
		
};


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Logger>());
    	rclcpp::shutdown();
    	return 0;
}


