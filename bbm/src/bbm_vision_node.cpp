// standard library includes
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>


//ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


#include "aruco.hpp"

/** Node that, given an image, a depth image and robot pose, finds (if any) ArUco markers and then compute the parameters of the line the robothas to converge at **/
class BBM_Vision_Node : public rclcpp::Node{

	private:
		// Subscribers
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoSub;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
		
		
		std::vector<double> pose;
		
		// Camera data
		cv::Matx33d camera_matrix;
		cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros(); 
		
		int markerSize;
		
	public:
	
		BBM_Vision_Node() : Node("bbm_vision_node"), pose(6,0){
			camera_matrix = cv::Matx33d::eye(); 
			markerSize = 250;
			
			rclcpp::QoS custom_qos(10);
			
			auto sub_opt = rclcpp::SubscriptionOptions();

        		imgSub = create_subscription<sensor_msgs::msg::Image>( "/zed/zed_node/left/image_rect_color", custom_qos, std::bind(&BBM_Vision_Node::leftImageCallback, this, std::placeholders::_1), sub_opt);
        		imgSub = create_subscription<sensor_msgs::msg::CameraInfo>( "/zed/zed_node/left/camera_info", custom_qos, std::bind(&BBM_Vision_Node::cameraCalibrationCallback, this, std::placeholders::_1), sub_opt);
        		poseSub = create_subscription<geometry_msgs::msg::PoseStamped>("/zed/zed_node/pose", custom_qos, std::bind(&BBM_Vision_Node::poseCallback, this, std::placeholders::_1), sub_opt);
      			 RCLCPP_INFO(this->get_logger(), "BBM_Vision_Node started");
		}
		
		/** Method that saves the pose for later computation **/
		void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

		    	tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,  msg->pose.orientation.w);

			// 3x3 Rotation matrix from quaternion
			tf2::Matrix3x3 m(q);

			// Roll Pitch and Yaw from rotation matrix
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
		    	
		    	pose[0] =  msg->pose.position.x;
		    	pose[1] =  msg->pose.position.y;
		    	pose[2] =  msg->pose.position.z;
		    	
		    	pose[3] =  roll;
		    	pose[4] =  pitch;
		    	pose[5] =  yaw;
		    	
		    	//RCLCPP_INFO(this->get_logger(), "BBM_Vision_Node got pose");
		}
		
		/** Method that elaborates left image: scans for ArUco markers, if found then compute the position of the marker, get the line angular coefficient (from the marker id) and finally 
			sends the parameters of the line towards the robot has to converge **/
		void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
			cv_bridge::CvImagePtr cv_ptr;
			try{
				//cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //restituisce un puntatore a CvImage (tra i suoi attributi c'è la Mat)
				cv::Mat img;
				rotate((cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)) -> image, img, 1); // rotate image 180°
				
				
				//RCLCPP_INFO(this->get_logger(), "Channel img: %d", img.channels());
				/** ArUco **/
				// Find ArUco markers 
				std::vector<int> markerIds;
				std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
				aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250); //FIXME quale dizionario?
				aruco::detectMarkers(img, dictionary, markerCorners, markerIds);
				
				std::vector<cv::Vec3d> rvecs, tvecs;
				
				cv::aruco::estimatePoseSingleMarkers( markerCorners, _markerSize, camera_matrix, dist_coeffs, rvecs, tvecs);
				
				for(int id : markerIds){
				
					
					//RCLCPP_INFO(this->get_logger(), "Riconosciuto id %d", id);
				}
						
			}catch (cv_bridge::Exception& e){
				RCLCPP_INFO(this->get_logger(), "Immagine non ottenuta");

				return;
			}			
		}
		
		
		
		void cameraCalibrationCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
			camera_matrix(0, 0) = msg->k[0];
		  	camera_matrix(1, 1) = msg->k[4];
		  	camera_matrix(0, 2) = msg->k[2];
		  	camera_matrix(1, 2) = msg->k[5];
		}
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BBM_Vision_Node>());
    	rclcpp::shutdown();
    	return 0;
}
    
