// standard library includes
#include <vector>
#include <utility>
#include <cmath>

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

// tf2 includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include "bbm_interfaces/msg/lineparams.hpp"

#include "aruco.hpp"

/** Node that, given an image, a depth image and robot pose, finds (if any) ArUco markers and then compute the parameters of the line the robothas to converge at **/
class BBM_Vision_Node : public rclcpp::Node{

	private:
		// Subscribers
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoSub;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
		
		rclcpp::Publisher<bbm_interfaces::msg::Lineparams>::SharedPtr paramsPub;
		
		
		// Camera data
		cv::Matx33d camera_matrix;
		cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros(); 
		
		// Node params
		double maxDist = 0.6; //[m]
		float markerSize = 0.17f; //[m]
		
		/* Transformation between coordinate frames */
		// From ros coordinate (x pointing forward, y pointing to the left, z pointing up) to image (x to right, y to down, z pointing forward)
		tf2::Transform ros2img;
		
		// From camera coordinates to ArUco marker coordinates
		tf2::Transform pose_aruco;
		
		// From fixed frame to rover frame
		tf2::Transform fixed2rover;
		
		
		std::pair<bool, double> getParallelLineAngularCoefficient(tf2::Vector3 x1, tf2::Vector3 x2){
			bool ok;
			double m;
			if (x1.x() == x2.x()){ //points collinear along x axis
				ok = true;
				m = 0;
			}
			else if(x1.y() == x2.y()){ // points collinear along y axis
				ok = false;
				m = 0;
			}else{
				ok = true;
				m = (x1.y() - x2.y())/(x1.x() - x2.x());			
			}	
			return std::pair<bool, double>(ok, m);
		}
		
		
		/** Method that compute upper marker corners' position, the slope of line that passes through both of them and, depending on the marker id, computes the slope of the new line. Returns a 
			pair where the first element indicates if the line can be expressed as y = m*x + q or not. This is needed because vertical lines cannot be represented this way.
		**/
		std::pair<bool, double> getAngularCoefficient(int id, tf2::Transform transf){
			std::pair<bool, double> ret;

			
			tf2::Vector3 topRight =  fixed2rover * transf * tf2::Vector3(markerSize/2, markerSize/2, 0);
			tf2::Vector3 topLeft =  fixed2rover * transf * tf2::Vector3(-markerSize/2, markerSize/2, 0);

			//RCLCPP_INFO(this->get_logger(), "TopLeft (%f, %f) TopRight(%f, %f)", topLeft.x(), topLeft.y(), topRight.x(), topRight.y());
			
			
			//Computing the slope of the line that contains both markers' corners. If the marker id is 0 or 1, we'll use this
			ret = getParallelLineAngularCoefficient(topLeft, topRight);
			
			
			double m;
			bool ok;
			if(id == 2){ //desired line forms 45° respect to ArUco
				if(ret.first && ret.second != 1){ //if the ArUco segment is not vertical and has slope !=1 
					m = ((ret.second + 1)/(1 - ret.second)); // m is the tangent of the line parallel to ArUco, we want it rotated by 45°, tan(45°) = 1, then we apply the sum of tangent formula
					ok = true;
				}else if(ret.first && ret.second == 1){ // if the line between corners is not vertical and is already at 45° => the resulting line is parallel to y axis
					ok = false;
					m = 0;
				}else{ // the line between corners is vertical => the slope of desired line is -1 (90 + 45° = 135°)
					ok = true;
					m = -1;
				}
			}else if(id == 3){ // ArUco segment forms an angle of 135° wrt marker				
				if(ret.first && ret.second  != -1){ // line not vertical and marker has slope != -1 
					ok = true;
					m = (ret.second - 1)/(1 + ret.second); // same concept as case with id = 2
				}else if(ret.first && ret.second  == -1){ // line not vertical, marker has slope -1 => resulting line parallel to y axis
					ok = false;
					m = 0;
				}else{ // line vertical => desired line has slope 1 (90°+ 135°)
					ok = true;
					m = 1;
				}
			}else if(id == 4){
				ok = true;
				m = 0;
			}
			return  (id == 0 || id == 1)? ret : std::pair<bool, double>(ok, m);
		}
		
		
		/** Method that given a translation and rotation compute the angle (on xy plane) of the object wrt robots' x axis **/
		tf2::Transform getTransform(cv::Vec3d t, cv::Vec3d c){
			
  			// From OpenCV to Tf2 Transform. It represent the transformation between camera coordinates and ArUco marker coordinates
			tf2::Vector3 tf2_origin(t[0], t[1], t[2]);
			cv::Mat cv_rot(3, 3, CV_64F);
			cv::Rodrigues(c, cv_rot);
				  
			tf2::Matrix3x3 tf2_rot(cv_rot.at<double>(0, 0), cv_rot.at<double>(0, 1), cv_rot.at<double>(0, 2), 
			cv_rot.at<double>(1, 0), cv_rot.at<double>(1, 1), cv_rot.at<double>(1, 2), cv_rot.at<double>(2, 0), 
			cv_rot.at<double>(2, 1), cv_rot.at<double>(2, 2));

			//tf2::Transform pose_aruco(tf2_rot, tf2_origin);
			pose_aruco.setBasis(tf2_rot);
			pose_aruco.setOrigin(tf2_origin);

			// From ros to ArUco, multiplying the previous transforms				
			tf2::Transform ros2aruco;
			ros2aruco.mult(ros2img, pose_aruco);

			return ros2aruco; 
		}
		
		/** Method that given the transaltion of a lsit of markers returns the index of closest one **/
		int getClosestMarker(std::vector<cv::Vec3d> tvecs){
			int ret = -1;
		  	double nearest_distance = 1e9;
		  	for (int i = 0; i < tvecs.size(); i++) {
				if (nearest_distance > tvecs[i](2)) {
			  		nearest_distance = tvecs[i](2);
			  		ret = i;
				}
		  }
			return ret; 	
		}
		
		
		
	public:
	
		BBM_Vision_Node() : Node("bbm_vision_node"){
			camera_matrix = cv::Matx33d::eye(); 
			
			tf2::Matrix3x3 basis;
			//basis = tf2::Matrix3x3(0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0); //ok if camera is normally posed, but in our case camera is rotated around z axis by 180°
			basis = tf2::Matrix3x3(0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
			ros2img.setBasis(basis);
			
			pose_aruco.setIdentity();
			fixed2rover.setIdentity();
			
			rclcpp::QoS custom_qos(10);
			
			auto sub_opt = rclcpp::SubscriptionOptions();

        		imgSub = create_subscription<sensor_msgs::msg::Image>( "/zed/zed_node/left/image_rect_color", custom_qos, std::bind(&BBM_Vision_Node::leftImageCallback, this, std::placeholders::_1), sub_opt);
        		camInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>( "/zed/zed_node/left/camera_info", custom_qos, std::bind(&BBM_Vision_Node::cameraCalibrationCallback, this, std::placeholders::_1), sub_opt);
        		poseSub = create_subscription<geometry_msgs::msg::PoseStamped>("/zed/zed_node/pose", custom_qos, std::bind(&BBM_Vision_Node::poseCallback, this, std::placeholders::_1), sub_opt);
        		paramsPub = create_publisher<bbm_interfaces::msg::Lineparams>  ("/bbm/line_params", custom_qos);
      			RCLCPP_INFO(this->get_logger(), "BBM_Vision_Node started");
		}
		
		/** Method that saves the pose for later computation **/
		void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
			tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,  msg->pose.orientation.w);

			// 3x3 Rotation matrix from quaternion
			tf2::Matrix3x3 m(q);

			fixed2rover.setRotation(q);
			fixed2rover.setOrigin(tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
		}
		
		/** Method that elaborates left image: scans for ArUco markers, if found then compute the position of the marker, get the line angular coefficient (from the marker id) and finally 
			sends the parameters of the line towards the robot has to converge **/
		void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
			
			try{
				//cv::Mat img;
				void *data = const_cast<void *>(reinterpret_cast<const void *>(&msg->data[0]));
  				cv::Mat img(msg->height, msg->width, CV_8UC4, data);
  				cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);
				//cv::rotate(img, img, 1);
				

			
				// Find ArUco markers 
				std::vector<int> markerIds;
				std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
				aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250); 
				aruco::detectMarkers(img, dictionary, markerCorners, markerIds);

				
				
				std::vector<cv::Vec3d> rvecs, tvecs;
				aruco::estimatePoseSingleMarkers( markerCorners, markerSize, camera_matrix, dist_coeffs, rvecs, tvecs);
				
				
				int closestMarker = getClosestMarker(tvecs);
				//RCLCPP_INFO(this->get_logger(), "Closest marker id: %d", closestMarker);
				if(closestMarker >= 0){
					double r = tvecs[closestMarker](2); 
					//RCLCPP_INFO(this->get_logger(), "Marker id: %d dist: %f x:%f y: %f, z:%f", markerIds[closestMarker], r, tvecs[closestMarker](0), tvecs[closestMarker](1), tvecs[closestMarker](2));
					if(r < maxDist){ 
						//RCLCPP_INFO(this->get_logger(), "Closest Marker id: %d", markerIds[closestMarker]);
						double m, q;
						tf2::Transform ros2aruco = getTransform(tvecs[closestMarker], rvecs[closestMarker]);
						tf2::Vector3 origin_rosCoord = ros2aruco*tf2::Vector3(0,0,0); //marker center in ros coordinates 
						
						// Line params computation
						tf2::Vector3 x_a = fixed2rover * ros2aruco * tf2::Vector3(0,0,0); //marker center in fixed frame coordinates 
						
						//RCLCPP_INFO(this->get_logger(), "Posa: (%f, %f), marker (%f, %f) (%f, %f)", x_a.x(), x_a.y());
						
						std::pair<bool, double> res = getAngularCoefficient(markerIds[closestMarker], ros2aruco);
						
						// if res.first == true, then the line can be represented as y = m*x +q
						if(res.first){
							m = res.second;
							q = x_a.y() - m * x_a.x();
						}else{
							m = 0;
							q = 0;
						}
						
						//RCLCPP_INFO(this->get_logger(), "Parametri retta: m %f q %f", pose[0], pose[1], pose[5]* 57.29578, m, q );
						
						// Lineparams message creation
						bbm_interfaces::msg::Lineparams msg;
						msg.x = x_a.x();
						msg.y = x_a.y();
						msg.m = m;
						msg.q = q;
						msg.vert = !res.first;
						msg.dx = (markerIds[closestMarker] == 0 || markerIds[closestMarker] == 2); //  direction of the line
						msg.end = (markerIds[closestMarker] == 4); // have we found the terminal ArUco?
						
						
						paramsPub -> publish(msg);
						
						//RCLCPP_INFO(this->get_logger(), "LeftImage");
					}
				}	
			}catch (cv_bridge::Exception& e){
				RCLCPP_INFO(this->get_logger(), "Immagine non ottenuta");
			}		
		}
		
		
		
		
		void cameraCalibrationCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
			camera_matrix(0, 0) = msg->k[0];
		  	camera_matrix(1, 1) = msg->k[4];
		  	camera_matrix(0, 2) = msg->k[2];
		  	camera_matrix(1, 2) = msg->k[5];
		  	//RCLCPP_INFO(this->get_logger(), "Camer Calib");
		}
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BBM_Vision_Node>());
    	rclcpp::shutdown();
    	return 0;
}
    
