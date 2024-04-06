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
		
		sclcpp::Publisher<bbm_interfaces::msg::Lineparams>::SharedPtr paramsPub;
		
		std::vector<double> pose; // pose = [x_r, y_r, z_r, roll, pitch, yaw]
		
		// Camera data
		cv::Matx33d camera_matrix;
		cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros(); 
		
		// Node params
		float maxDist;
		int markerSize; //[cm]
		
		
		
		
		std::pair<bool, double> getParallelLineAngularCoefficient(std::vector<double> x1, std::vector<double> x2){
			bool ok;
			double m;
			if (x1[1] == x2[1]){
				ok = true;
				m = 0;
			}
			else if(x1[0] == x2[0]){
				ok = false;
				m = 0;
			}else{
				ok = true;
				m = (x1[1] - x2[1])/(x1[0] - x2[0]);			
			}	
			return std::pair<bool, double>(ok, m);
		}
		
		
		/** Method that compute upper marker corners' position, the slope of line that passes through both of them and, depending on the marker id, computes the slope of the new line. Returns a 
			pair where the first element indicates if the line can be expressed as y = m*x + q or not. This is needed because vertical lines cannot be represented this way.
		**/
		std::pair<bool, double> getAngularCoefficient(int id, cv::Vec3d t, cv::Vec3d c){
			std::pair<bool, double> ret;
			cv::Mat cv_rot(3, 3, CV_64F);
			cv::Rodrigues(r, cv_rot);
			
			// Top right corner in ArUco coordinate frame: (size/2, size/2, 0) while top left is (-size/2, -size/2, 0). In order to get position in camera coordinate frame, 
			// we compute R * point_ArUco_frame + t
			cv::Vec3d topRightCorner = cv_rot.t() * cv::Vec3d(markerSize/2, markerSize/2, 0) + t;
			cv::Vec3d topLeftCorner = cv_rot.t() * cv::Vec3d(-markerSize/2, -markerSize/2, 0) + t;
			
			// compute yaw angle between robot coordinate system and corners
			double angleRight = getAngle(topRightCorner, c);
			double angleLeft = getAngle(topLeftCorner, c);
			
			// computing  2D distance between robot and corners
			double rRight = sqrt(topRightCorner[0] * topRightCorner[0] + topRightCorner[1] * topRightCorner[1]);
			double rLeft = sqrt(topLeftCorner[0] * topLeftCorner[0] + topLeftCorner[1] * topLeftCorner[1]);
			
			// computing corners position in fixed frame
			std::vector<double> leftPoint(pose[0] + rLeft*cos(pose[5] + angleLeft), pose[1] + rLeft*sin(pose[5] + angleLeft));
			std::vector<double> rightPoint(pose[0] + rRight*cos(pose[5] + angleRight), pose[1] + rRight*sin(pose[5] + angleRight));
			
			//Computing the slope of the line that contains both markers' corners. If the marker id is 0 or 1, we'll use this
			ret = getParallelLineAngularCoefficient(leftPoint, rightPoint);
			
			if(id == 2){ //desired line forms 45° respect to ArUco
				if(ret.first && ret.second != 1){ //if the line is not vertical and has slope !=1 
					m = ((ret.second + 1)/(1 - ret.second)); // m is the tangent of the line parallel to ArUco, we want it rotated by 45°, tan(45°) = 1, the we apply the sum of tangent formula
					ok = true;
				}else if(ret.first && ret.second == 1){ // if the line between corners is not vertical and is already at 45° => the resulting line is parallel to y axis
					ok = false;
					m = 0;
				}else{ // the line between corners is vertical => the slope of desired line is 1
					ok = true;
					m = 1;
				}
			}else if(id == 3){ // line that forms an angle of 135° wrt marker				
				if(ret.first && ret.second  != -1){ // line not vertical and marker has slope != -1 
					ok = true;
					m = (ret.second - 1)/(1 + ret.second); // same concept as case with id = 2
				}else if(ret.first && ret.second  == -1){ // line not vertical, marker has slope -1 => resulting line parallel to y axis
					ok = false;
					m = 0;
				}else{ // line vertical => desired line has slope -1
					ok = true;
					m = -1;
				}
			}else if(id == 5){
				ok = true;
				m = 0;
			}
			return  (id == 0 || id == 1)? ret : std::pair<bool, double>(ok, m);
		}
		
		
		/** Method that given a translation and rotation compute the angle (on xy plane) of the object wrt robots' x axis **/
		double getAngle(cv::Vec3d t, cv::Vec3d r){
		
			tf2::Transform img2aruco;
			tf2::Transform aruco2img;
			
			double r, p, y;
			tf2::Matrix3x3 basis;
			basis = tf2::Matrix3x3(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
			img2aruco.setBasis(basis);
			aruco2img = img2aruco.inverse();
  
			tf2::Vector3 tf2_origin(t[0], t[1], t[2]);
			cv::Mat cv_rot(3, 3, CV_64F);
			cv::Rodrigues(r, cv_rot);
				  
			tf2::Matrix3x3 tf2_rot(cv_rot.at<double>(0, 0), cv_rot.at<double>(0, 1), cv_rot.at<double>(0, 2), 
			cv_rot.at<double>(1, 0), cv_rot.at<double>(1, 1), cv_rot.at<double>(1, 2), cv_rot.at<double>(2, 0), 
			cv_rot.at<double>(2, 1), cv_rot.at<double>(2, 2));

			tf2::Transform pose_aruco(tf2_rot, tf2_origin);
				
			tf2::Transform pose_img;
			pose_img.mult(_img2aruco, pose_aruco);
			pose_img = pose_img.inverse();
			pose_img.getBasis().getRPY(r, p, y); // get rpy angles
			return y;		
		}
		
		/** Method that given the transaltion of a lsit of markers returns the index of closest one **/
		size_t getClosestMarker(std::vector<cv::Vec3d> tvecs){
			size_t ret = 999;
		  	double nearest_distance = 1e9;
		  	for (size_t i = 0; i < ids.size(); i++) {
				double x = tvecs[i](0);
				double y = tvecs[i](1);
				double z = tvecs[i](2);
				double current_distance = sqrt(x * x + y * y + z * z);
				if (nearest_distance > current_distance) {
			  		nearest_distance = current_distance;
			  		ret = i;
				}
		  }
			return ret; 	
		}
		
		
		
	public:
	
		BBM_Vision_Node() : Node("bbm_vision_node"), pose(6,0), maxDist(0.5){
			camera_matrix = cv::Matx33d::eye(); 
			markerSize = 20;
			
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

			// Roll Pitch and Yaw from rotation matrix
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
		    	
		    pose[0] =  msg->pose.position.x;
		    pose[1] =  msg->pose.position.y;
		    pose[2] =  msg->pose.position.z;
		    	
		    pose[3] =  roll;
		    pose[4] =  pitch;
		    pose[5] =  yaw;
		}
		
		/** Method that elaborates left image: scans for ArUco markers, if found then compute the position of the marker, get the line angular coefficient (from the marker id) and finally 
			sends the parameters of the line towards the robot has to converge **/
		void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
			cv_bridge::CvImagePtr cv_ptr;
			try{
				cv::Mat img;
				rotate((cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)) -> image, img, 1); // rotate image 180°
			
				// Find ArUco markers 
				std::vector<int> markerIds;
				std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
				aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250); //FIXME quale dizionario?
				aruco::detectMarkers(img, dictionary, markerCorners, markerIds);
				
				std::vector<cv::Vec3d> rvecs, tvecs;
				aruco::estimatePoseSingleMarkers( markerCorners, markerSize, camera_matrix, dist_coeffs, rvecs, tvecs);
				
				size_t closestMarker = getClosestMarker(tvecs);
				if(sqrt(tvecs[closestMarker](0) * tvecs[closestMarker](0) + tvecs[closestMarker](1) * tvecs[closestMarker](1)) < maxDist){ 
					double m, q;
					double markerAngle = getAngle(tvecs[closestMarker], rvecs[closestMarker]);
					
					
					// Line params computation
					double r = sqrt(tvecs[closestMarker](0) * tvecs[closestMarker](0) + tvecs[closestMarker](1) * tvecs[closestMarker](1) ); // distance in 2d between robot and marker
					double x_a = pose[0] + r*cos(pose[5] + markerAngle);
					double y_a = pose[1] + r*sin(pose[5] + markerAngle);
					

					std::pair<bool, double> res = getAngularCoefficient(markerIds[closestMarker], tvecs[closestMarker], cvecs[closestMarker]);
					// if res.first == true, then the line can be represented as y = m*x +q
					if(res.first){
						m = res.second;
						q = y_a - m * x_a;
					}else{
						m = 0;
						q = 0;
					}
					// Lineparams message creation
					bbm_interfaces::msg::Lineparams msg;
					msg.x = x_a;
					msg.y = y_a;
					msg.m = m;
					msg.q = q;
					msg.vert = !res.first;
					msg.dir = (id == 0 || id == 2 || id == 3) ? true: false; // direction of the line
					msg.end = (id == 5); // have we found the terminal ArUco?
					
					
					paramsPub.publish(msg);
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
    
