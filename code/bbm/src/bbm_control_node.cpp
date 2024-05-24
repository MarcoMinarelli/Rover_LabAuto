// standard library includes 
#include <vector>
#include <cmath>

//ros2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//custom include
#include "bbm_interfaces/msg/lineparams.hpp"


using namespace std::chrono_literals;

/** Node that, given robot position, lidar measurements and line we want to converge to, comput linear velocity and angular velocity **/
/** References: [1] "A sliding mode based controller for trajectory tracking of perturbed unicycle mobile robots" M.Mera H.Rios E.A.Martinez
		[2] "Line following for an autonomous sailboat using potential fields method" F.Plumet H.Saoud M.D.Hua **/
		
class BBM_Control_Node : public rclcpp::Node{
	private:
		// subscriber 
		rclcpp::Subscription<bbm_interfaces::msg::Lineparams>::SharedPtr lpSub;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
		rclcpp::TimerBase::SharedPtr timer_;
		
		//Publisher 
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
		
		//sensor feedback
		std::vector<double> pose; //pose=[x_r, y_r] 
		double yaw;
		std::vector<float> laser;
		
		// parameters
		float m, q, x_a, y_a; // line params
		bool vert, dx, end;

		// APF parameters
		double r; // [m]
		double k_r; // repulsive gain
		double k_a; // attractive gain
		double d_inf; // [m]
		double delta; // threshold between quadratic potential and distance potential [m]
		double k_theta; // omega gain 
		
		//SMC control parameters
		double rho1, rho2; //SMC gains
		double delta1, delta2; // SMC f(e2) params
		double d1, d2, d3; //systems' disturbance
		double v_min, v_max; //linear velocity limits, in [m/s] 
		double omega_max; //angular velocity limits in [rad/s]
		
		
		double acc_bias = 0;
		double x_bias = 0;
		double y_bias = 0;

		
		bool ok = false;
		int count = 0;
		bool arucoReceived = false;
		int count_pose = 0;
		
		/** Method that compute the distance between two points **/
		double distance (std::vector<double> x1, std::vector<double> x2){
			return sqrt(pow(x1[0] - x2[0],2)+pow(x1[1]-x2[1],2));
		}
		
		
		/** Method that returns the intersection between convergence line and a circle with radius r centered in robot position. From [2]**/ 
		std::vector<double> intersection(){
			std::vector<double> ret(2,0);
			std::vector<double> x_aruco = {x_a, y_a};
			//RCLCPP_INFO(this->get_logger(), "x_a %f, y_a %f, vert %d", x_a, y_a, vert);
			if (vert==false){ // if the line is not vertical, i.e it can be represented as y=mx+q 
				double b=-2*pose[0]+2*m*q-2*m*pose[1];
				double a=1+pow(m,2);
				double c=pow(pose[0],2)+pow(q,2)+pow(pose[1],2)-2*q*pose[1]-pow(r,2);
				double delta=pow(b,2)-4*a*c;
				double xd=0;

				if(delta>0){
				
					double x1=(-b+sqrt(delta))/(2*a);
					double x2=(-b-sqrt(delta))/(2*a);
					
					std::vector<double> c1{x1, m*x1+q};
					std::vector<double> c2{x2, m*x2+q};
					
					if(distance(pose,x_aruco)>r)
						xd=(distance(c1, x_aruco)>distance(c2, x_aruco))?x1:x2;
					else{
										
						double m1=(c1[0]==0)? c1[1]/0.01 :  c1[1]/c1[0];
						double m2=(c2[0]==0)? c2[1]/0.01 :  c2[1]/c2[0];
					
						if(dx==true)
							xd=(m1>m2) ? m2:m1;
						else 
							xd=(m1>m2) ? m1:m2;		
					}
				}else if(delta==0){
				
					xd=-b/(2*a);
				
				}else {
					
					double m_s= -1/m;
					double q_s= pose[1]-m_s*pose[0];
					xd=(q_s-q)/(m-m_s);
				
				}
				
				double yd=m*xd+q;
				ret[0]=xd;
				ret[1]=yd;
			} else{ 
				double b=-2*pose[1];
				double c=pow(pose[0],2)+pow(x_a,2)-2*pose[0]*x_a+pow(pose[1],2)-pow(r,2);
				double delta=pow(b,2)-4*c;
				double yd=0;

				if(delta>0){
				
					double y1=(-b+sqrt(delta))/(2);
					double y2=(-b-sqrt(delta))/(2);
					
					std::vector<double> c1 = {x_a, y1};
					std::vector<double> c2 = {x_a, y2};
					

					
					if(distance(pose, x_aruco)>r)
						yd=(distance(c1, x_aruco )>distance(c2, x_aruco)) ? y1:y2;	
					else{
					
						double m1=(c1[0]==0)? c1[1]/0.01 :  c1[1]/c1[0];
						double m2=(c2[0]==0)? c2[1]/0.01 :  c2[1]/c2[0];
					
						if(dx==true)
							yd=(m1>m2) ? m2:m1;
						else 
							yd=(m1>m2) ? m1:m2;		
					}
						
				}else{
					yd=pose[1];
				}
				

				ret[0]=x_a;
				ret[1]=yd;
			}

			return ret;
		}
		
		/**Method that compute sign of a float**/
		int sign(double x){
			 return (x==0)?-2:x/fabs(x);
		}
		
		
		
		/** Method that returns the difference v1 - v2 **/
		std::vector<double> minus(std::vector<double> v1, std::vector<double> v2){
			std::vector<double> ret(v1.size(), 0);
			if(v1.size()==v2.size()){
				for(int i=0; i<v1.size(); i++)
					ret[i]=v1[i]-v2[i];
			}
			return ret;
		}
		
		/** Method that returns the sum v1 + v2 **/
		std::vector<double> sum(std::vector<double> v1, std::vector<double> v2){
			std::vector<double> ret(v1.size(), 0);
			if(v1.size()==v2.size()){
				for(int i=0; i<v1.size(); i++)
					ret[i]=v1[i]+v2[i];
			}
			return ret;
		}
		
		/** Method that returns the product between scalar s and vector v: s*v **/
		std::vector<double> scalar_prod(double s,std::vector<double> v){
			std::vector<double> ret(v.size(), 0);
			for(int i=0; i<v.size(); i++)
				ret[i]=s*v[i];
			return ret;
		}
		
		/** Method that computes the attractive force that drives the robot towards the goal **/
		std::vector<double> computeAttractiveForce(std::vector<double> goal){
			double dist= distance(pose, goal); 
			std::vector<double> f_attr(2,0);
			//RCLCPP_INFO(this->get_logger(), "goal %f %f pose %f %f ", goal[0], goal[1], pose[0], pose[1]);
			if(dist<=delta){
				f_attr=minus(goal, pose);
			}
			else{
				f_attr=scalar_prod(1/dist, minus(goal, pose));
			}
			return scalar_prod(k_a, f_attr);
		} 
			
		/** Method that computes the total repulsive force that drives the robot away from obstacles **/
		std::vector<double> computeRepulsiveForce(){
			std::vector<double> f_rep(2,0);
			for (int i=0; i<laser.size();i++){
				if (laser[i]<=d_inf){
					std::vector<double> f(2,0);
					double s=1/pow(laser[i],3)*(1/laser[i]-1/d_inf);

					std::vector<double> loc_obs{laser[i]*cos(i),laser[i]*sin(i)};
					std::vector<double> obs=sum(pose,loc_obs);
					f=minus(pose,obs);
					f=scalar_prod(s,f);
					f_rep=sum(f_rep,f);
				}

			}
			return scalar_prod(k_r, f_rep);
		}

	
	public:
		BBM_Control_Node() : Node("bbm_control_node"), pose(2,0), laser (360, 16.0){
			
			r=1.5;
			k_a=3;
			k_r=5;
			d_inf=1;
			delta=0.4;
			k_theta=3;
			delta1=0.5; // delta1 in (0, 1)
			delta2=1;   // delta2 > 0
			d1=0.1;
			d2= 0.002; // 0.01°/s in rad/s
			d3=0.001;
			v_min=0;   // 0 is the output of throttle of 0 - 0.2
			v_max=0.8; // [m/s]
			omega_max= 0.48; // experimental max angular velocity value (28°/s) in rad/s
			
			// initial line params
			x_a=0; 
			y_a=-6; 
			vert=true;
			dx=true;
			end=false;
			
			
			rho2 = computeRho2();
			rho1 = computeRho1(rho2);
			
			rclcpp::QoS custom_qos(10);
			auto sub_opt = rclcpp::SubscriptionOptions();
        		lpSub = create_subscription<bbm_interfaces::msg::Lineparams>( "/bbm/line_params", custom_qos, std::bind(&BBM_Control_Node::lineParamsCallback, this, std::placeholders::_1), sub_opt);
        		laserSub = create_subscription<sensor_msgs::msg::LaserScan>( "/scan", custom_qos, std::bind(&BBM_Control_Node::laserCallback, this, std::placeholders::_1), sub_opt);
        		poseSub = create_subscription<geometry_msgs::msg::PoseStamped>("/zed/zed_node/pose", custom_qos, std::bind(&BBM_Control_Node::poseCallback, this, std::placeholders::_1), sub_opt);
  			timer_ = this->create_wall_timer(50ms, std::bind(&BBM_Control_Node::controlCallback, this));
  			
  			
  			vel_pub= create_publisher<geometry_msgs::msg::Twist> ("/cmd_vel", custom_qos);
      			RCLCPP_INFO(this->get_logger(), "BBM_Control_Node started");
		}
		
		/** Callback for Pose message. Estimates velocity along x-axis from pose and relative derivative**/
		// Pose rate: 100 Hz
		void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
			
			if(count_pose < 60){
				x_bias += msg->pose.position.x;
				y_bias += msg->pose.position.y;

				count_pose++;
			}else{
				if(count == 60){
					x_bias = x_bias/count_pose;
					y_bias = y_bias/count_pose;

				}		
						
				pose[0] =  msg->pose.position.x - x_bias;
				pose[1] =  msg->pose.position.y - y_bias;
				tf2::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
				tf2::Matrix3x3 m(q);
				double r, p;
				m.getRPY(r, p, yaw);
				
				ok = true;
				//RCLCPP_INFO(this->get_logger(), " %f %f %f ",  msg->pose.position.x - x_bias,  msg->pose.position.y - y_bias, yaw - yaw_bias);
			}
		}
		
		/** Method that stores the parameters of the line towards which the robot has to converge **/
		void lineParamsCallback(const bbm_interfaces::msg::Lineparams::SharedPtr msg){
			m=msg->m;
			q=msg->q;
			x_a=msg->x;
			y_a=msg->y;
			dx=msg->dx;
			vert=msg->vert;
			end=msg->end;
			arucoReceived = true;
			//RCLCPP_INFO(this->get_logger(), "x_a %f, y_a %f, m %f, q %f, dx %d", x_a, y_a, m, q, dx);
		}
		
		/** Method that stores the LiDar ranges for later usage **/
		void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
			RCLCPP_INFO(this->get_logger(), "raga sono dentro");
			for (int i=0;i<360 ;i++){
				laser[i]=msg->ranges[i];
				RCLCPP_INFO(this->get_logger(), "raga sono dentro");
			}
		}
		/** function used in SMC. If e2 ==0, then 1/|e2| = inf and so min is delta2 **/
		double f(double e2){ 
			if (e2==0) return delta2;
			double v = delta1*(1/fabs(e2))<delta2? delta1*(1/fabs(e2)) : delta2 ;
			return v*(-e2); 	
		}
		
		/** From [1] **/
		double computeRho1(double rho2){
			double c1=((delta1+(d1-d3)*sqrt(1-pow(delta1,2)))/(1+d3-d1))*v_max;
			double c2=((1+d2)*(omega_max+rho2)*(delta1/delta2)+d2*v_max)/(1-d1);
			double r1=(c1>c2) ? c1:c2;
			
			double c3=((delta1-d3*sqrt((1-pow(delta1, 2)))/d3)*v_min);
			double c4=(delta1/d3)*v_min-v_max*sqrt(1-pow(delta1,2));
			double r2=(c3<c4) ? c3:c4;
			
			return (r1+r2)/2; // r1<rho1<r2 so we choose the average	
		}
		
		/** From [1] **/
		double computeRho2(){
			double r3=((d2*omega_max*sqrt(1-pow(delta1,2))+delta2*v_max+delta1*v_min))/((1-d2)*sqrt(1-pow(delta1,2)));
			return r3+0.2; //rho2>r3 so we sum 0.2
					
		}
		
		/** Method that compute the linear and angular velocity in order to converge to the line **/
		void controlCallback(){				
			if (end==true || count < 30){
				geometry_msgs::msg::Twist t_msg;
				t_msg.linear.x=0;
				t_msg.angular.z=0;
				//RCLCPP_INFO(this->get_logger(), " vel pub %f ",  msg.linear.x);
				vel_pub -> publish(t_msg);
				count++;
			} else{
				if(ok){
					std::vector<double> goal= arucoReceived ? intersection() : std::vector<double>{pose[0]+0.3, pose[1]} ;
					std::vector<double> f_attr=computeAttractiveForce(goal);
					std::vector<double> f_rep= computeRepulsiveForce();
					std::vector<double> f_tot= sum(f_attr,f_rep);
					
					
					double v_d = f_tot[0]* cos(yaw)+ f_tot[1]* sin(yaw);
					double omega_d = k_theta*(atan2(f_tot[1], f_tot[0])-yaw);

					//SMC
					double u1=-rho1*sign(goal[0]-pose[0]);
					double u2=-rho2*sign(atan2(f_tot[1], f_tot[0])-yaw-asin(f(goal[1]-pose[1])));
					
					double v=v_d*cos(atan2(f_tot[1], f_tot[0])-yaw)-u1;
					double omega=omega_d-u2;
					
					//double v = v_d;
					//double omega = omega_d;
					RCLCPP_INFO(this->get_logger(), "ftot (%f, %f)  omega %f v %f ",f_tot[0], f_tot[1], omega, v);
					geometry_msgs::msg::Twist msg;
					msg.linear.x=v;
					msg.angular.z=omega;
					vel_pub -> publish(msg); 
				}
			}
		}
};


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BBM_Control_Node>());
    	rclcpp::shutdown();
    	return 0;
}
