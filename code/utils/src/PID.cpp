#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "writecsv.cpp"

/**	Class that implements a discrete time PID with Back Calculation and Tracking
**/
class PID{

	private:
	
		double kp;
		double ki;
		double kd;
		
		double deltaT;

		double sum;
		double last_err;
		double U_max;
		double U_min;
		

		WriteCSV file{"pid.csv"};
		
	public:
	
		PID(double p, double i, double d, double deltaT, double U_max,double U_min): kp(p), ki(i), kd(d), deltaT(deltaT), U_max(U_max), U_min(U_min){
			sum=0;
			last_err=0;
		}
		
	
		double compute(double e){
			//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error PID %f ", e);
			double ret=kp*e;
			

			
			double new_sum;
			new_sum=sum+e;
			ret=ret+new_sum*ki*deltaT;
			

			
			ret=ret+ kd * (e-last_err)/deltaT;
			

			std::vector<double> v(5,0);
                                        v[0] = kp*e;
                                        v[1] = new_sum*ki*deltaT;
                                        v[2] = kd * (e-last_err)/deltaT;
                                        v[3] = ret;
                                        v[4] = e;
                                        file.writeData (v);

			last_err=e;
			//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "risultato prop %f risultato int %f risultato der %f risultato PID %f prima sat", kp*e, new_sum*ki*deltaT, kd * (e-last_err)/deltaT,   renew_sum*ki*deltaTt);
			
		

			if(ret>U_max) ret=U_max;
			else if (ret<U_min) ret=U_min;
			else  sum=new_sum;
			return ret;
		}
		
		
};
   
