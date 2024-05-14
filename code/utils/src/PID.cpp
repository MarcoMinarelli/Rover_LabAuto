#include <cmath>

/**	Class that implements a discrete time PID with Back Calculation and Tracking
**/
class PID{

	private:
	
		double kp;
		double ki;
		double kd;
		
		double deltaT;
		double Tt;
		double sum;
		double last_err;
		double U_max;
		double U_min;
		
		
	public:
	
		PID(double p, double i, double d, double deltaT, double Tt, double U_max,double U_min): kp(p), ki(i), kd(d), deltaT(deltaT), Tt(Tt), U_max(U_max), U_min(U_min){
			sum=0;
			last_err=0;
		}
		
	
		double compute(double e){
			double ret=kp*e;
			
			sum=sum+e;
			ret=ret+sum*ki;
			
			ret=ret+(e-last_err)/deltaT;
			last_err=e;
			
			if(ret>U_max){
				//double u_s=(ret>0) ? U:-U;
				ret=U_max;
			}
			if (ret<U_min) ret=U_min;
			return ret;
		}
		
		
};
   
