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
		double U;
		
		
	public:
	
		PID(double p, double i, double d, double deltaT, double Tt, double U): kp(p), ki(i), kd(d), deltaT(deltaT), Tt(Tt), U(U){
			sum=0;
			last_err=0;
		}
		
	
		double compute(double e){
			double ret=kp*e;
			
			sum=sum+e;
			ret=ret+sum*ki;
			
			ret=ret+(e-last_err)/deltaT;
			last_err=e;
			
			if(fabs(ret)>U){
				double u_s=(ret>0) ? U:-U;
				ret=ret+(u_s-ret)/Tt;
			}
			return ret;
		}
		
		
};
   
