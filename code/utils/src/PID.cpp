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
		
		double sum_bct; 
		
		
	public:
	
		PID(double p, double i, double d, double deltaT, double Tt, double U_max,double U_min): kp(p), ki(i), kd(d), deltaT(deltaT), Tt(Tt), U_max(U_max), U_min(U_min){
			sum=0;
			last_err=0;
			sum_bct = 0;
		}
		
		PID(double p, double i, double d, double deltaT, double U_max, double U_min): kp(p), ki(i), kd(d), deltaT(deltaT), Tt(Tt), U_max(U_max), U_min(U_min){
			sum=0;
			last_err=0;
			sum_bct = 0;
			Tt = sqrt( kd/ki ); // automatic calculation of Tt as sqrt(Td * Ti)
		}
		
	
		double compute(double e){
			double ret=kp*e;
			
			sum=sum+e;
			ret=ret+sum*ki*deltaT;
			
			ret=ret+(e-last_err)/deltaT;
			last_err=e;
			
			if(ret>U_max){
				sum_bct = sum_bct + (U_max-ret);
				ret= ret + 1/Tt * deltaT * sum_bct;
			}
			if (ret<U_min) ret=U_min;
			return ret;
		}
		
		
};
   
