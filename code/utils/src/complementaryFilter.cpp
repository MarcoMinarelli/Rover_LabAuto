

class ComplementaryFilter{
	double lastout_l;
	double lastout_h;
	double lastin_h;
	double RC_l;
	double RC_h;
	double alpha_l;
	double alpha_h;
	
	
	public:
		ComplementaryFilter(double cutoff_freq_l, double cutoff_freq_h, double dt): lastout_l(0), lastout_h(0), lastin_h(0){
			RC_l=1.0/(cutoff_freq_l*2*3.14);
			alpha_l = dt/(RC_l+dt);
			RC_h=1.0/(cutoff_freq_h*2*3.14);
			alpha_h = RC_h/(RC_h+dt);
		};
		double update (double low_pass_input, double high_pass_input){
			double low_pass_res= lastout_l+ (alpha_l*(low_pass_input - lastout_l));
			lastout_l=low_pass_res;
		
			double high_pass_res = alpha_h * (lastout_h + high_pass_input - lastin_h);
			lastout_h=high_pass_res;
			lastin_h=high_pass_input;
			
			return low_pass_res + high_pass_res;	
		}
		
}; 
