

class ComplementaryFilter{
	private:
		double alpha;
		
	public:
		ComplementaryFilter(double cutoff_freq, double dt){
			double RC=1.0/(cutoff_freq*2*3.14);
			alpha = RC/(RC+dt);
		};
		
		ComplementaryFilter(double alpha): alpha(alpha){};
		
		double update (double low_pass_input, double high_pass_input){
			return alpha * low_pass_input + (1-alpha) * high_pass_input;	
		}
		
}; 
