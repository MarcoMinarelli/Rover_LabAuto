
/** Method that implements a Complementary Filter. The gain can be both directly set or one can pass the sampling time and cutoff frequency **/
class ComplementaryFilter{
	private:
		double alpha;
		
	public:
		/** Constructor from cutoff frequency and sampling time **/
		ComplementaryFilter(double cutoff_freq, double dt){
			double RC=1.0/(cutoff_freq*2*3.14);
			alpha = RC/(RC+dt);
		};
		
		/** Constructor directly from gain **/
		ComplementaryFilter(double alpha): alpha(alpha){};
		
		double update (double low_pass_input, double high_pass_input){
			return alpha * low_pass_input + (1-alpha) * high_pass_input;	
		}
		
}; 
