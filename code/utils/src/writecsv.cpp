#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>

/** Class that writes values in a file (whose path is given in constructor) in csv style**/
class WriteCSV {

public:
	std::string fileName;
	std::ofstream csv_file;

	WriteCSV(std::string filename) : fileName(filename) { this->csv_file.open(this->fileName); }

	/**Method that writes the given list in the file previously open, every value separated by commas **/
	void writeData(std::vector<double> list) {
		for (int i = 0; i < list.size()-1; ++i) {
        		this->csv_file << std::setprecision(5) << list[i]<<",";
		};
		this->csv_file << std::setprecision(5) << list[list.size() - 1]<<std::endl;
	}

	void close_csv() { this->csv_file.close(); }
};	
