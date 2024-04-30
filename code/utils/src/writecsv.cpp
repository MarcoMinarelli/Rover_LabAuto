#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>

class WriteCSV {

public:
	std::string fileName;
	std::ofstream csv_file;

	WriteCSV(std::string filename) : fileName(filename) { this->csv_file.open(this->fileName); }


	void writeData(std::vector<double> list) {
		for (int i = 0; i < list.size()-1; ++i) {
        		this->csv_file << std::setprecision(11) << list[i]<<",";
		};
		this->csv_file << std::setprecision(11) << list[list.size()]<<"\n";
	}

	void close_csv() { this->csv_file.close(); }
};	
