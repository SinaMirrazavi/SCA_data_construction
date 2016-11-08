


#include "eigen3/Eigen/Dense"



#include <ctime>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include "ros/ros.h"
#include  <omp.h>

#define MAXBUFSIZE  ((int) 1e6)

int Num_of_robots=2;


namespace Eigen{
template<class Matrix>
void write_binary(const char* filename, const Matrix& matrix){
	std::ofstream out(filename,ios::out | ios::binary | ios::trunc);
	typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
	out.write((char*) (&rows), sizeof(typename Matrix::Index));
	out.write((char*) (&cols), sizeof(typename Matrix::Index));
	out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
	out.close();
}
template<class Matrix>
void read_binary(const char* filename, Matrix& matrix){
	std::ifstream in(filename,ios::in | std::ios::binary);
	typename Matrix::Index rows=0, cols=0;
	in.read((char*) (&rows),sizeof(typename Matrix::Index));
	in.read((char*) (&cols),sizeof(typename Matrix::Index));
	matrix.resize(rows, cols);
	in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
	in.close();
}
}


string addTwochar(string folder_path, string b, int integer1,int integer2=-1)
{
	std::string str;
	str.append(folder_path);
	str.append(b);
	ostringstream convert;
	convert << integer1;
	str.append(convert.str());
	if (integer2!=-1)
	{
		ostringstream convert;
		convert << integer2;
		str.append(convert.str());
	}
	str.append(".txt");
	return str;
}



string buffer_path;

string folder_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data";
