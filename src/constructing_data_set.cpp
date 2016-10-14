#include <constructing_data_set.h>
const int SAMPLES_DIM = 15;


const char *Collision_path="/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/colision.txt";
int main(int argc, char** argv)
{

	MathLib::Matrix Data_RTK;
	Data_RTK.Load(Collision_path);
	MatrixXd Data_Complete(Data_RTK.RowSize(),Data_RTK.ColumnSize());
	MatrixXd Data_With_out_collision(0,6);
	MatrixXd Data_With_collision(0,6);
	VectorXd Dummy(6);


	for (int i=0;i<Data_RTK.RowSize();i++)
	{
		for (int j=0;j<Data_RTK.ColumnSize();j++)
		{
			Data_Complete(i,j)=Data_RTK(i,j);
		}
		if (Data_RTK(i,0)==0)
		{
			Data_With_out_collision.conservativeResize(Data_With_out_collision.rows()+1, Data_With_out_collision.cols());
			Data_With_out_collision.row(Data_With_out_collision.rows()-1)=Data_Complete.block(i,3,1,6);
		}
		else
		{
			Data_With_collision.conservativeResize(Data_With_collision.rows()+1, Data_With_collision.cols());
			Data_With_collision.row(Data_With_collision.rows()-1)=Data_Complete.block(i,3,1,6);

		}
	}
	cout<<"Data_With_out_collision "<<Data_With_out_collision<<endl;
	cout<<"Data_With_collision "<<Data_With_collision<<endl;
	cout<<"Data_Complete "<<Data_Complete<<endl;

	return 0;
}
