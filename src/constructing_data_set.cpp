#include <constructing_data_set.h>
const int SAMPLES_DIM = 15;


const char *Collision_path="/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/colision.txt";

const char *Position11_path="/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Position11.txt";
const char *Position12_path="/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Position12.txt";
const char *Position13_path="/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Position13.txt";
const char *Position21_path="/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Position21.txt";
const char *Position22_path="/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Position22.txt";
const char *Position23_path="/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Position23.txt";

const char *Theta1_path="/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Theta1.txt";
const char *Theta2_path="/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Theta2.txt";





int main(int argc, char** argv)
{

	/*
	 * NOTE: The matrices should be N*D, where N is the number of the data points and D is the dimention
	 *
	 * 			%% %%%%%%%%%%%%%%%%%%%%%%%%%%
			%  Loading the Data points	%
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%			*/
	MatrixXd Position11(1,1);MatrixXd Position12(1,1);MatrixXd Position13(1,1);
	MatrixXd Position21(1,1);MatrixXd Position22(1,1);MatrixXd Position23(1,1);

	MatrixXd Theta1(1,1);MatrixXd Theta2(1,1);

	Position11=readMatrix(Position11_path);
	Position12=readMatrix(Position12_path);
	Position13=readMatrix(Position13_path);
	Position21=readMatrix(Position21_path);
	Position22=readMatrix(Position22_path);
	Position23=readMatrix(Position23_path);

	Theta1=readMatrix(Theta1_path);
	Theta2=readMatrix(Theta2_path);


	cout <<"The size of theta1 is "<< Theta1.rows()<<" * "<<Theta1.cols()<<endl;
	cout <<"The size of Theta2 is "<< Theta2.rows()<<" * "<<Theta2.cols()<<endl;
	cout <<"The size of Position11 is "<< Position11.rows()<<" * "<<Position11.cols()<<endl;
	cout <<"The size of Position12 is "<< Position12.rows()<<" * "<<Position12.cols()<<endl;
	cout <<"The size of Position13 is "<< Position13.rows()<<" * "<<Position13.cols()<<endl;
	cout <<"The size of Position21 is "<< Position21.rows()<<" * "<<Position21.cols()<<endl;
	cout <<"The size of Position22 is "<< Position22.rows()<<" * "<<Position22.cols()<<endl;
	cout <<"The size of Position23 is "<< Position23.rows()<<" * "<<Position23.cols()<<endl;


	MatrixXd Position1(Position11.rows()+Position12.rows()+Position13.rows(),Position11.cols());
	MatrixXd Position2(Position21.rows()+Position22.rows()+Position23.rows(),Position21.cols());

	Position1 << Position11,
			Position12,
			Position13;
	Position2 << Position21,
			Position22,
			Position23;
	cout <<"The size of Position1 is "<< Position1.rows()<<" * "<<Position1.cols()<<endl;
	cout <<"The size of Position2 is "<< Position2.rows()<<" * "<<Position2.cols()<<endl;
	/*			%% %%%%%%%%%%%%%%%%%%%%%%%%%%
				%  Fidning_The_Collisions	%
				%%%%%%%%%%%%%%%%%%%%%%%%%% %%			*/



	MatrixXd Data_Complete(2*Position1.rows(),1+1+1+Theta1.cols()+Theta2.cols());
	MatrixXd Data_With_out_collision(0,6);
	MatrixXd Data_With_collision(0,6);
	VectorXd Dummy(6);


	int dummy_dimention=Position1.cols();
	const std::size_t num_results = Position2.rows()-1;
	int Joint_Robot1=0;
	int Joint_Robot2=0;
	for (int i=0; i<2;i++)
	{
		cout<<"i "<<i<<endl;
		Joint_Robot1=i/Position1.rows();
		std::vector<double> query_pt(dummy_dimention);

		//		cout<<"query_pt ";
		for (int j=0; j<Position1.cols();j++)
		{
			query_pt[j]=Position1(i,j);
			//		cout<<"| "<<query_pt[j]<<"| "<<endl;
		}

		typedef KDTreeEigenMatrixAdaptor< Eigen::Matrix<double,Dynamic,Dynamic> > my_kd_tree_t;
		my_kd_tree_t mat_index(dummy_dimention, Position2, 20 );
		mat_index.index->buildIndex();

		std::vector<long unsigned int>   ret_indexes(num_results);
		std::vector<double> 			 out_dists_sqr(num_results);
		nanoflann::KNNResultSet<double> resultSet(num_results);
		resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
		mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));


		for (int j=0; j<num_results;j++)
		{
			cout<<"j "<<j<<endl;
			Joint_Robot2=j/Position21.rows();
			Data_Complete(i+j,1)=Joint_Robot1;
			Data_Complete(i+j,2)=Joint_Robot2;
			if ( out_dists_sqr[j]<0.5)
			{
				Data_Complete(i+j,0)=1;
			}
			else
			{
				Data_Complete(i+j,0)=0;
			}
			for (int ii=0;ii<Theta1.cols() ;ii++)
			{
				Data_Complete(i+j,3+ii)=Theta1(i,ii);
				Data_Complete(i+j,3+Theta1.cols()+ii)=Theta2(j,ii);
			}
		}
	}

	cout<<"Data_Complete "<<Data_Complete<<endl;
	/*	std::cout << "knnSearch(nn="<<num_results<<"): \n";
		for (int	 i=0;i<num_results;i++)
			std::cout << "ret_index["<<i<<"]=" << ret_indexes[i] << " out_dist_sqr=" << out_dists_sqr[i] << endl;
		cout<<Position2(ret_indexes[i],0)<<" "<<Position2(ret_indexes[i],1)<<" "<<Position2(ret_indexes[i],2)<<endl;
	 */





	/*
	cout<<"First Step passed"<<endl;

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


	cout <<"The size of Data is "<< Data_Complete.rows()<<" * "<<Data_Complete.cols()<<endl;
	cout <<"The size of Data with out collision is "<< Data_With_out_collision.rows()<<" * "<<Data_With_out_collision.cols()<<endl;
	cout <<"The size of Data with collision is "<< Data_With_collision.rows()<<" * "<<Data_With_collision.cols()<<endl;

	int dummy_dimention=Data_With_out_collision.cols();
	int i=0;
	for (int i=0; i<Data_With_out_collision.rows();i++)
	{

	std::vector<double> query_pt(dummy_dimention);

	cout<<"query_pt ";
	for (int j=0; j<Data_With_out_collision.cols();j++)
	{
		query_pt[j]=Data_With_collision(i,j);
		cout<<"| "<<query_pt[j]<<"| "<<endl;
	}

	typedef KDTreeEigenMatrixAdaptor< Eigen::Matrix<double,Dynamic,Dynamic> > my_kd_tree_t;
	my_kd_tree_t mat_index(dummy_dimention, Data_With_out_collision, 10 );
	mat_index.index->buildIndex();

	std::vector<long unsigned int>   ret_indexes(num_results);
	std::vector<double> out_dists_sqr(num_results);
	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
	mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

	std::cout << "knnSearch(nn="<<num_results<<"): \n";
	for (int	 i=0;i<num_results;i++)
		std::cout << "ret_index["<<i<<"]=" << ret_indexes[i] << " out_dist_sqr=" << out_dists_sqr[i] << endl;

	}

	 */


	return 0;
}
