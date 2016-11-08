#include <analysing_data_set.h>
const int SAMPLES_DIM = 15;


const char *Collision_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/datacolision.txt";

const char *Position10_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position00.txt";
const char *Position11_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position01.txt";
const char *Position12_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position02.txt";
const char *Position13_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position03.txt";
const char *Position14_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position04.txt";
const char *Position15_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position05.txt";

const char *Position20_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position10.txt";
const char *Position21_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position11.txt";
const char *Position22_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position12.txt";
const char *Position23_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position13.txt";
const char *Position24_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position14.txt";
const char *Position25_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position15.txt";


const char *Theta1_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Theta0.txt";
const char *Theta2_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Theta1.txt";


bool Save_complete_data_set=false;
MatrixXd Pairwisedistance(MatrixXd A, MatrixXd B)
{

	MatrixXd result(A.rows(),B.rows());
	VectorXd A_vector(A.cols());
	VectorXd B_vector(B.cols());
#pragma omp parallel num_threads(8)
	{
#pragma omp for
		for (int i=0;i<A.rows();i++)
		{
			for (int j=0;j<B.rows();j++)
			{
				result(i,j)=(A.block(i,0,1,A.cols()).transpose()-B.block(j,0,1,B.cols()).transpose()).norm();
			}
		}
	}

	return result;
}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_manager");
	ros::NodeHandle n;
	/*
	 * NOTE: The matrices should be N*D, where N is the number of the data points and D is the dimention
	 *
	 * 		%% %%%%%%%%%%%%%%%%%%%%%%%%%%
			%  Loading the Data points	%
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%			*/

	MathLib::Matrix Position10_RTK;	Position10_RTK.Load(Position10_path);
	MathLib::Matrix Position11_RTK;	Position11_RTK.Load(Position11_path);
	MathLib::Matrix Position12_RTK;	Position12_RTK.Load(Position12_path);
	MathLib::Matrix Position13_RTK;	Position13_RTK.Load(Position13_path);
	MathLib::Matrix Position14_RTK;	Position14_RTK.Load(Position14_path);
	MathLib::Matrix Position15_RTK;	Position15_RTK.Load(Position15_path);

	MathLib::Matrix Position20_RTK; Position20_RTK.Load(Position20_path);
	MathLib::Matrix Position21_RTK;	Position21_RTK.Load(Position21_path);
	MathLib::Matrix Position22_RTK;	Position22_RTK.Load(Position22_path);
	MathLib::Matrix Position23_RTK;	Position23_RTK.Load(Position23_path);
	MathLib::Matrix Position24_RTK;	Position24_RTK.Load(Position24_path);
	MathLib::Matrix Position25_RTK;	Position25_RTK.Load(Position25_path);

	MathLib::Matrix Theta1_RTK;	Theta1_RTK.Load(Theta1_path);
	MathLib::Matrix Theta2_RTK;	Theta2_RTK.Load(Theta2_path);

	MatrixXd Position10(Position10_RTK.RowSize(),Position10_RTK.ColumnSize());MatrixXd Position11(Position10_RTK.RowSize(),Position10_RTK.ColumnSize());MatrixXd Position12(Position10_RTK.RowSize(),Position10_RTK.ColumnSize());MatrixXd Position13(Position10_RTK.RowSize(),Position10_RTK.ColumnSize());
	MatrixXd Position14(Position10_RTK.RowSize(),Position10_RTK.ColumnSize());MatrixXd Position15(Position10_RTK.RowSize(),Position10_RTK.ColumnSize());

	MatrixXd Position20(Position20_RTK.RowSize(),Position20_RTK.ColumnSize());MatrixXd Position21(Position20_RTK.RowSize(),Position20_RTK.ColumnSize());MatrixXd Position22(Position20_RTK.RowSize(),Position20_RTK.ColumnSize());MatrixXd Position23(Position20_RTK.RowSize(),Position20_RTK.ColumnSize());
	MatrixXd Position24(Position20_RTK.RowSize(),Position20_RTK.ColumnSize());MatrixXd Position25(Position20_RTK.RowSize(),Position20_RTK.ColumnSize());

	MatrixXd Theta1(Theta1_RTK.RowSize(),Theta1_RTK.ColumnSize());MatrixXd Theta2(Theta2_RTK.RowSize(),Theta2_RTK.ColumnSize());


	cout<<"Loading is finished"<<endl;


	Position10=M2E_m(Position10_RTK).transpose();
	Position11=M2E_m(Position11_RTK).transpose();
	Position12=M2E_m(Position12_RTK).transpose();
	Position13=M2E_m(Position13_RTK).transpose();
	Position14=M2E_m(Position14_RTK).transpose();
	Position15=M2E_m(Position15_RTK).transpose();


	Position20=M2E_m(Position20_RTK).transpose();
	Position21=M2E_m(Position21_RTK).transpose();
	Position22=M2E_m(Position22_RTK).transpose();
	Position23=M2E_m(Position23_RTK).transpose();
	Position24=M2E_m(Position24_RTK).transpose();
	Position25=M2E_m(Position25_RTK).transpose();

	Theta1=M2E_m(Theta1_RTK).transpose();
	Theta2=M2E_m(Theta2_RTK).transpose();


	Position10_RTK.Resize(0,0);
	Position11_RTK.Resize(0,0);
	Position12_RTK.Resize(0,0);
	Position13_RTK.Resize(0,0);
	Position14_RTK.Resize(0,0);
	Position15_RTK.Resize(0,0);

	Position20_RTK.Resize(0,0);
	Position21_RTK.Resize(0,0);
	Position22_RTK.Resize(0,0);
	Position23_RTK.Resize(0,0);
	Position24_RTK.Resize(0,0);
	Position25_RTK.Resize(0,0);

	Theta1_RTK.Resize(0,0);
	Theta2_RTK.Resize(0,0);


	cout <<"The size of theta1 is "<< Theta1.rows()<<" * "<<Theta1.cols()<<endl;
	cout <<"The size of Theta2 is "<< Theta2.rows()<<" * "<<Theta2.cols()<<endl;
	cout <<"The size of Position11 is "<< Position10.rows()<<" * "<<Position10.cols()<<endl;
	cout <<"The size of Position11 is "<< Position11.rows()<<" * "<<Position11.cols()<<endl;
	cout <<"The size of Position12 is "<< Position12.rows()<<" * "<<Position12.cols()<<endl;
	cout <<"The size of Position13 is "<< Position13.rows()<<" * "<<Position13.cols()<<endl;
	cout <<"The size of Position12 is "<< Position14.rows()<<" * "<<Position14.cols()<<endl;
	cout <<"The size of Position13 is "<< Position15.rows()<<" * "<<Position15.cols()<<endl;
	cout <<"The size of Position21 is "<< Position20.rows()<<" * "<<Position20.cols()<<endl;
	cout <<"The size of Position21 is "<< Position21.rows()<<" * "<<Position21.cols()<<endl;
	cout <<"The size of Position22 is "<< Position22.rows()<<" * "<<Position22.cols()<<endl;
	cout <<"The size of Position23 is "<< Position23.rows()<<" * "<<Position23.cols()<<endl;
	cout <<"The size of Position21 is "<< Position24.rows()<<" * "<<Position24.cols()<<endl;
	cout <<"The size of Position22 is "<< Position25.rows()<<" * "<<Position25.cols()<<endl;


	MatrixXd Position1(Position10.rows()+Position11.rows()+Position12.rows()+Position13.rows()
			+Position14.rows()+Position15.rows(),Position11.cols());
	MatrixXd Position2(Position20.rows()+Position21.rows()+Position22.rows()+Position23.rows()
			+Position24.rows()+Position25.rows(),Position21.cols());

	Position1 <<Position10,
			Position11,
			Position12,
			Position13,
			Position14,
			Position15;

	Position2 <<Position20,
			Position21,
			Position22,
			Position23,
			Position24,
			Position25;
	cout <<"The size of Position1 is "<< Position1.rows()<<" * "<<Position1.cols()<<endl;
	cout <<"The size of Position2 is "<< Position2.rows()<<" * "<<Position2.cols()<<endl;


	/*			%% %%%%%%%%%%%%%%%%%%%%%%%%%%
				%  Fidning_The_Collisions	%
				%%%%%%%%%%%%%%%%%%%%%%%%%% %%			*/

	const std::size_t num_results = Position2.rows();

	MatrixXf Data_Complete(0,0);
	if (Save_complete_data_set)
	{
		Data_Complete.resize(Theta2.rows()*Theta1.rows(),1+1+1+Theta1.cols()+Theta2.cols());
	}
	MatrixXf Complete_Collision(Theta2.rows()*Theta1.rows()/10,1+1+1+Theta1.cols()+Theta2.cols());
	MatrixXf Complete_Neighbour(Theta2.rows()*Theta1.rows()/10,1+1+1+Theta1.cols()+Theta2.cols());
	VectorXd Collision(Theta2.rows());
	VectorXd Joint_Robot1_colided(Theta2.rows());
	VectorXd Joint_Robot2_colided(Theta2.rows());
	MatrixXd query_pt_eigen(6,3);
	MatrixXd result(6,Position2.rows());

	MatrixXd Debug_position1_colided(Theta2.rows()*Theta1.rows()/10,6*3);
	MatrixXd Debug_position2_colided(Theta2.rows()*Theta1.rows()/10,6*3);

	MatrixXd Debug_position1_neighbour(Theta2.rows()*Theta1.rows()/10,6*3);
	MatrixXd Debug_position2_neighbour(Theta2.rows()*Theta1.rows()/10,6*3);

	int dummy_dimention=Position1.cols();

	int joint_Robot1;
	int joint_Robot2;
	//	div_t divresult;
	int N_colisions=0;
	int N_neighbour_colisions=0;

	double begin=ros::Time::now().toNSec();

	int nthreads, tid, procs, maxt, inpar, dynamic, nested;


	/*		tid = omp_get_thread_num();
		// Get environment information
		procs = omp_get_num_procs();
		nthreads = omp_get_num_threads();
		maxt = omp_get_max_threads();
		inpar = omp_in_parallel();
		dynamic = omp_get_dynamic();
		nested = omp_get_nested();

		// Print environment information
		printf("Number of processors = %d\n", procs);
		printf("Number of threads = %d\n", nthreads);
		printf("Max threads = %d\n", maxt);
		printf("In parallel? = %d\n", inpar);
		printf("Dynamic threads enabled? = %d\n", dynamic);
		printf("Nested parallelism supported? = %d\n", nested);*/

	for (int i=0; i<Theta1.rows();i++)
	{
		if (ros::ok())
		{
			cout<<"i "<<i<<" Out of "<<Theta1.rows()<<" N_neighbour_colisions "<<N_neighbour_colisions
					<<" N_colisions "<<N_colisions<<endl;
			joint_Robot1=i;
			Collision.setOnes();
			Joint_Robot1_colided.setZero();
			Joint_Robot2_colided.setZero();
			std::cout << "Distance "<<num_results<<endl;
			for (int j=0; j<Position11.cols();j++)
			{
				query_pt_eigen(0,j)=Position10(joint_Robot1,j);
				query_pt_eigen(1,j)=Position11(joint_Robot1,j);
				query_pt_eigen(2,j)=Position12(joint_Robot1,j);
				query_pt_eigen(3,j)=Position13(joint_Robot1,j);
				query_pt_eigen(4,j)=Position14(joint_Robot1,j);
				query_pt_eigen(5,j)=Position15(joint_Robot1,j);

			}
			result=Pairwisedistance(query_pt_eigen,Position2);

#pragma omp parallel num_threads(8)
			{
#pragma omp for
				for (int ii=0;ii<result.cols();ii++)
				{
					//		divresult = div ((int)ii,(int)Theta2.rows());
					for (int j=0;j<result.rows();j++)
					{
						if (result(j,ii)<0.2)
						{
							Collision(div ((int)ii,(int)Theta2.rows()).rem)=-1;
							Joint_Robot1_colided(div ((int)ii,(int)Theta2.rows()).rem)=j+1;
							Joint_Robot2_colided(div ((int)ii,(int)Theta2.rows()).rem)=ii/Theta2.rows()+1;
						}
						else if ((result(j,ii)<0.33)&&(result(j,ii)>0.31)&&(Collision(div ((int)ii,(int)Theta2.rows()).rem)==1))
						{
							Collision(div ((int)ii,(int)Theta2.rows()).rem)=0;
							Joint_Robot1_colided(div ((int)ii,(int)Theta2.rows()).rem)=j+1;
							Joint_Robot2_colided(div ((int)ii,(int)Theta2.rows()).rem)=ii/Theta2.rows()+1;
						}
					}
				}
			}

			for (int j=0; j<Theta2.rows();j++)
			{
				joint_Robot2=j;
				if (Save_complete_data_set)
				{
					Data_Complete(i*(Theta2.rows())+j,1)=Joint_Robot1_colided(j);
					Data_Complete(i*(Theta2.rows())+j,2)=Joint_Robot2_colided(j);
					Data_Complete(i*(Theta2.rows())+j,0)=Collision(j);
					for (int ii=0;ii<Theta2.cols() ;ii++)
					{
						Data_Complete(i*(Theta2.rows())+j,3+ii)=Theta1(joint_Robot1,ii);
						Data_Complete(i*(Theta2.rows())+j,3+Theta1.cols()+ii)=Theta2(joint_Robot2,ii);
					}
				}
				if (Collision(j)==-1)
				{
					N_colisions=N_colisions+1;
					if (Complete_Collision.rows()<=N_colisions)
					{
						Complete_Collision.conservativeResize(Complete_Collision.rows()+100*Theta2.rows(), Complete_Collision.cols());
						Debug_position1_colided.conservativeResize(Debug_position1_colided.rows()+100*Theta2.rows(), Debug_position1_colided.cols());
						Debug_position2_colided.conservativeResize(Debug_position2_colided.rows()+100*Theta2.rows(), Debug_position2_colided.cols());
					}
					Complete_Collision(N_colisions-1,1)=Joint_Robot1_colided(j);
					Complete_Collision(N_colisions-1,2)=Joint_Robot2_colided(j);
					Complete_Collision(N_colisions-1,0)=Collision(j);
					for (int ii=0;ii<Theta1.cols() ;ii++)
					{
						Complete_Collision(N_colisions-1,3+ii)=Theta1(joint_Robot1,ii);
						Complete_Collision(N_colisions-1,3+Theta1.cols()+ii)=Theta2(joint_Robot2,ii);
					}
					Debug_position1_colided.row(N_colisions-1)<<Position10.row(joint_Robot1),Position11.row(joint_Robot1),Position12.row(joint_Robot1),
							Position13.row(joint_Robot1),Position14.row(joint_Robot1),Position15.row(joint_Robot1);
					Debug_position2_colided.row(N_colisions-1)<<Position20.row(joint_Robot2),Position21.row(joint_Robot2),Position22.row(joint_Robot2),
							Position23.row(joint_Robot2),Position24.row(joint_Robot2),Position25.row(joint_Robot2);

				}
				else if (Collision(j)==0)
				{
					N_neighbour_colisions=N_neighbour_colisions+1;
					if (Complete_Neighbour.rows()<=N_neighbour_colisions)
					{
						Complete_Neighbour.conservativeResize(Complete_Neighbour.rows()+100*Theta2.rows(), Complete_Neighbour.cols());
						Debug_position1_neighbour.conservativeResize(Debug_position1_neighbour.rows()+100*Theta2.rows(), Debug_position1_neighbour.cols());
						Debug_position2_neighbour.conservativeResize(Debug_position2_neighbour.rows()+100*Theta2.rows(), Debug_position2_neighbour.cols());
					}
					Complete_Neighbour(N_neighbour_colisions-1,1)=Joint_Robot1_colided(j);
					Complete_Neighbour(N_neighbour_colisions-1,2)=Joint_Robot2_colided(j);
					Complete_Neighbour(N_neighbour_colisions-1,0)=Collision(j);
					for (int ii=0;ii<Theta1.cols() ;ii++)
					{
						Complete_Neighbour(N_neighbour_colisions-1,3+ii)=Theta1(joint_Robot1,ii);
						Complete_Neighbour(N_neighbour_colisions-1,3+Theta1.cols()+ii)=Theta2(joint_Robot2,ii);
					}
					Debug_position1_neighbour.row(N_neighbour_colisions-1)<<Position10.row(joint_Robot1),Position11.row(joint_Robot1),Position12.row(joint_Robot1),
							Position13.row(joint_Robot1),Position14.row(joint_Robot1),Position15.row(joint_Robot1);
					Debug_position2_neighbour.row(N_neighbour_colisions-1)<<Position20.row(joint_Robot2),Position21.row(joint_Robot2),Position22.row(joint_Robot2),
							Position23.row(joint_Robot2),Position24.row(joint_Robot2),Position25.row(joint_Robot2);


				}
			}
		}
	}
	double end=ros::Time::now().toNSec();
	cout<<"Time in second "<<(end-begin)*1E-9<<endl;
	Complete_Collision.conservativeResize(N_colisions, Complete_Collision.cols());
	Debug_position1_colided.conservativeResize(N_colisions, Debug_position1_colided.cols());
	Debug_position2_colided.conservativeResize(N_colisions, Debug_position2_colided.cols());


	Complete_Neighbour.conservativeResize(N_neighbour_colisions, Complete_Neighbour.cols());
	Debug_position1_neighbour.conservativeResize(N_neighbour_colisions, Debug_position1_neighbour.cols());
	Debug_position2_neighbour.conservativeResize(N_neighbour_colisions, Debug_position2_neighbour.cols());


	/*			%% %%%%%%%%%%%%%%%%%%%%%%%%%%
				%  Saving the results		%
				%%%%%%%%%%%%%%%%%%%%%%%%%% %%			*/
	if (ros::ok())
	{
		/*	Eigen::write_binary("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/dataData_Complete.dat",Data_Complete);
		Eigen::write_binary("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/dataCollision_Complete.dat",Complete_Collision);
		Eigen::write_binary("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/dataNeighbour_Complete.dat",Complete_Neighbour);*/

		if (Save_complete_data_set){
			std::ofstream file("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Data_Complete.txt");
			cout<<"Data_Complete "<<endl;file<<Data_Complete<<endl;
		}
		std::ofstream file1("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Collision_Complete.txt");
		file1<<Complete_Collision<<endl;
		std::ofstream file2("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Debug_position1_colided.txt");
		file2<<Debug_position1_colided<<endl;
		std::ofstream file3("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Debug_position2_colided.txt");
		file3<<Debug_position2_colided<<endl;
		std::ofstream file4("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Neighbour_Complete.txt");
		file4<<Complete_Neighbour<<endl;
		std::ofstream file5("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Debug_position1_neighbour.txt");
		file5<<Debug_position1_neighbour<<endl;
		std::ofstream file6("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Debug_position2_neighbour.txt");
		file6<<Debug_position2_neighbour<<endl;
	}
	return 0;
}
