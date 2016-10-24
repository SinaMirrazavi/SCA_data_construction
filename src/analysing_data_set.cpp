#include <analysing_data_set.h>
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



MatrixXd Pairwisedistance(MatrixXd A, MatrixXd B)
{

	MatrixXd result(A.rows(),B.rows());
	VectorXd A_vector(A.cols());
	VectorXd B_vector(B.cols());


	for (int i=0;i<A.rows();i++)
	{
		A_vector=A.block(i,0,1,A.cols()).transpose();
		for (int j=0;j<B.rows();j++)
		{
			B_vector=B.block(j,0,1,B.cols()).transpose();
			result(i,j)=(A_vector-B_vector).norm();
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

	const std::size_t num_results = Position2.rows();


	MatrixXd Data_Complete(Theta2.rows()*Theta1.rows(),1+1+1+Theta1.cols()+Theta2.cols());
	MatrixXd Complete_Collision(0,1+1+1+Theta1.cols()+Theta2.cols());
	MatrixXd Complete_Neighbour(0,1+1+1+Theta1.cols()+Theta2.cols());
	MatrixXd Data_With_out_collision(0,6);
	MatrixXd Data_With_collision(0,6);
	VectorXd Dummy(6);
	VectorXd Collision(Theta2.rows());
	VectorXd Joint_Robot1_colided(Theta2.rows());
	VectorXd Joint_Robot2_colided(Theta2.rows());
	MatrixXd query_pt_eigen(3,3);
	MatrixXd result(3,Position2.rows());

	int dummy_dimention=Position1.cols();

	int joint_Robot1;
	int joint_Robot2;
	div_t divresult;
	int N_colisions=0;
	int N_neighbour_colisions=0;

	double begin=ros::Time::now().toNSec();
	for (int i=0; i<Theta1.rows();i++)
	{
		if (ros::ok())
		{
			cout<<"i "<<i<<" Out of "<<Theta1.rows() <<endl;
			joint_Robot1=i;
			std::vector<double> query_pt(dummy_dimention);
			Collision.setOnes();
			Joint_Robot1_colided.setZero();
			Joint_Robot2_colided.setZero();
			std::cout << "Distance "<<num_results<<endl;
			for (int j=0; j<Position11.cols();j++)
			{
				query_pt[j]=Position11(joint_Robot1,j);
				query_pt_eigen(0,j)=Position11(joint_Robot1,j);
				query_pt_eigen(1,j)=Position12(joint_Robot1,j);
				query_pt_eigen(2,j)=Position13(joint_Robot1,j);

			}


			result=Pairwisedistance(query_pt_eigen,Position2);
			for (int ii=0;ii<result.cols();ii++)
			{
				divresult = div ((int)ii,(int)Theta2.rows());
				for (int j=0;j<result.rows();j++)
				{
					if (result(j,ii)<0.5)
					{
						Collision(divresult.rem)=-1;
						Joint_Robot1_colided(divresult.rem)=j+1;
						Joint_Robot2_colided(divresult.rem)=ii/Theta2.rows()+1;
					}
					else if ((result(j,ii)<0.75)&&(Collision(divresult.rem)==1))
					{
						Collision(divresult.rem)=0;
						Joint_Robot1_colided(divresult.rem)=j+1;
						Joint_Robot2_colided(divresult.rem)=ii/Theta2.rows()+1;
					}
				}
			}

			for (int j=0; j<Theta2.rows();j++)
			{
				joint_Robot2=j;
				Data_Complete(i*(Theta1.rows())+j,1)=Joint_Robot1_colided(j);
				Data_Complete(i*(Theta1.rows())+j,2)=Joint_Robot2_colided(j);
				Data_Complete(i*(Theta1.rows())+j,0)=Collision(j);
				for (int ii=0;ii<Theta1.cols() ;ii++)
				{
					Data_Complete(i*(Theta1.rows())+j,3+ii)=Theta1(joint_Robot1,ii);
					Data_Complete(i*(Theta1.rows())+j,3+Theta1.cols()+ii)=Theta2(joint_Robot2,ii);
				}
				if (Collision(j)==-1)
				{
					N_colisions=N_colisions+1;
					if (Complete_Collision.rows()<=N_colisions)
					{
						Complete_Collision.conservativeResize(Complete_Collision.rows()+Theta2.rows(), Complete_Collision.cols());

					}
					Complete_Collision(N_colisions-1,1)=Joint_Robot1_colided(j);
					Complete_Collision(N_colisions-1,2)=Joint_Robot2_colided(j);
					Complete_Collision(N_colisions-1,0)=Collision(j);
					for (int ii=0;ii<Theta1.cols() ;ii++)
					{
						Complete_Collision(N_colisions-1,3+ii)=Theta1(joint_Robot1,ii);
						Complete_Collision(N_colisions-1,3+Theta1.cols()+ii)=Theta2(joint_Robot2,ii);
					}

				}
				else if (Collision(j)==0)
				{
					N_neighbour_colisions=N_neighbour_colisions+1;
					if (Complete_Neighbour.rows()<=N_neighbour_colisions)
					{
						Complete_Neighbour.conservativeResize(Complete_Neighbour.rows()+Theta2.rows(), Complete_Neighbour.cols());

					}
					Complete_Neighbour(N_neighbour_colisions-1,1)=Joint_Robot1_colided(j);
					Complete_Neighbour(N_neighbour_colisions-1,2)=Joint_Robot2_colided(j);
					Complete_Neighbour(N_neighbour_colisions-1,0)=Collision(j);
					for (int ii=0;ii<Theta1.cols() ;ii++)
					{
						Complete_Neighbour(N_neighbour_colisions-1,3+ii)=Theta1(joint_Robot1,ii);
						Complete_Neighbour(N_neighbour_colisions-1,3+Theta1.cols()+ii)=Theta2(joint_Robot2,ii);
					}

				}
			}
		}
	}
	double end=ros::Time::now().toNSec();
	cout<<"Time1 "<<(end-begin)*1E-6<<endl;
	Complete_Collision.conservativeResize(N_colisions, Complete_Collision.cols());
	Complete_Neighbour.conservativeResize(N_neighbour_colisions, Complete_Neighbour.cols());
	if (ros::ok())
	{
		Eigen::write_binary("/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Data_Complete.dat",Data_Complete);
		Eigen::write_binary("/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Collision_Complete.dat",Complete_Collision);
		Eigen::write_binary("/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Neighbour_Complete.dat",Complete_Neighbour);
/*
		std::ofstream file("/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Data_Complete.txt");
		cout<<"Data_Complete "<<endl;file<<Data_Complete<<endl;

		std::ofstream file1("/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Collision_Complete.txt");
		file1<<Complete_Collision<<endl;

		std::ofstream file2("/home/sina/Dropbox/Sinas_stuff/Gaga & Borat Shared folder/IJRR/IEEE/Testing_In_Simulation/Neighbour_Complete.txt");
		file2<<Complete_Neighbour<<endl;*/
	}
	return 0;
}
