
/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Sina Mirrazavi
 * email:   sina.mirrazavi@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <analysing_data_set.h>


#include "common.h"

const char *Collision_path="/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/datacolision.txt";

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
	 * NOTE: The matrices should be N*D, where N is the number of the data points and D is the dimension
	 *
	 * 		%% %%%%%%%%%%%%%%%%%%%%%%%%%%
			%  Loading the Data points	%
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%			*/

	MathLib::Matrix Position_RTK[Num_of_robots][6];
	MathLib::Matrix Theta_RTK[Num_of_robots];

	MatrixXd Position_each_link[Num_of_robots][6];
	MatrixXd Theta[Num_of_robots];
	MatrixXd Position[Num_of_robots];
	int dimesion_row=0;
	int dimesion_cols=0;


	for (int i=0;i<Num_of_robots;i++)
	{
		dimesion_row=0;
		dimesion_cols=0;
		for (int j=0;j<6;j++)
		{
			buffer_path=addTwochar(folder_path,"/Position",i,j);
			Position_RTK[i][j].Load(buffer_path.c_str());
			Position_each_link[i][j].resize(Position_RTK[i][j].RowSize(),Position_RTK[i][j].ColumnSize());
			Position_each_link[i][j]=M2E_m(Position_RTK[i][j]).transpose();
			Position_RTK[i][j].Resize(0,0);
			cout <<"The size of Position_each_link["<<i<<"]["<<j<<"] is "<< Position_each_link[i][j].rows()<<" * "<<Position_each_link[i][j].cols()<<endl;
			dimesion_row=dimesion_row+Position_each_link[i][j].rows();
			dimesion_cols=Position_each_link[i][j].cols();
		}
		buffer_path=addTwochar(folder_path,"/Theta",i);
		Theta_RTK[i].Load(buffer_path.c_str());
		Theta[i].resize(Theta_RTK[i].RowSize(),Theta_RTK[i].ColumnSize());
		Theta[i]=M2E_m(Theta_RTK[i]).transpose();
		cout <<"The size of Theta["<<i<<"] is "<< Theta[i].rows()<<" * "<<Theta[i].cols()<<endl;
		Position[i].resize(dimesion_row,dimesion_cols);Position[i].setZero();
		dimesion_row=0;
		dimesion_cols=0;
		for (int j=0;j<6;j++)
		{
			Position[i].block(dimesion_row,0,Position_each_link[i][j].rows(),Position_each_link[i][j].cols())=Position_each_link[i][j];
			dimesion_row=dimesion_row+Position_each_link[i][j].rows();
		}
		cout <<"The size of Position["<<i<<"] is "<< Position[i].rows()<<" * "<<Position[i].cols()<<endl;
	}

	MatrixXd Position1;
	MatrixXd Position2;
	MatrixXd Theta1;
	MatrixXd Theta2;
	std::size_t num_results;

	MatrixXf Data_Complete(0,0);

	MatrixXf Complete_Collision;
	MatrixXf Complete_Neighbour;
	VectorXd Collision;
	VectorXd Joint_Robot1_colided;
	VectorXd Joint_Robot2_colided;
	MatrixXd query_pt_eigen(6,3);
	MatrixXd result;

	MatrixXd Debug_position1_colided;
	MatrixXd Debug_position2_colided;

	MatrixXd Debug_position1_neighbour;
	MatrixXd Debug_position2_neighbour;

	int dummy_dimention;

	int joint_Robot1;
	int joint_Robot2;
	//	div_t divresult;
	int N_colisions=0;
	int N_neighbour_colisions=0;

	double begin=ros::Time::now().toNSec();

	int nthreads, tid, procs, maxt, inpar, dynamic, nested;



	std::ofstream file;

	for (int o=0;o<Num_of_robots;o++)
	{
		for (int oo=o+1;oo<Num_of_robots;oo++)
		{
			Position1.resize(Position[o].rows(),Position[o].cols());	Position1=Position[o];
			Position2.resize(Position[oo].rows(),Position[oo].cols());	Position2=Position[oo];
			Theta1.resize(Theta[o].rows(),Theta[o].cols());				Theta1=Theta[o];
			Theta2.resize(Theta[oo].rows(),Theta[oo].cols());			Theta2=Theta[oo];
			cout <<"The size of Position1 is "<< Position1.rows()<<" * "<<Position1.cols()<<endl;
			cout <<"The size of Position2 is "<< Position2.rows()<<" * "<<Position2.cols()<<endl;
			cout <<"The size of Theta1 is "<< Theta1.rows()<<" * "<<Theta1.cols()<<endl;
			cout <<"The size of Theta2 is "<< Theta2.rows()<<" * "<<Theta2.cols()<<endl;
			num_results = Position2.rows();
			if (Save_complete_data_set)
			{
				Data_Complete.resize(Theta2.rows()*Theta1.rows(),1+1+1+Theta1.cols()+Theta2.cols());
				Data_Complete.setZero();
			}
			Complete_Collision.resize(Theta2.rows()*Theta1.rows()/10,1+1+1+Theta1.cols()+Theta2.cols());Complete_Collision.setZero();
			Complete_Neighbour.resize(Theta2.rows()*Theta1.rows()/10,1+1+1+Theta1.cols()+Theta2.cols());Complete_Neighbour.setZero();
			Collision.resize(Theta2.rows());														Collision.setZero();
			Joint_Robot1_colided.resize(Theta2.rows());												Joint_Robot2_colided.setZero();
			Joint_Robot2_colided.resize(Theta2.rows());												Joint_Robot2_colided.setZero();
			query_pt_eigen.setZero();
			result.resize(6,Position2.rows());														result.setZero();
			Debug_position1_colided.resize(Theta2.rows()*Theta1.rows()/10,6*3);						Debug_position1_colided.setZero();
			Debug_position2_colided.resize(Theta2.rows()*Theta1.rows()/10,6*3);						Debug_position2_colided.setZero();
			Debug_position1_neighbour.resize(Theta2.rows()*Theta1.rows()/10,6*3);					Debug_position1_neighbour.setZero();
			Debug_position2_neighbour.resize(Theta2.rows()*Theta1.rows()/10,6*3);					Debug_position2_neighbour.setZero();

			dummy_dimention=Position1.cols();
			N_colisions=0;
			N_neighbour_colisions=0;
			begin=ros::Time::now().toNSec();


			/*	%% %%%%%%%%%%%%%%%%%%%%%%%%%%
			%  Fidning_The_Collisions	%
			%%%%%%%%%%%%%%%%%%%%%%%%%% %%	*/

			cout<<" Fidning_The_Collisions"<<endl;
			for (int i=0; i<Theta1.rows();i++)
			{
				if (ros::ok())
				{
					cout<<"The first robot is "<<o<<" and the second robot is "<<oo<<". i "<<i<<" Out of "<<Theta1.rows()<<" N_neighbour_colisions "<<N_neighbour_colisions
							<<" N_colisions "<<N_colisions<<". Size of  the second robot is "<<num_results<<endl;
					joint_Robot1=i;
					Collision.setOnes();
					Joint_Robot1_colided.setZero();
					Joint_Robot2_colided.setZero();
					for (int j=0; j<Position_each_link[0][0].cols();j++)
					{
						query_pt_eigen(0,j)=Position_each_link[o][0](joint_Robot1,j);
						query_pt_eigen(1,j)=Position_each_link[o][1](joint_Robot1,j);
						query_pt_eigen(2,j)=Position_each_link[o][2](joint_Robot1,j);
						query_pt_eigen(3,j)=Position_each_link[o][3](joint_Robot1,j);
						query_pt_eigen(4,j)=Position_each_link[o][4](joint_Robot1,j);
						query_pt_eigen(5,j)=Position_each_link[o][5](joint_Robot1,j);

					}
					result=Pairwisedistance(query_pt_eigen,Position2);

#pragma omp parallel num_threads(8)
					{
#pragma omp for
						for (int ii=0;ii<result.cols();ii++)
						{
							for (int j=0;j<result.rows();j++)
							{
								if (result(j,ii)<0.2)
								{
									Collision(div ((int)ii,(int)Theta2.rows()).rem)=-1;
									Joint_Robot1_colided(div ((int)ii,(int)Theta2.rows()).rem)=j+1;
									Joint_Robot2_colided(div ((int)ii,(int)Theta2.rows()).rem)=ii/Theta2.rows()+1;
								}
								else if ((result(j,ii)<0.30)&&(result(j,ii)>0.33)&&(Collision(div ((int)ii,(int)Theta2.rows()).rem)==1))
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
							Debug_position1_colided.row(N_colisions-1)<<Position_each_link[o][0].row(joint_Robot1),Position_each_link[o][1].row(joint_Robot1),Position_each_link[o][2].row(joint_Robot1),
									Position_each_link[o][3].row(joint_Robot1),Position_each_link[o][4].row(joint_Robot1),Position_each_link[o][5].row(joint_Robot1);
							Debug_position2_colided.row(N_colisions-1)<<Position_each_link[oo][0].row(joint_Robot2),Position_each_link[oo][1].row(joint_Robot2),Position_each_link[oo][2].row(joint_Robot2),
									Position_each_link[oo][3].row(joint_Robot2),Position_each_link[oo][4].row(joint_Robot2),Position_each_link[oo][5].row(joint_Robot2);

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
							Debug_position1_neighbour.row(N_neighbour_colisions-1)<<Position_each_link[o][0].row(joint_Robot1),Position_each_link[o][1].row(joint_Robot1),Position_each_link[o][2].row(joint_Robot1),
									Position_each_link[o][3].row(joint_Robot1),Position_each_link[o][4].row(joint_Robot1),Position_each_link[o][5].row(joint_Robot1);
							Debug_position2_neighbour.row(N_neighbour_colisions-1)<<Position_each_link[oo][0].row(joint_Robot2),Position_each_link[oo][1].row(joint_Robot2),Position_each_link[oo][2].row(joint_Robot2),
									Position_each_link[oo][3].row(joint_Robot2),Position_each_link[oo][4].row(joint_Robot2),Position_each_link[oo][5].row(joint_Robot2);
						}
					}
				}// if (ros::ok())
			}//for (int i=0; i<Theta1.rows();i++)
			double end=ros::Time::now().toNSec();
			cout<<"Time in second "<<(end-begin)*1E-9<<endl;
			Complete_Collision.conservativeResize(N_colisions, Complete_Collision.cols());
			Debug_position1_colided.conservativeResize(N_colisions, Debug_position1_colided.cols());
			Debug_position2_colided.conservativeResize(N_colisions, Debug_position2_colided.cols());


			Complete_Neighbour.conservativeResize(N_neighbour_colisions, Complete_Neighbour.cols());
			Debug_position1_neighbour.conservativeResize(N_neighbour_colisions, Debug_position1_neighbour.cols());
			Debug_position2_neighbour.conservativeResize(N_neighbour_colisions, Debug_position2_neighbour.cols());

			if (Save_complete_data_set){
				buffer_path=addTwochar(folder_path,"/Data_Complete",o,oo);
				file.open(buffer_path.c_str()); cout<<"Data_Complete "<<endl;file<<Data_Complete<<endl;
				file.close();
			}


			buffer_path=addTwochar(folder_path,"/Complete_Collision",o,oo);
			file.open(buffer_path.c_str()); cout<<"Complete_Collision "<<endl;file<<Complete_Collision<<endl;
			file.close();

			buffer_path=addTwochar(folder_path,"/position1_colided",o,oo);
			file.open(buffer_path.c_str()); cout<<"position1_colided "<<endl;file<<Debug_position1_colided<<endl;
			file.close();

			buffer_path=addTwochar(folder_path,"/position2_colided",o,oo);
			file.open(buffer_path.c_str()); cout<<"position2_colided "<<endl;file<<Debug_position2_colided<<endl;
			file.close();

			buffer_path=addTwochar(folder_path,"/Complete_Neighbour",o,oo);
			file.open(buffer_path.c_str()); cout<<"Complete_Neighbour "<<endl;file<<Complete_Neighbour<<endl;
			file.close();

			buffer_path=addTwochar(folder_path,"/position1_neighbour",o,oo);
			file.open(buffer_path.c_str()); cout<<"position1_neighbour "<<endl;file<<Debug_position1_neighbour<<endl;
			file.close();

			buffer_path=addTwochar(folder_path,"/position2_neighbour",o,oo);
			file.open(buffer_path.c_str()); cout<<"position2_neighbour "<<endl;file<<Debug_position2_neighbour<<endl;
			file.close();

		}

	}
	return 0;
}
