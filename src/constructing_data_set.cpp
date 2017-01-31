#include <constructing_data_set.h>


#include "common.h"

double resolution=9.0;





int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_manager");
	ros::NodeHandle n;
	sKinematics *KUKA[Num_of_robots];
	double Position_base[Num_of_robots][3];
	double Position_constraint[Num_of_robots][3];
	double Position_constraint_direction[Num_of_robots][3];




	/*
	 * NOTE: The constraint is Position_constraint_direction*Position_of_5th_link<Position_constraint
	 * Front of the robot is negative in x and left of the robot is positive in y!
	 * Make sure that the constraints are in compatible with the position of the robots!....
	 * Plot the sixth link positions and see if all the intersections are covered or not!
	 * 	%% %%%%%%%%%%%%%%%%%%%%%%%%%%
		%  Define the constraints 	%
		%%%%%%%%%%%%%%%%%%%%%%%%%% %%			*/


	Position_constraint_direction[0][0]=0;	Position_constraint_direction[0][1]=-1;	Position_constraint_direction[0][2]=-1;
	Position_constraint[0][0]=0.1;			Position_constraint[0][1]=1.3;			Position_constraint[0][2]=-0.34;
	Position_base[0][0]=0;					Position_base[0][1]=-1.3;					Position_base[0][2]=0.144383;
	if (Num_of_robots>1)
	{
		Position_constraint_direction[1][0]=0;	Position_constraint_direction[1][1]=1;	Position_constraint_direction[1][2]=-1;
		Position_constraint[1][0]=0.1;			Position_constraint[1][1]=0.0;			Position_constraint[1][2]=-0.34;
		Position_base[1][0]=0;					Position_base[1][1]=0.0;				Position_base[1][2]=0;
	}
	if (Num_of_robots>2)
	{
		Position_constraint_direction[2][0]=1;	Position_constraint_direction[2][1]=0;	Position_constraint_direction[2][2]=-1;
		Position_constraint[2][0]=0.9;			Position_constraint[2][1]=0.1;			Position_constraint[2][2]=-0.34;
		Position_base[2][0]=0.9;				Position_base[2][1]=-0.45;				Position_base[2][2]=0;

	}

	if (Num_of_robots>3)
	{
		cout<<"You need to define a new set of position constraints"<<endl;
		return 0;
	}

	MathLib::Vector  JointPos;JointPos.Resize(7);

	int initial_size=(170/(resolution*10))*(170/(resolution*10))*(170/(resolution*10))*(170/(resolution*10));
	MatrixXd Theta;
	MatrixXd Position_each_link_complete[6];
	Eigen::Vector3d Position_each_link[6];

	Eigen::Vector3d  Position_base_eigen;


	for (int i=0;i<Num_of_robots;i++)
	{
		/*
		 * NOTE: The matrices should be N*D, where N is the number of the data points and D is the dimension
		 *
		 * 	%% %%%%%%%%%%%%%%%%%%%%%%%%%%
			%  Constructing_the_robots 1%
			%%%%%%%%%%%%%%%%%%%%%%%%%% %%			*/
		if (i==0)
		{
			KUKA[i] = new sKinematics(KUKA_DOF, 0.002);

			KUKA[i]->setDH(0,  0.0,  0.36, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
			KUKA[i]->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0));
			KUKA[i]->setDH(2,  0.0,  0.42,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
			KUKA[i]->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(130.0));
			KUKA[i]->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
			KUKA[i]->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)); // reduced joint ang$
			KUKA[i]->setDH(6,  0.00,  0.126+0.04, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$

		}
		else if  (i==1)
		{
			KUKA[i] = new sKinematics(KUKA_DOF, 0.002);

			KUKA[i]->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
			KUKA[i]->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0));
			KUKA[i]->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
			KUKA[i]->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(130.0));
			KUKA[i]->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
			KUKA[i]->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)); // reduced joint ang$
			KUKA[i]->setDH(6,  0.00,  0.126+0.04, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		}

		double T0_1[4][4];
		for(int ii=0; ii<4; ii++)
			for(int j=0; j<4; j++)
				T0_1[ii][j] = 0.0;

		T0_1[0][0] = 1;
		T0_1[1][1] = 1;
		T0_1[2][2] = 1;
		T0_1[3][3] = 1;
		T0_1[0][3]=Position_base[i][0];
		T0_1[1][3]=Position_base[i][1];
		T0_1[2][3]=Position_base[i][2];
		KUKA[i]->setT0(T0_1);
		KUKA[i]->readyForKinematics();

		JointPos.Zero();

		MatrixXd KUKA_Position(initial_size,6*3+7);
		Position_base_eigen(0)=Position_base[i][0];
		Position_base_eigen(1)=Position_base[i][1];
		Position_base_eigen(2)=Position_base[i][2];
		int count=0;

		for (int ii=0;ii<6;ii++)
		{
			Position_each_link_complete[ii].resize(initial_size,3);
			Position_each_link_complete[ii].setZero();
		}
		Theta.resize(initial_size,7);Theta.setZero();

		for (double Dq_0=KUKA[i]->getMin(0);Dq_0<=KUKA[i]->getMax(0);Dq_0=Dq_0+resolution*DEG2RAD(10.0))
		{
			cout<<"Dq_0 "<<Dq_0<<" out_of "<<KUKA[i]->getMax(0)<<" count "<<count<<endl;
			for (double Dq_1=KUKA[i]->getMin(1);Dq_1<=KUKA[i]->getMax(1);Dq_1=Dq_1+resolution*DEG2RAD(10.0))
			{
				for (double Dq_2=KUKA[i]->getMin(2);Dq_2<=KUKA[i]->getMax(2);Dq_2=Dq_2+resolution*DEG2RAD(10.0))
				{
					for (double Dq_3=KUKA[i]->getMin(3);Dq_3<=KUKA[i]->getMax(3);Dq_3=Dq_3+resolution*DEG2RAD(10.0))
					{
						JointPos(0)=Dq_0;
						JointPos(1)=Dq_1;
						JointPos(2)=Dq_2;
						JointPos(3)=Dq_3;
						JointPos(4)=0.0;
						JointPos(5)=0.0;
						JointPos(6)=0.0;
						KUKA[i]->setJoints(JointPos.Array());


						for(int j=0;j<6;j++)
						{
							KUKA[i]->getEndPos(j,Position_each_link[j]);
						}
						Position_each_link[0]=(Position_each_link[0]+Position_base_eigen)/2;
						Position_each_link[2]=(Position_each_link[2]+Position_each_link[1])/2;
						Position_each_link[4]=(Position_each_link[4]+Position_each_link[3])/2;

						if ((Position_constraint_direction[i][0]*Position_each_link[5](0)<Position_constraint[i][0])
						  &&(Position_constraint_direction[i][1]*Position_each_link[5](1)<Position_constraint[i][1])
						  &&(Position_constraint_direction[i][2]*Position_each_link[5](2)<Position_constraint[i][2])
						  					   )
						{
							count=count+1;
							if (Theta.rows()<=count)
							{
								Theta.conservativeResize(Theta.rows()+initial_size/10, Theta.cols());
								for(int j=0;j<6;j++)
								{
									Position_each_link_complete[j].conservativeResize(Position_each_link_complete[j].rows()+initial_size/10, Position_each_link_complete[j].cols());
								}

								KUKA_Position.conservativeResize(KUKA_Position.rows()+initial_size/10, KUKA_Position.cols());

							}
							for(int j=0;j<7;j++)
							{
								Theta(count-1,j)=JointPos(j);
							}
							for(int j=0;j<6;j++)
							{
								Position_each_link_complete[j].row(count-1)=Position_each_link[j];
							}

							KUKA_Position.row(count-1)<<Position_each_link_complete[0].row(count-1),Position_each_link_complete[1].row(count-1),Position_each_link_complete[2].row(count-1),
									Position_each_link_complete[3].row(count-1),Position_each_link_complete[4].row(count-1),Position_each_link_complete[5].row(count-1),Theta.row(count-1);
						}


					}

				}
			}
		}

		Theta.conservativeResize(count, Theta.cols());
		for(int j=0;j<6;j++)
		{
			Position_each_link_complete[j].conservativeResize(count, Position_each_link_complete[j].cols());
		}

		KUKA_Position.conservativeResize(count, KUKA_Position.cols());


		buffer_path=addTwochar(folder_path,"/KUKA_Position",i);
		std::ofstream file(buffer_path.c_str());	cout<<"KUKA_Position "<<i<<endl;	file<<KUKA_Position<<endl; file.close();
		buffer_path=addTwochar(folder_path,"/Theta",i);
		file.open(buffer_path.c_str());	cout<<"Theta "<<i<<endl;	file<<Theta<<endl; file.close();

		for(int j=0;j<6;j++)
		{
			buffer_path=addTwochar(folder_path,"/Position",i,j);
			file.open(buffer_path.c_str());	cout<<"Position "<<i<<" "<<j<<endl;	file<<Position_each_link_complete[j]<<endl; file.close();
		}

	}

	return 0;
}


