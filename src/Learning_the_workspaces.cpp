#include <Learning_the_workspaces.h>


#include "common.h"


int resolution=10;




int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_manager");
	ros::NodeHandle n;
	sKinematics *KUKA;
	double Position_base[3];
	double Position_constraint[3];
	double Position_constraint_direction[3];

	/*
	 * NOTE: The constraint is Position_constraint_direction*Position_of_5th_link<Position_constraint
	 * Front of the robot is negative in x and left of the robot is positive in y!
	 * Make sure that the constraints are in compatible with the position of the robots!....
	 * Plot the sixth link positions and see if all the intersections are covered or not!
	 * 	%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%  Define the base of the robots and constraints	%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%			*/


	Position_constraint_direction[0]=0;	Position_constraint_direction[1]=0;	Position_constraint_direction[2]=-1;
	Position_constraint[0]=0.1;			Position_constraint[1]=0.1;			Position_constraint[2]=-0.17;
	Position_base[0]=0;					Position_base[1]=0;					Position_base[2]=0;


	MathLib::Vector  JointPos;JointPos.Resize(7);


	Eigen::Vector3d Position_each_link;

	Eigen::Vector3d  Position_base_eigen;



		/*
		 * NOTE: The matrices should be N*D, where N is the number of the data points and D is the dimension
		 *
		 * 	%% %%%%%%%%%%%%%%%%%%%%%%%%%%
			%  Constructing_the_robots 1%
			%%%%%%%%%%%%%%%%%%%%%%%%%% %%			*/
		KUKA= new sKinematics(7, 1.0/500);
		KUKA->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0)*0.90);
		KUKA->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0)*0.90);
		KUKA->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0)*0.90);
		KUKA->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(130.0)*0.90);
		KUKA->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0)*0.90);
		KUKA->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)*0.90); // reduced joint ang$
		KUKA->setDH(6,  0.0,  0.27, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)*0.90); // reduced joint ang$

		double T0_1[4][4];
		for(int ii=0; ii<4; ii++)
			for(int j=0; j<4; j++)
				T0_1[ii][j] = 0.0;

		T0_1[0][0] = 1;
		T0_1[1][1] = 1;
		T0_1[2][2] = 1;
		T0_1[3][3] = 1;
		T0_1[0][3]=Position_base[0];
		T0_1[1][3]=Position_base[1];
		T0_1[2][3]=Position_base[2];
		KUKA->setT0(T0_1);
		KUKA->readyForKinematics();

		JointPos.Zero();
		int initial_size=1;
		for (int j=0;j<KUKA_DOF;j++)
		{
			initial_size=initial_size*resolution;
		}

		MatrixXd KUKA_Position(initial_size,3);KUKA_Position.setZero();
		Position_base_eigen(0)=Position_base[0];
		Position_base_eigen(1)=Position_base[1];
		Position_base_eigen(2)=Position_base[2];
		int count=0;


		for (double Dq_0=KUKA->getMin(0);Dq_0<=KUKA->getMax(0);Dq_0=Dq_0+(KUKA->getMax(0)-KUKA->getMin(0))/resolution)
		{
			cout<<"Dq_0 "<<Dq_0<<" out_of "<<KUKA->getMax(0)<<" count "<<count<<endl;
			for (double Dq_1=KUKA->getMin(1);Dq_1<=KUKA->getMax(1);Dq_1=Dq_1+(KUKA->getMax(1)-KUKA->getMin(1))/resolution)
			{
				for (double Dq_2=KUKA->getMin(2);Dq_2<=KUKA->getMax(2);Dq_2=Dq_2+(KUKA->getMax(2)-KUKA->getMin(2))/resolution)
				{
					for (double Dq_3=KUKA->getMin(3);Dq_3<=KUKA->getMax(3);Dq_3=Dq_3+(KUKA->getMax(3)-KUKA->getMin(3))/resolution)
					{
						for (double Dq_4=KUKA->getMin(4);Dq_4<=KUKA->getMax(4);Dq_4=Dq_4+(KUKA->getMax(4)-KUKA->getMin(4))/resolution)
						{
							for (double Dq_5=KUKA->getMin(5);Dq_5<=KUKA->getMax(5);Dq_5=Dq_5+(KUKA->getMax(5)-KUKA->getMin(5))/resolution)
							{
									JointPos(0)=Dq_0;
									JointPos(1)=Dq_1;
									JointPos(2)=Dq_2;
									JointPos(3)=Dq_3;
									JointPos(4)=Dq_4;
									JointPos(5)=Dq_5;
									JointPos(6)=0;
									KUKA->setJoints(JointPos.Array());

									KUKA->getEndPos(Position_each_link);
									if ((Position_constraint_direction[0]*Position_each_link(0)<Position_constraint[0])
											&&(Position_constraint_direction[1]*Position_each_link(1)<Position_constraint[1])
											&&(Position_constraint_direction[2]*Position_each_link(2)<Position_constraint[2])
									)
									{
										count=count+1;
										if (KUKA_Position.rows()<=count)
										{
											KUKA_Position.conservativeResize(KUKA_Position.rows()+initial_size/10, KUKA_Position.cols());

										}
										KUKA_Position.row(count-1)=Position_each_link.transpose();
								}
							}
						}
					}

				}
			}
		}

		KUKA_Position.conservativeResize(count, KUKA_Position.cols());


		buffer_path=addTwochar(folder_path,"/KUKA_END_POSITION",0);
		std::ofstream file(buffer_path.c_str());	cout<<"KUKA_Position "<<endl;	file<<KUKA_Position<<endl; file.close();


	return 0;
}


