#include <constructing_data_set.h>



int resolution=3;
int Num_of_robots=2;




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


	Position_constraint_direction[0][0]=0;	Position_constraint_direction[0][1]=1;	Position_constraint_direction[0][2]=-1;
	Position_constraint[0][0]=0.1;			Position_constraint[0][1]=0;			Position_constraint[0][2]=-0.34;
	Position_base[0][0]=0;					Position_base[0][1]=0;					Position_base[0][2]=0;
	if (Num_of_robots>1)
	{
		Position_constraint_direction[1][0]=0;	Position_constraint_direction[1][1]=-1;	Position_constraint_direction[1][2]=-1;
		Position_constraint[1][0]=0.1;			Position_constraint[1][1]=0.9;			Position_constraint[1][2]=-0.34;
		Position_base[1][0]=0;					Position_base[1][1]=-0.9;				Position_base[1][2]=0;
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
		KUKA[i]= new sKinematics(7, 1.0/500);
		KUKA[i]->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0)*0.90);
		KUKA[i]->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0)*0.90);
		KUKA[i]->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0)*0.90);
		KUKA[i]->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(130.0)*0.90);
		KUKA[i]->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0)*0.90);
		KUKA[i]->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)*0.90); // reduced joint ang$
		KUKA[i]->setDH(6,  0.0,  0.00, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)*0.90); // reduced joint ang$

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


	/*if (generate_data_second_robot)
	{

	 * NOTE: The matrices should be N*D, where N is the number of the data points and D is the dimention
	 *
	 * 	%% %%%%%%%%%%%%%%%%%%%%%%%%%%
			%  Constructing_the_robots 2%
			%%%%%%%%%%%%%%%%%%%%%%%%%% %%

		sKinematics KUKA_2(7, 1.0/500);

		KUKA_2.setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0)*0.90);
		KUKA_2.setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0)*0.90);
		KUKA_2.setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0)*0.90);
		KUKA_2.setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(130.0)*0.90);
		KUKA_2.setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0)*0.90);
		KUKA_2.setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)*0.90); // reduced joint ang$
		KUKA_2.setDH(6,  0.0,  0.00, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)*0.90); // reduced joint ang$

		double T0_2[4][4];
		for(int i=0; i<4; i++)
			for(int j=0; j<4; j++)
				T0_2[i][j] = 0.0;

		T0_2[0][0] = 1;
		T0_2[1][1] = 1;
		T0_2[2][2] = 1;
		T0_2[3][3] = 1;
		T0_2[0][3]=X_2;
		T0_2[1][3]=Y_2;
		T0_2[2][3]=Z_2;
		KUKA_2.setT0(T0_2);
		KUKA_2.readyForKinematics();


		MathLib::Vector  JointPos;JointPos.Resize(7);JointPos.Zero();

		int initial_size=34*34*34*34;
		MatrixXd Theta2(initial_size,7);
		MatrixXd Position20(initial_size,3);MatrixXd Position21(initial_size,3);MatrixXd Position22(initial_size,3);
		MatrixXd Position23(initial_size,3);MatrixXd Position24(initial_size,3);MatrixXd Position25(initial_size,3);
		MatrixXd KUKA_2_Position(initial_size,6*3+7);

		Vector3 Position_base; Position_base(0)=X_2;Position_base(1)=Y_2;Position_base(2)=Z_2;
		Vector3 Position0;Vector3 Position1;Vector3 Position2;
		Vector3 Position3;Vector3 Position4;Vector3 Position5;


		int count=0;
		for (double Dq_0=KUKA_2.getMin(0);Dq_0<=KUKA_2.getMax(0);Dq_0=Dq_0+resolution*DEG2RAD(10.0))
		{
			cout<<"Dq_0 "<<Dq_0<<" out_of "<<KUKA_2.getMax(0)<<" count "<<count<<endl;
			for (double Dq_1=KUKA_2.getMin(1);Dq_1<=KUKA_2.getMax(1);Dq_1=Dq_1+resolution*DEG2RAD(10.0))
			{
				for (double Dq_2=KUKA_2.getMin(2);Dq_2<=KUKA_2.getMax(2);Dq_2=Dq_2+resolution*DEG2RAD(10.0))
				{
					for (double Dq_3=KUKA_2.getMin(3);Dq_3<=KUKA_2.getMax(3);Dq_3=Dq_3+resolution*DEG2RAD(10.0))
					{

						JointPos(0)=Dq_0;
						JointPos(1)=Dq_1;
						JointPos(2)=Dq_2;
						JointPos(3)=Dq_3;
						JointPos(4)=0.0;
						JointPos(5)=0.0;
						JointPos(6)=0.0;
						KUKA_2.setJoints(JointPos.Array());
						KUKA_2.getEndPos(0,Position0.Array());	Position0=(Position0+Position_base)/2;
						KUKA_2.getEndPos(1,Position1.Array());
						KUKA_2.getEndPos(2,Position2.Array());	Position2=(Position2+Position1)/2;
						KUKA_2.getEndPos(3,Position3.Array());
						KUKA_2.getEndPos(4,Position4.Array());	Position4=(Position4+Position3)/2;
						KUKA_2.getEndPos(5,Position5.Array());
						if ((Position5(1)>Position1(1))&&(Position5(2)>Position1(2)))
						{count=count+1;
						if (Theta2.rows()<=count)
						{
							Theta2.conservativeResize(Theta2.rows()+initial_size/10, Theta2.cols());
							Position20.conservativeResize(Position20.rows()+initial_size/10, Position20.cols());
							Position21.conservativeResize(Position21.rows()+initial_size/10, Position21.cols());
							Position22.conservativeResize(Position22.rows()+initial_size/10, Position22.cols());
							Position23.conservativeResize(Position23.rows()+initial_size/10, Position23.cols());
							Position24.conservativeResize(Position24.rows()+initial_size/10, Position24.cols());
							Position25.conservativeResize(Position25.rows()+initial_size/10, Position25.cols());
							KUKA_2_Position.conservativeResize(KUKA_2_Position.rows()+initial_size/10, KUKA_2_Position.cols());

						}
						Theta2(count-1,0)=JointPos(0);Theta2(count-1,1)=JointPos(1);Theta2(count-1,2)=JointPos(2);
						Theta2(count-1,3)=JointPos(3);Theta2(count-1,4)=JointPos(4);Theta2(count-1,5)=JointPos(5);
						Theta2(count-1,6)=JointPos(6);
						Position20.row(count-1)<<Position0(0),Position0(1),Position0(2);
						Position21.row(count-1)<<Position1(0),Position1(1),Position1(2);
						Position22.row(count-1)<<Position2(0),Position2(1),Position2(2);
						Position23.row(count-1)<<Position3(0),Position3(1),Position3(2);
						Position24.row(count-1)<<Position4(0),Position4(1),Position4(2);
						Position25.row(count-1)<<Position5(0),Position5(1),Position5(2);
						KUKA_2_Position.row(count-1)<<Position20.row(count-1),Position21.row(count-1),Position22.row(count-1),
								Position23.row(count-1),Position24.row(count-1),Position25.row(count-1),Theta2.row(count-1);
						}
					}
				}
			}
		}

		Theta2.conservativeResize(count, Theta2.cols());
		Position20.conservativeResize(count, Position20.cols());
		Position21.conservativeResize(count, Position21.cols());
		Position22.conservativeResize(count, Position22.cols());
		Position23.conservativeResize(count, Position23.cols());
		Position24.conservativeResize(count, Position24.cols());
		Position25.conservativeResize(count, Position25.cols());
		KUKA_2_Position.conservativeResize(count, KUKA_2_Position.cols());

		std::ofstream file("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/KUKA_2_Position.txt");	cout<<"KUKA_2_Position "<<endl;	file<<KUKA_2_Position<<endl;
		std::ofstream file1("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Theta2.txt");			cout<<"Theta2 "<<endl;			file1<<Theta2<<endl;
		std::ofstream file2("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position20.txt");		cout<<"Position20 "<<endl;		file2<<Position20<<endl;
		std::ofstream file3("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position21.txt");		cout<<"Position21 "<<endl;		file3<<Position21<<endl;
		std::ofstream file4("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position22.txt");		cout<<"Position22 "<<endl;		file4<<Position22<<endl;
		std::ofstream file5("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position23.txt");		cout<<"Position23 "<<endl;		file5<<Position23<<endl;
		std::ofstream file6("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position24.txt");		cout<<"Position24 "<<endl;		file6<<Position24<<endl;
		std::ofstream file7("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position25.txt");		cout<<"Position25 "<<endl;		file7<<Position25<<endl;
	}*/

	return 0;
}


