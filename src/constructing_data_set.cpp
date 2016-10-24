#include <constructing_data_set.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_manager");
	ros::NodeHandle n;
	/*
	 * NOTE: The matrices should be N*D, where N is the number of the data points and D is the dimention
	 *
	 * 		%% %%%%%%%%%%%%%%%%%%%%%%%%%%
			%  Constructing_the_robots 1	%
			%%%%%%%%%%%%%%%%%%%%%%%%%% %%			*/

	sKinematics KUKA_1(7, 1.0/500);

	KUKA_1.setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD( -170.), DEG2RAD( 170.), DEG2RAD(98.0)*0.90);
	KUKA_1.setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -120.), DEG2RAD( 120.), DEG2RAD(98.0)*0.90);
	KUKA_1.setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0)*0.90);
	KUKA_1.setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(130.0)*0.90);
	KUKA_1.setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0)*0.90);
	KUKA_1.setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -120.), DEG2RAD( 120.), DEG2RAD(180.0)*0.90); // reduced joint ang$

	double T0[4][4];
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			T0[i][j] = 0.0;

	T0[0][0] = 1;
	T0[1][1] = 1;
	T0[2][2] = 1;
	T0[3][3] = 1;
	KUKA_1.setT0(T0);
	KUKA_1.readyForKinematics();


	MathLib::Vector  JointPos;JointPos.Resize(7);JointPos.Zero();

	int initial_size=34*34*34*34;
	MatrixXd Theta1(initial_size,7);
	MatrixXd Position10(initial_size,3);MatrixXd Position11(initial_size,3);MatrixXd Position12(initial_size,3);
	MatrixXd Position13(initial_size,3);MatrixXd Position14(initial_size,3);MatrixXd Position15(initial_size,3);
	MatrixXd KUKA_1_Position(initial_size,6*3+7);

	Vector3 Position0;Vector3 Position1;Vector3 Position2;
	Vector3 Position3;Vector3 Position4;Vector3 Position5;


	int count=0;
	int resolution=1;
	for (double Dq_0=KUKA_1.getMin(0);Dq_0<=KUKA_1.getMax(0);Dq_0=Dq_0+resolution*DEG2RAD(10.0))
	{
		cout<<"Dq_0 "<<Dq_0<<" out_of "<<KUKA_1.getMax(0)<<" count "<<count<<endl;
		for (double Dq_1=KUKA_1.getMin(1);Dq_1<=KUKA_1.getMax(1);Dq_1=Dq_1+resolution*DEG2RAD(10.0))
		{
			for (double Dq_2=KUKA_1.getMin(2);Dq_2<=KUKA_1.getMax(2);Dq_2=Dq_2+resolution*DEG2RAD(10.0))
			{
				for (double Dq_3=KUKA_1.getMin(3);Dq_3<=KUKA_1.getMax(2);Dq_3=Dq_3+resolution*DEG2RAD(10.0))
				{

					JointPos(0)=Dq_0;
					JointPos(1)=Dq_1;
					JointPos(2)=Dq_2;
					JointPos(3)=Dq_3;
					JointPos(4)=0.0;
					JointPos(5)=0.0;
					JointPos(6)=0.0;
					KUKA_1.setJoints(JointPos.Array());
					KUKA_1.getEndPos(0,Position0.Array());	Position0=Position0/2;
					KUKA_1.getEndPos(1,Position1.Array());
					KUKA_1.getEndPos(2,Position2.Array());	Position2=(Position2+Position1)/2;
					KUKA_1.getEndPos(3,Position3.Array());
					KUKA_1.getEndPos(4,Position4.Array());	Position4=(Position4+Position3)/2;
					KUKA_1.getEndPos(5,Position5.Array());
					count=count+1;
					if (Theta1.rows()<=count)
					{
						Theta1.conservativeResize(Theta1.rows()+initial_size/10, Theta1.cols());
						Position10.conservativeResize(Position10.rows()+initial_size/10, Position10.cols());
						Position11.conservativeResize(Position11.rows()+initial_size/10, Position11.cols());
						Position12.conservativeResize(Position12.rows()+initial_size/10, Position12.cols());
						Position13.conservativeResize(Position13.rows()+initial_size/10, Position13.cols());
						Position14.conservativeResize(Position14.rows()+initial_size/10, Position14.cols());
						Position15.conservativeResize(Position15.rows()+initial_size/10, Position15.cols());
						KUKA_1_Position.conservativeResize(KUKA_1_Position.rows()+initial_size/10, KUKA_1_Position.cols());

					}
					Theta1(count-1,0)=JointPos(0);Theta1(count-1,1)=JointPos(1);Theta1(count-1,2)=JointPos(2);
					Theta1(count-1,3)=JointPos(3);Theta1(count-1,4)=JointPos(4);Theta1(count-1,5)=JointPos(5);
					Theta1(count-1,6)=JointPos(6);
					Position10.row(count-1)<<Position0(0),Position0(1),Position0(2);
					Position11.row(count-1)<<Position1(0),Position1(1),Position1(2);
					Position12.row(count-1)<<Position2(0),Position2(1),Position2(2);
					Position13.row(count-1)<<Position3(0),Position3(1),Position3(2);
					Position14.row(count-1)<<Position4(0),Position4(1),Position4(2);
					Position15.row(count-1)<<Position5(0),Position5(1),Position5(2);
					KUKA_1_Position.row(count-1)<<Position10.row(count-1),Position11.row(count-1),Position12.row(count-1),
							Position13.row(count-1),Position14.row(count-1),Position15.row(count-1),Theta1.row(count-1);
				}
			}
		}
	}

	std::ofstream file("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/KUKA_1_Position.txt");	cout<<"KUKA_1_Position "<<endl;	file<<KUKA_1_Position<<endl;
	std::ofstream file1("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Theta1.txt");			cout<<"Theta1 "<<endl;			file1<<Theta1<<endl;
	std::ofstream file2("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position10.txt");		cout<<"Position10 "<<endl;		file2<<Position10<<endl;
	std::ofstream file3("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position11.txt");		cout<<"Position11 "<<endl;		file3<<Position11<<endl;
	std::ofstream file4("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position12.txt");		cout<<"Position12 "<<endl;		file4<<Position12<<endl;
	std::ofstream file5("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position13.txt");		cout<<"Position13 "<<endl;		file5<<Position13<<endl;
	std::ofstream file6("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position14.txt");		cout<<"Position14 "<<endl;		file6<<Position14<<endl;
	std::ofstream file7("/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Data_Analysis/constructing_data_set/data/Position15.txt");		cout<<"Position15 "<<endl;		file7<<Position15<<endl;



	/*
	 * NOTE: The matrices should be N*D, where N is the number of the data points and D is the dimention
	 *
	 * 		%% %%%%%%%%%%%%%%%%%%%%%%%%%%
			%  Constructing_the_robots 2	%
			%%%%%%%%%%%%%%%%%%%%%%%%%% %%			*/





	return 0;
}


