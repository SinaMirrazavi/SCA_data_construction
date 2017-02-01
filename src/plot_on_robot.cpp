
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

#include <plot_on_robot.h>




int resolution=100;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Sent_to_robot");
	ros::NodeHandle n;
	pub_command_pos_left = n.advertise<kuka_fri_bridge::JointStateImpedance>("/l_arm_controller/joint_imp_cmd", 3);
	pub_command_pos_right = n.advertise<kuka_fri_bridge::JointStateImpedance>("/r_arm_controller/joint_imp_cmd", 3);
	sub_left_position = n.subscribe("/l_arm_pos_controller/joint_states", 3,  chatterCallback_left_position);
	sub_right_position = n.subscribe("/r_arm_pos_controller/joint_states", 3, chatterCallback_right_position);


	/*
	 * NOTE: The matrices should be N*D, where N is the number of the data points and D is the dimention
	 *
	 * 		%% %%%%%%%%%%%%%%%%%%%%%%%%%%
				%  Loading the Data points	%
				%%%%%%%%%%%%%%%%%%%%%%%%%%%%			*/

	MathLib::Matrix Positioncollided;	Positioncollided.Load(Positioncolided_path);
	MathLib::Matrix PositionNeighbour;	PositionNeighbour.Load(PositionNeighbour_path);

	Position.Resize(14);
	JointPos.Resize(14);


	cout<<"Size of Positioncollided "<<Positioncollided.RowSize()<<" * "<<Positioncollided.ColumnSize()<<endl;
	cout<<"Size of PositionNeighbour "<<PositionNeighbour.RowSize()<<" * "<<PositionNeighbour.ColumnSize()<<endl;



	int Size=std::min(Positioncollided.RowSize(),PositionNeighbour.RowSize());
//	int Size=Positioncollided.RowSize();

	for (int i=1;i<Size;i=i+resolution)
	{
		JointPos.Zero();
		cout<<"Collided"<<endl;

		for (int j=0;j<Position.Size();j++)
		{
			Position(j)=Positioncollided(i,j+3);
		}


		Position.Print("Position");

		while ((JointPos-Position).Norm2()>0.06)
		{
			Send_Postion_To_Robot(Position);
			ros::spinOnce();
		}



		ros::Duration(1).sleep();


		for (int j=0;j<Position.Size();j++)
		{
			Position(j)=PositionNeighbour(i,j+3);
		}
		cout<<"Neighbour"<<endl;

		Position.Print("Position");

		while ((JointPos-Position).Norm2()>0.06)
		{
			Send_Postion_To_Robot(Position);
			ros::spinOnce();
		}

		ros::Duration(1).sleep();


	}


	return 0;
}


