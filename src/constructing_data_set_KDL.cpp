
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

#include <constructing_data_set_KDL.h>

#include <urdf/model.h>
#include "common.h"

double resolution=2;


using namespace KDL;
using namespace std;
using namespace urdf;




void printLink(const SegmentMap::const_iterator& link, const std::string& prefix)
{
	cout << prefix << "- Segment " << link->second.segment.getName() << " has " << link->second.children.size() << " children" << endl;
	for (unsigned int i=0; i<link->second.children.size(); i++)
		printLink(link->second.children[i], prefix + "  ");
}

void printTree(Tree& tree){
	cout << " ======================================" << endl;
	cout << " Tree has " << tree.getNrOfSegments() << " link(s) and a root link" << endl;
	cout << " ======================================" << endl;

	SegmentMap::const_iterator root = tree.getRootSegment();
	printLink(root, "");
}


void printChain(const KDL::Chain& kdl_chain)
{
	if(kdl_chain.getNrOfSegments() == 0)
		ROS_WARN("KDL chain empty !");

	ROS_INFO_STREAM("  Chain has "<<kdl_chain.getNrOfJoints()<<" joints");
	ROS_INFO_STREAM("  Chain has "<<kdl_chain.getNrOfSegments()<<" segments");

	for(unsigned int i=0; i<kdl_chain.getNrOfSegments(); ++i)
		ROS_INFO_STREAM("    "<<kdl_chain.getSegment(i).getName());
}

void make_space()
{
	cout<<""<<endl;
	cout<<""<<endl;
	cout<<""<<endl;
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "data_manager");
	ros::NodeHandle n;
	double Position_base[Num_of_robots][3];
	double Position_constraint[Num_of_robots][3];
	double Position_constraint_direction[Num_of_robots][3];
	Tree my_tree;


	if (!kdl_parser::treeFromFile("valkyrie_sim.urdf", my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	cout<<" Number of joints "<< my_tree.getNrOfJoints()<<endl;
	cout<<" Number of Segments "<< my_tree.getNrOfSegments()<<endl;
	printTree(my_tree);

	Chain chain[2];
	ChainFkSolverPos_recursive *fksolver[2];
	JntArray q[2];
	Frame F_result[2][8];
	Eigen::Vector3d Position_each_link[7];

	if (!my_tree.getChain("torso","rightMiddleFingerPitch1Link",chain[0])){
		ROS_ERROR("Failed to construct a chain from torso to rightPalm");
		return false;
	}
	if (!my_tree.getChain("torso","leftMiddleFingerPitch1Link",chain[1])){
		ROS_ERROR("Failed to construct a chain from torso to leftPalm");
		return false;
	}


	urdf::Model model;
	if (!model.initFile("valkyrie_sim.urdf")){
		ROS_ERROR("Failed to parse urdf file");
		return -1;
	}



	fksolver[0] = new ChainFkSolverPos_recursive(chain[0]);
	fksolver[1] = new ChainFkSolverPos_recursive(chain[1]);


	q[0].resize(chain[0].getNrOfJoints());
	q[1].resize(chain[1].getNrOfJoints());


	make_space();
	cout<<"Right arm chain"<<endl;
	make_space();
	printChain(chain[0]);
	make_space();
	cout<<"Left arm chain"<<endl;
	make_space();
	printChain(chain[1]);

	VectorXd Lower[2]; Lower[0].resize(chain[0].getNrOfJoints());	Lower[1].resize(chain[0].getNrOfJoints());
	VectorXd Upper[2]; Upper[0].resize(chain[0].getNrOfJoints());	Upper[1].resize(chain[0].getNrOfJoints());




	for (int j=0;j<2;j++)
	{	q[j].data.setZero();
	for (int i=0;i<chain[j].getNrOfJoints();i++)
	{
		cout<<"i "<<i<<" "<<fksolver[j]->JntToCart(q[j],F_result[j][i],i+1)<<endl;
		cout<<chain[j].getSegment(i).getName()<<" "<<F_result[j][i].p<<endl;
		cout<<chain[j].getSegment(i).getJoint().getName()<<endl;
		cout<<"lower"<<model.getJoint(chain[j].getSegment(i).getJoint().getName()).get()->limits.get()->lower<<endl;
		cout<<"upper"<<model.getJoint(chain[j].getSegment(i).getJoint().getName()).get()->limits.get()->upper<<endl;
		Lower[j](i)=model.getJoint(chain[j].getSegment(i).getJoint().getName()).get()->limits.get()->lower;
		Upper[j](i)=model.getJoint(chain[j].getSegment(i).getJoint().getName()).get()->limits.get()->upper;
	}

	for (int i=0;i<3;i++)
	{
		Position_each_link[0].data()[i]=F_result[j][0].p.data[i];
		Position_each_link[1].data()[i]=F_result[j][1].p.data[i];
		Position_each_link[2].data()[i]=(F_result[j][2].p.data[i]+F_result[j][3].p.data[i])/2;
		Position_each_link[3].data()[i]=F_result[j][3].p.data[i];
		Position_each_link[4].data()[i]=(F_result[j][4].p.data[i]+F_result[j][5].p.data[i])/2;
		Position_each_link[5].data()[i]=F_result[j][5].p.data[i];
		Position_each_link[6].data()[i]=(F_result[j][6].p.data[i]+F_result[j][7].p.data[i])/2;
	}

	for (int i=0;i<chain[0].getNrOfJoints()-1;i++)
	{
		cout<<"Position_each_link["<<i<<"]"<<Position_each_link[i]<<endl;
	}

	}





	/* * NOTE: The constraint is Position_constraint_direction*Position_of_5th_link<Position_constraint
	 * Front of the robot is negative in x and left of the robot is positive in y!
	 * Make sure that the constraints are in compatible with the position of the robots!....
	 * Plot the sixth link positions and see if all the intersections are covered or not!
	 * 	%% %%%%%%%%%%%%%%%%%%%%%%%%%%
		%  Define the constraints 	%
		%%%%%%%%%%%%%%%%%%%%%%%%%% %%*/




	MathLib::Vector  JointPos;JointPos.Resize(7);

	int initial_size=(170/(resolution*10))*(170/(resolution*10))*(170/(resolution*10))*(170/(resolution*10));
	MatrixXd Theta;
	MatrixXd Position_each_link_complete[7];


	for (int i=0;i<2;i++)
	{

		/*	 * NOTE: The matrices should be N*D, where N is the number of the data points and D is the dimension
		 *
		 * 	%% %%%%%%%%%%%%%%%%%%%%%%%%%%
			%  Constructing_the_robots 1%
			%%%%%%%%%%%%%%%%%%%%%%%%%% %%*/

		MatrixXd KUKA_Position(initial_size,7*3+7);

		int count=0;

		for (int ii=0;ii<7;ii++)
		{
			Position_each_link_complete[ii].resize(initial_size,3);
			Position_each_link_complete[ii].setZero();
		}
		Theta.resize(initial_size,7);Theta.setZero();

		for (double Dq_0= Lower[i](0);Dq_0<=Upper[i](0);Dq_0=Dq_0+resolution*DEG2RAD(10.0))
		{
			cout<<"Dq_0 "<<Dq_0<<" out_of "<<Upper[i](0)<<" count "<<count<<endl;
			for (double Dq_1= Lower[i](1);Dq_1<=Upper[i](1);Dq_1=Dq_1+resolution*DEG2RAD(10.0))
			{
				//	cout<<"Dq_1 "<<Dq_1<<" out_of "<<Upper[i](1)<<" count "<<count<<endl;
				for (double Dq_2= Lower[i](2);Dq_2<=Upper[i](2);Dq_2=Dq_2+resolution*DEG2RAD(10.0))
				{
					//	cout<<"Dq_2 "<<Dq_2<<" out_of "<<Upper[i](2)<<" count "<<count<<endl;
					for (double Dq_3= Lower[i](3);Dq_3<=Upper[i](3);Dq_3=Dq_3+resolution*DEG2RAD(10.0))
					{
						//cout<<"Dq_3 "<<Dq_3<<" out_of "<<Upper[i](3)<<" count "<<count<<endl;
						for (double Dq_4= Lower[i](4);Dq_4<=Upper[i](4);Dq_4=Dq_4+DEG2RAD(180.0))
						{
							//	cout<<"Dq_4 "<<Dq_4<<" out_of "<<Upper[i](4)<<" count "<<count<<endl;
							for (double Dq_5= Lower[i](5);Dq_5<=Upper[i](5);Dq_5=Dq_5+DEG2RAD(120.0))
							{
								//	cout<<"Dq_5 "<<Dq_5<<" out_of "<<Upper[i](5)<<" count "<<count<<endl;
								for (double Dq_6= Lower[i](6);Dq_6<=Upper[i](6);Dq_6=Dq_6+DEG2RAD(120.0))
								{
									//		cout<<"Dq_6 "<<Dq_6<<" out_of "<<Upper[i](6)<<" count "<<count<<endl;
									q[i].data[0]=Dq_0;
									q[i].data[1]=Dq_1;
									q[i].data[2]=Dq_2;
									q[i].data[3]=Dq_3;
									q[i].data[4]=Dq_4;
									q[i].data[5]=Dq_5;
									q[i].data[6]=Dq_6;


									//		cout<<"1 "<<endl;
									for (int j=0;j<chain[i].getNrOfJoints();j++)
									{
										fksolver[i]->JntToCart(q[i],F_result[i][j],j+1);
									}
									//			cout<<"11 "<<endl;
									for (int ii=0;ii<3;ii++)
									{
										Position_each_link[0].data()[ii]=F_result[i][0].p.data[ii];
										Position_each_link[1].data()[ii]=F_result[i][1].p.data[ii];
										Position_each_link[2].data()[ii]=(F_result[i][2].p.data[ii]+F_result[i][3].p.data[ii])/2;
										Position_each_link[3].data()[ii]=F_result[i][3].p.data[ii];
										Position_each_link[4].data()[ii]=(F_result[i][4].p.data[ii]+F_result[i][5].p.data[ii])/2;
										Position_each_link[5].data()[ii]=F_result[i][5].p.data[ii];
										Position_each_link[6].data()[ii]=(F_result[i][6].p.data[ii]+F_result[i][7].p.data[ii])/2;
									}
									//		cout<<"2 "<<endl;

									/*																	if ((Position_constraint_direction[i][0]*Position_each_link[5](0)<=Position_constraint[i][0])
										&&(Position_constraint_direction[i][1]*Position_each_link[5](1)<=Position_constraint[i][1])
										&&(Position_constraint_direction[i][2]*Position_each_link[5](2)<=Position_constraint[i][2])
								)
								{*/
									//cout<<"3 "<<endl;
									count=count+1;
									if (Theta.rows()<=count)
									{
										Theta.conservativeResize(Theta.rows()+initial_size/10, Theta.cols());
										for(int j=0;j<7;j++)
										{
											Position_each_link_complete[j].conservativeResize(Position_each_link_complete[j].rows()+initial_size/10, Position_each_link_complete[j].cols());
										}

										KUKA_Position.conservativeResize(KUKA_Position.rows()+initial_size/10, KUKA_Position.cols());

									}
									//	cout<<"4 "<<endl;
									for(int j=0;j<7;j++)
									{
										Theta(count-1,j)=q[i].data[j];
									}
									for(int j=0;j<7;j++)
									{
										//			cout<<"j "<<j<<endl;
										Position_each_link_complete[j].row(count-1)=Position_each_link[j];
									}
									//			cout<<"5 "<<KUKA_Position.row(count-1)<<endl;
									KUKA_Position.row(count-1)<<Position_each_link_complete[0].row(count-1),Position_each_link_complete[1].row(count-1),Position_each_link_complete[2].row(count-1),
											Position_each_link_complete[3].row(count-1),Position_each_link_complete[4].row(count-1),Position_each_link_complete[5].row(count-1),
											Position_each_link_complete[6].row(count-1),Theta.row(count-1);
									//		}

								}
							}
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


		buffer_path=addTwochar(folder_path,"/Val_Arm_Position",i);
		std::ofstream file(buffer_path.c_str());	cout<<"Val_Arm_Position "<<i<<endl;	file<<KUKA_Position<<endl; file.close();
		buffer_path=addTwochar(folder_path,"/Theta",i);
		file.open(buffer_path.c_str());	cout<<"Theta "<<i<<endl;	file<<Theta<<endl; file.close();

		for(int j=0;j<7;j++)
		{
			buffer_path=addTwochar(folder_path,"/Position",i,j);
			file.open(buffer_path.c_str());	cout<<"Position "<<i<<" "<<j<<endl;	file<<Position_each_link_complete[j]<<endl; file.close();
		}

	}


	return 0;
}


