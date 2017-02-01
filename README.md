# SCA_data_construction
[![Build Status](https://travis-ci.org/sinamr66/SCA_data_construction.svg?branch=master)](https://travis-ci.org/sinamr66/SCA_data_construction)

This repository includes the packages and instructions to generate a data set for Self-Collision Avoidance (SCA) between two or more arms. You can find the paper here:

and the corresponding video here: 


**This package does not learn the SCA boundary. It just generates the SCA data-set.**

# Dependencies 
  Mathlib form Robot-toolkit (https://github.com/epfl-lasa/robot-toolkit) 
  
  KUKA Rviz visualization  (https://github.com/epfl-lasa/kuka-rviz-simulation) (Actually, you don't need it if you only want to generate the SCA data-set. This package is used  to vitualize the motion of the robots in plot_on_robot.cpp)
  
  KUKA FRI bridge (https://github.com/nbfigueroa/kuka_interface_packages) (Actually, you don't need it if you only want to generate the SCA data-set. This package is used  to vitualize the motion of the robots in plot_on_robot.cpp)
  
  Mlpack  (https://github.com/mlpack/mlpack) (Actually, you don't need it if you only want to generate the SCA data-set. This package is used to construct a probabilistic model for the reachable workspace of each robot. I prefer Matlab for doing this. The matlab code is also included in this package.)
  
  
 # Features:
- Generating the data for self-collision boundary for two or more arms.

- Vitalizing the robot configurations on the Rviz simulator.

- Generating the data set of the positions of one end-effector. 

# Before make!

1. Open [common.h](https://github.com/sinamr66/SCA_data_construction/blob/master/include/common.h).  and change folder_path to your home folder. 
2. To change the resolution of sampling and the position of the bases of the robots, open [constructing_data_set.cpp](https://github.com/sinamr66/SCA_data_construction/blob/master/src/constructing_data_set.cpp). 
  
# How to run
## 1.Generating SCA data-set
  
Make the data-set of each robot :
  
```
rosrun  constructing_data_set constructing_data_set
``` 
  
Then analysis the data set and find the collided configuration and boundaries of the collided configurations :
  
```
rosrun  constructing_data_set analysing_data_set

```
### 1.1 Visualizing the generated data set
 
 
  Launch Rviz simulator with the correct parameters of each robot, for more information see [KUKA Rviz visualization](https://github.com/epfl-lasa/kuka-rviz-simulation).
  
  ```
roslaunch kuka_lwr_bringup bimanual_simulation.launch
``` 

Run 

```
rosrun  constructing_data_set plot_on_robot
``` 

## 2. Constructing model of reachbale space of robot

Open [Learning_the_workspaces.cpp](https://github.com/sinamr66/SCA_data_construction/blob/master/src/Learning_the_workspaces.cpp) and edit it accordingly. Then make the package and run 

*If you want to learn the data set in C++, you need to uncomment some lines in this file.*

```
rosrun  constructing_data_set Learning_the_workspaces
```  

Open Matlab and run [Learning_Workspace.m](https://github.com/sinamr66/SCA_data_construction/blob/master/models/Learning_Workspace.m)
For more information contact Sina Mirrazavi.
