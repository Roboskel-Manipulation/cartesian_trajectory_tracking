# cartesian_trajectory_tracking

## Overall Functionality
This package provides the utility for cartesian trajectory tracking by a robotic manipulator (UR3 COBOT). Currently, only the position of the end effector can be controlled.

## Input 
The <em> trajectory points </em> are `geometry_msgs/PointStamped`.

## Robot Motion Generation
The generation of the commands is based on two frameworks. 

* P Controller: The P controller generates velocities based on the spatial error between the control points and the end effector's current position.

* PD Controller: The PD controller generates velocities based on computed acceleration. The acceleration is constructed by the spatial error between the control points and the end effector's current position, and the end effector's current velocity.

Once the end effector's commanded velocities have been computed, they are sent to a [Cartesian Velocity Controller](https://github.com/ThanasisTs/manos_control), which accepts cartesian velocity commands.  
 
## Dependencies 
This repo depends on:
* End effector state msgs [here](https://github.com/Roboskel_Manipulation/trajectory_execution_pkg/tree/master/trajectory_execution_msgs).

## Run 
* In a terminal run `roslaunch cartesian_trajectory_tracking cartesian_trajectory_tracking.launch`

* Arguments
   * p_control: true if P Controller is used (false for PD)
   * gazebo: true if running Gazebo
