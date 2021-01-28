# A ROS package for real-time cartesian trajectory tracking by a robotic manipulator (UR3 Cobot)

## Overall Functionality
This repo provides the utility for real-time cartesian trajectory tracking by a robotic manipulator. Currently, only the position of the end effector can be controlled.

## Input 
The input consists of `geometry_msgs/PointStamped` which correspond to the <em> trajectory points </em>.

## Robot Motion Generation
The robot accepts <em> trajectory points </em> one at a time and first checks if they lead it to self collision or overextension. If that is not the case, then they are considered valid and are referred to as <em> control points </em>.

The generation of the robot motion is based on this [Cartesian Velocity Controller](https://github.com/ThanasisTs/manos_control)(CVC), which accepts cartesian velocity commands. The generation of the commands is based on two frameworks. 

* P Controller: The P controller generates velocities based on the spatial error between the control points and the end effector's current position.

* PD Controller: The PD controller generates velocities based on computed acceleration. The acceleration is constructed by the spatial error between the control points and the end effector's current position, and the end effector's current velocity.

Once the end effector's commanded velocities have been computed, they are sent to the CVC.
 
## Dependencies 
This repo depends on:
* End effector state msgs [here](https://github.com/Roboskel_Manipulation/trajectory_execution_pkg/tree/master/trajectory_execution_msgs).

## Run 
* In a terminal run `roslaunch cartesian_trajectory_tracking cartesian_trajectory_tracking.launch`

* Arguments
   * p_control: true if P Controller is used (false for PD)
