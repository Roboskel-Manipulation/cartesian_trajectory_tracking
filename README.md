# A ROS package for real-time cartesian trajectory replication of a robotic manipulator (UR3 Cobot)

## Overall Functionality
This repo provides the utility for real-time cartesian trajectory replication by a robotic manipulator. Currently, only the position of the end effector can be controlled.

## Input 
The input consists of `geometry_msgs/PointStamped` which correspond the the <em> trajectory points </em>.

## Robot Motion Generation
The robot accepts the <em> trajectory points </em> and first checks if they lead it to self collision or overextension. If that is not the case, then they are considered valid and are referred to as <em> control points </em>.

The generation of the robot motion is based on this [Cartesian Velocity Controller](https://github.com/ThanasisTs/manos_control)(CVC), which accepts cartesian velocity commands. The generation of the commands is based on two frameworks. 

* Single Integrator: The single integrator model generates velocities based on the spatial error between the control points and the end effector's current position. The state of the end effector consists of its pose.

* Double Integrator: The double integrator model generates velocities based on computed acceleration. The acceleration is constructed by the spatial error between the control points and the end effector's current position, and the end effector's current velocity. The state of the end effector consists of its pose and its twist.

Once the end effector's commanded velocities have been computed, they are sent to the CVC, which maps them to joint velocities using IK.
 
## Dependencies 
This repo depends on:
* End effector state msgs [here](https://github.com/ThanasisTs/trajectory_execution_pkg/tree/master/trajectory_execution_msgs).

## Run 
* In a terminal run `roslaunch reactive_control reactive_control.launch`

* Arguments
   * single_integrator: true if single integrator model is used (false for double)
