# A ROS package for reactive HRI with Manos

# Description
<b>Overall Functionality</b>: The goal of this project is the development of a ROS-based framework for HRI and HRC tasks between a human operator and a robotic manipulator. The interaction is based on visual input, and specifically the Openpose project is utilized for the recording of the human motion. Currently, the robotic manipulator can mimic the human motion.

<b>Input Control Points</b>: The human motion is recorded using Openpose and 3D coordinates of the human right wrist are computed. These coordinates are utilized either directly or indirectly as input control points to the robot. The 'directly' term means that no preprocessing of the 3D coordinates happens, while the 'indirectly' terms means that a preproccesing procedure exists. 

<b> Preprocess of the 3D Human Coordinates </b>: Two seperate methods are utilized in the preprocessing stage aiming to denoising the 3D coordinates:

* Piecewise Bezier: Construct a bezier of 20 points and keep only these whose distance is greater than 5mm. Publish each point
with 0.0005s delay on top of the Openpose delay (0.047s)

* Downsampling-Interpolation: Publish interpolated points with 0.0005s delay on top of the Openpose delay (0.047s). Approximatelly
at least in 8% of the points in the natural movements and in 32% of the points in the fast movements.

<b> Robot Control </b>: The main functionality of the packege is the robot control. The control of the robot is based on this [Cartesian Velocity Controller](https://github.com/ThanasisTs/manos_control)(CVC). There are two control frameworks which generate velocity commands which are fed to the CVC.

* Single Integrator: The single integrator model generates velocities based on the spatial error between the human position and the end effector's current position.
The state of the end effector consists of its pose.

* Double Integrator: The double integrator model generates velocities based on computed accelration. The acceleration is constructed by the spatial error between the human position and the end effector's current position, and the end effector's current velocity. The state of the end effector consists of its pose and its twist.

# Dependencies
This repo depends on:
* Keypoint 3D matching repo [here](https://github.com/ThanasisTs/openpose_utils/tree/master/keypoint_3d_matching) for creating the control input (human position).
* Trajectory process [here](https://github.com/ThanasisTs/trajectory_process_utils) for processing the openpose points.
* ROS messages [here](https://github.com/ThanasisTs/trajectory_execution_pkg/tree/master/trajectory_execution_msgs)

# Run
* If you want to run in a real robot in real time, run `roslaunch reactive_control reactive_framework_live.launch`.
* If you want to teleoperate the robot using a rosbag, run `roslaunch reactive_control reactive_framework_live_rosbag.launch`.
* If you want to teleoperate the simulated robot in GAZEBO, replace the word `live` in the aforementrioned commands with `sim`.
