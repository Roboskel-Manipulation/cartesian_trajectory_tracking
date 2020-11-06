# A ROS package for reactive HRI with Manos

<b>Overall Functionality</b>: The goal of this project is the development of a ROS-based framework for HRI and HRC tasks between a human operator and a robotic manipulator. The interaction is based on visual input, and specifically the Openpose project is utilized for the recording of the human motion. The robotic manipulator can either mimic the human motion (visual teleoperation) or mirror it or react according to any other way depending on the current application.

<b>Input Control Points</b>: The human motion is recorded using Openpose and 3D coordinates of the human right wrist are computed. These coordinates are utilized either directly either indirectly as input control points to the robot. The 'directly' term means that no preprocessing of the 3D coordinates happens, while the 'indirectly' terms means that a preproccesing procedure exists. 

<b> Preprocess of the 3D Human Coordinates </b>: Two seperate methods are utilized in the preprocessing stage aiming to denoising the 3D coordinates:

* Piecewise Bezier: Construct a bezier of 20 points and keep only these whose distance is greater than 5mm. Publish each point
with 0.0005s delay on top of the Openpose delay (0.047s)

* Downsampling-Interpolation: Publish interpolated points with 0.0005s delay on top of the Openpose delay (0.047s). Approximatelly
at least in 8% of the points in the natural movements and in 32% of the points in the fast movements.

<b> Robot Control </b>: The main functionality of the packege is the robot control. The control of the robot is based on this [Cartesian Velocity Controller](https://github.com/ThanasisTs/manos_control). At the moment, the framework results in the robot mimicing the human motion (visual teleoperation).