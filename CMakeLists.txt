cmake_minimum_required(VERSION 2.8.3)
project(reactive_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	visualization_msgs
	keypoint_3d_matching_msgs
	trajectory_execution_msgs
	trajectory_process_utils_srvs
	tf
)


catkin_package(
	INCLUDE_DIRS 
		${THIS_PACKAGE_INCLUDE_DIRS}
 		include
# 	LIBRARIES reactive_control
 	CATKIN_DEPENDS trajectory_execution_msgs keypoint_3d_matching_msgs visualization_msgs geometry_msgs
# 	DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


add_executable (reactive_control_single_integrator src/reactive_control_single_integrator.cpp)
target_link_libraries (reactive_control_single_integrator ${catkin_LIBRARIES})

add_executable (reactive_control_double_integrator_sim src/reactive_control_double_integrator_sim.cpp)
target_link_libraries (reactive_control_double_integrator_sim ${catkin_LIBRARIES})

add_executable (reactive_control_double_integrator_realhw src/reactive_control_double_integrator_realhw.cpp)
target_link_libraries (reactive_control_double_integrator_realhw ${catkin_LIBRARIES})

add_executable (jog_arm_teleop src/jog_arm_teleop.cpp)
target_link_libraries (jog_arm_teleop ${catkin_LIBRARIES})
