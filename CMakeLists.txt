cmake_minimum_required(VERSION 2.8.3)
project(reactive_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

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


add_executable (reactive_control src/reactive_control.cpp)
target_link_libraries (reactive_control ${catkin_LIBRARIES})

add_executable (reactive_control_acc src/reactive_control_acc.cpp)
target_link_libraries (reactive_control_acc ${catkin_LIBRARIES})