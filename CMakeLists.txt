cmake_minimum_required(VERSION 2.8.3)
project(cartesian_trajectory_tracking)

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
	cartesian_state_msgs
)


catkin_package(
	INCLUDE_DIRS 
		${THIS_PACKAGE_INCLUDE_DIRS}
 		include
# 	LIBRARIES cartesian_trajectory_tracking
 	CATKIN_DEPENDS cartesian_state_msgs visualization_msgs geometry_msgs
# 	DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


add_executable (p_control src/p_control.cpp src/utils.cpp)
target_link_libraries (p_control ${catkin_LIBRARIES})

add_executable (pd_control_sim src/pd_control_sim.cpp src/utils.cpp)
target_link_libraries (pd_control_sim ${catkin_LIBRARIES})

add_executable (pd_control src/pd_control.cpp src/utils.cpp)
target_link_libraries (pd_control ${catkin_LIBRARIES})


