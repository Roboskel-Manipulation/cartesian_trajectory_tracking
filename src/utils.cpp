#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

extern float self_collision_limit, z_limit, overextension_limit, consecutive_points_distance;
extern bool limit_flag, received_point;
extern geometry_msgs::PointPtr last_valid_point;
extern geometry_msgs::PointStampedPtr desired_robot_position;


double euclidean_distance(const geometry_msgs::PointConstPtr candidate_point, const geometry_msgs::PointConstPtr last_valid_point){
	double dis_x = candidate_point->x - last_valid_point->x;
	double dis_y = candidate_point->y - last_valid_point->y;
	double dis_z = candidate_point->z - last_valid_point->z;
	return sqrt(pow(dis_x, 2) + pow(dis_y, 2) + pow(dis_z, 2));
}

void check_trajectory_point(const geometry_msgs::PointConstPtr candidate_point){
	if (sqrt(pow(candidate_point->x, 2) + pow(candidate_point->y, 2)) < self_collision_limit 
		and candidate_point->z < z_limit){
		ROS_WARN_STREAM("Control point leading to self collision. Waiting for valid control point");
		limit_flag = true;
	}
	else if (sqrt(pow(candidate_point->x, 2) + pow(candidate_point->y, 2) + 
		pow(candidate_point->z, 2)) > overextension_limit){
		ROS_WARN_STREAM("Control point leading to overextention Waiting for valid control point");
		limit_flag = true;
	}
	else{
		if (limit_flag){
			if (euclidean_distance(candidate_point, last_valid_point) < consecutive_points_distance){
				desired_robot_position->point.x = candidate_point->x;
				desired_robot_position->point.y = candidate_point->y;
				desired_robot_position->point.z = candidate_point->z;
				last_valid_point->x = candidate_point->x;
				last_valid_point->y = candidate_point->y;
				last_valid_point->z = candidate_point->z;
				limit_flag = false;
			}
		}
		else{
			desired_robot_position->point.x = candidate_point->x;
			desired_robot_position->point.y = candidate_point->y;
			desired_robot_position->point.z = candidate_point->z;			
			last_valid_point->x = candidate_point->x;
			last_valid_point->y = candidate_point->y;
			last_valid_point->z = candidate_point->z;
			received_point = true;
			limit_flag = false;
		}
	}
}
