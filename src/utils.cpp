#include <reactive_control/utils.h>

// Euclidean distance calculator
double euclidean_distance(const geometry_msgs::PointConstPtr candidate_point, const geometry_msgs::PointConstPtr last_valid_point){
	double dis_x = candidate_point->x - last_valid_point->x;
	double dis_y = candidate_point->y - last_valid_point->y;
	double dis_z = candidate_point->z - last_valid_point->z;
	return sqrt(pow(dis_x, 2) + pow(dis_y, 2) + pow(dis_z, 2));
}

// Check if trajectory point is leading to overextension or self collision
void check_trajectory_point(const geometry_msgs::PointConstPtr candidate_point){
	// Self-collision checking
	if (sqrt(pow(candidate_point->x, 2) + pow(candidate_point->y, 2)) < self_collision_limit 
		and candidate_point->z < z_limit){
		ROS_WARN_STREAM("Control point leading to self collision. Waiting for valid control point");
		limit_flag = true;
	}
	// Overextension checking
	else if (sqrt(pow(candidate_point->x, 2) + pow(candidate_point->y, 2) + 
		pow(candidate_point->z, 2)) > overextension_limit){
		ROS_WARN_STREAM("Control point leading to overextention Waiting for valid control point");
		limit_flag = true;
	}
	else{
		if (limit_flag){
			// If the robot is in overextension or self-collision state
			// check if the next valid point is at most `consecutive_points_distance`
			// meters apart from the last valid point
			if (euclidean_distance(candidate_point, last_valid_point) < consecutive_points_distance){
				desired_robot_position->point.x = candidate_point->x;
				desired_robot_position->point.y = candidate_point->y;
				desired_robot_position->point.z = candidate_point->z;
				last_valid_point->x = candidate_point->x;
				last_valid_point->y = candidate_point->y;
				last_valid_point->z = candidate_point->z;
				limit_flag = false;
				control_points_pub.publish(*desired_robot_position);
			}
		}
		// Publish the trajectory point if the robot is not in self-collision / overextension state
		// and the point does not lead to such a state
		else{
			desired_robot_position->point.x = candidate_point->x;
			desired_robot_position->point.y = candidate_point->y;
			desired_robot_position->point.z = candidate_point->z;			
			last_valid_point->x = candidate_point->x;
			last_valid_point->y = candidate_point->y;
			last_valid_point->z = candidate_point->z;
			received_point = true;
			limit_flag = false;
			control_points_pub.publish(*desired_robot_position);
		}
	}
}

// trajectory points callback
void trajectory_points_callback(const geometry_msgs::PointStampedConstPtr human_msg){
	// If it is the first trajectory point, compute the translation offset
	if (init_point_flag){
		xOffset = robot_pose->pose.position.x - human_msg->point.x;
		yOffset = robot_pose->pose.position.y - human_msg->point.y;
		zOffset = robot_pose->pose.position.z - human_msg->point.z;
		
		init_point->x = human_msg->point.x;
		init_point->y = human_msg->point.y;
		init_point->z = human_msg->point.z;

		init_point_flag = false;
	}
	// Omit the distance between the first and second trajectory points,
	// because it causes abrupt beginning of the robot motion
	else if (second_point_flag){
		jump_dis->x = human_msg->point.x - init_point->x;
		jump_dis->y = human_msg->point.y - init_point->y;
		jump_dis->z = human_msg->point.z - init_point->z;

		second_point_flag = false;
	}
	else{
		candidate_point->x = human_msg->point.x + xOffset - jump_dis->x;
		candidate_point->y = human_msg->point.y + yOffset - jump_dis->y;
		candidate_point->z = human_msg->point.z + zOffset - jump_dis->z;

		// Check if the trajectory point is valid and update the desired trajectory point accordingly
		check_trajectory_point(candidate_point);
	}
}