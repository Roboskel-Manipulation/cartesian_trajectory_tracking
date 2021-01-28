#include <cartesian_trajectory_tracking/utils.h>

// Euclidean distance calculator
double euclidean_distance(const geometry_msgs::PointConstPtr p1, const geometry_msgs::PointConstPtr p2){
	double dis_x = p1->x - p2->x;
	double dis_y = p1->y - p2->y;
	double dis_z = p1->z - p2->z;
	return sqrt(pow(dis_x, 2) + pow(dis_y, 2) + pow(dis_z, 2));
}

// trajectory points callback
void control_points_callback(const geometry_msgs::PointStampedConstPtr control_point){

	desired_robot_position->point.x = control_point->point.x;
	desired_robot_position->point.y = control_point->point.y;
	desired_robot_position->point.z = control_point->point.z;
	motion_started = true;
	
	// Visualize in RViz
	if (motion_started){
		marker_robot->points.push_back(robot_pose->pose.position);
		marker_human->points.push_back(desired_robot_position->point);
		vis_human_pub.publish(*marker_human);
		vis_robot_pub.publish(*marker_robot);
	}
}
