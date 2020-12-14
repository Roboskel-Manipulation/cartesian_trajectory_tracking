#include "reactive_control/reactive_control_double_integrator.h"

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
		std::cout << overextension_limit << std::endl;
		ROS_WARN_STREAM("Control point leading to overextention Waiting for valid control point");
		limit_flag = true;
	}
	else{
		if (limit_flag){
			if (euclidean_distance(candidate_point, last_valid_point) > consecutive_points_distance){
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

void trajectory_points_callback(const geometry_msgs::PointStampedConstPtr human_msg){
	if (init_point_flag){
		xOffset = robot_pose->pose.position.x - human_msg->point.x;
		yOffset = robot_pose->pose.position.y - human_msg->point.y;
		zOffset = robot_pose->pose.position.z - human_msg->point.z;
		
		init_point->x = human_msg->point.x;
		init_point->y = human_msg->point.y;
		init_point->z = human_msg->point.z;

		init_point_flag = false;
	}
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

		check_trajectory_point(candidate_point);
	}
}


void state_callback (const trajectory_execution_msgs::PoseTwist::ConstPtr state_msg){
	if (vel_flag){
		// Computed robot velocity from robot position (bug in simulation) 
		vel_duration = ros::Time::now().toSec() - robot_pose->header.stamp.toSec();
		robot_velocity->twist.linear.x = (state_msg->pose.position.x - robot_pose->pose.position.x)/vel_duration;
		robot_velocity->twist.linear.y = (state_msg->pose.position.y - robot_pose->pose.position.y)/vel_duration;
		robot_velocity->twist.linear.z = (state_msg->pose.position.z - robot_pose->pose.position.z)/vel_duration;
		robot_velocity->header.stamp = ros::Time::now();
		real_vel_sim_pub.publish(*robot_velocity);
	}

	if (not vel_flag)
		vel_flag = true;

	// Robot position
	robot_pose->pose.position.x = state_msg->pose.position.x;
	robot_pose->pose.position.y = state_msg->pose.position.y;
	robot_pose->pose.position.z = state_msg->pose.position.z;
	robot_pose->header.stamp = ros::Time::now();

	if (received_point){
		// std::cout << "ok" << std::endl;
		
		// Velocity component
		error->twist.linear.x = desired_robot_position->point.x - robot_pose->pose.position.x;
		error->twist.linear.y = desired_robot_position->point.y - robot_pose->pose.position.y;
		error->twist.linear.z = desired_robot_position->point.z - robot_pose->pose.position.z;
		error->header.stamp = ros::Time::now();
		error_pub.publish(*error);

		acc[0] = -Kx*robot_velocity->twist.linear.x + Dx*error->twist.linear.x;
		acc[1] = -Ky*robot_velocity->twist.linear.y + Dy*error->twist.linear.y;
		acc[2] = -Kz*robot_velocity->twist.linear.z + Dz*error->twist.linear.z;
		
		vel_control_prev->linear.x = vel_control->linear.x;
		vel_control_prev->linear.y = vel_control->linear.y;
		vel_control_prev->linear.z = vel_control->linear.z;

		vel_control->linear.x += acc[0]*vel_duration;
		vel_control->linear.y += acc[1]*vel_duration;
		vel_control->linear.z += acc[2]*vel_duration;
		vel_control_stamp->twist.linear.x = vel_control->linear.x;
		vel_control_stamp->twist.linear.y = vel_control->linear.y;
		vel_control_stamp->twist.linear.z = vel_control->linear.z;
		vel_control_stamp->header.stamp = ros::Time::now();
		
		if (sqrt(pow(robot_pose->pose.position.x, 2) + pow(robot_pose->pose.position.y, 2)) < self_col_dis and robot_pose->pose.position.z < z_dis){
			ROS_WARN_STREAM("Motion leading to self collision.\nCommanding zero velocities.");
			vel_control->linear.x = 0;
			vel_control->linear.y = 0;
			vel_control->linear.z = 0;
			// pub.publish(*vel_control);
		}
		else if (!std::isnan(vel_control->linear.x) and !std::isnan(vel_control->linear.y) and !std::isnan(vel_control->linear.y)){
			ROS_INFO_STREAM("Valid commanded velocity");
			vel_control_stamp_pub.publish(*vel_control_stamp);				
			pub.publish(*vel_control);
		}
		else{
			ROS_INFO_STREAM("The commanded acceleration is " << acc[0] << " " << acc[1] << " " << acc[2]);
			ROS_INFO_STREAM("The control time cycle is " << vel_duration);
			ROS_INFO_STREAM("Gonna publish previous velocities");
			vel_control->linear.x = vel_control_prev->linear.x;
			vel_control->linear.y = vel_control_prev->linear.y;
			vel_control->linear.z = vel_control_prev->linear.z;
		}
		robot_state_pub.publish(*state_msg);
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "reactive_control_node");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(3);
	spinner.start();

	// Control gains
	n.param("reactive_control_node/Dx", Dx, 0.0f);
	n.param("reactive_control_node/Dy", Dy, 0.0f);
	n.param("reactive_control_node/Dz", Dz, 0.0f);
	n.param("reactive_control_node/Kx", Kx, 0.0f);
	n.param("reactive_control_node/Ky", Ky, 0.0f);
	n.param("reactive_control_node/Kz", Kz, 0.0f);

	// Self collision distances
	n.param("reactive_control_node/self_col_dis", self_collision_limit, 0.0f);
	n.param("reactive_control_node/z_dis", z_limit, 0.0f);

	// Extention distance
	n.param("reactive_control_node/extention_dis", overextension_limit, 0.0f);

	// Distance between consecutive valid points
	n.param("reactive_control_node/consecutive_points_distance", consecutive_points_distance, 0.0f);

	// Rotation angle
	n.param("reactive_control_node/theta", theta, 0.0f);

	std::cout << overextension_limit << " " << self_collision_limit << " " << consecutive_points_distance << std::endl;
	theta = theta * M_PI / 180;
	// Human marker - Rviz
	marker_human->header.frame_id = "base_link";
	marker_human->type = visualization_msgs::Marker::LINE_STRIP;
	marker_human->action = visualization_msgs::Marker::ADD;
	marker_human->scale.x = 0.002;
	marker_human->scale.y = 0.002;
	marker_human->scale.z = 0.002;
	marker_human->color.r = 0.0f;
	marker_human->color.g = 1.0f;
	marker_human->color.b = 0.0f;
	marker_human->color.a = 1.0;
  	marker_human->lifetime = ros::Duration(100);
	
	// Robot marker - Rviz
	marker_robot->header.frame_id = "base_link";
	marker_robot->header.stamp = ros::Time::now();
	marker_robot->type = visualization_msgs::Marker::LINE_STRIP;
	marker_robot->action = visualization_msgs::Marker::ADD;
	marker_robot->scale.x = 0.0035;
	marker_robot->scale.y = 0.0035;
	marker_robot->scale.z = 0.0035;
	marker_robot->color.r = 0.0f;
	marker_robot->color.g = 0.0f;
	marker_robot->color.b = 1.0f;
	marker_robot->color.a = 1.0;
  	marker_robot->lifetime = ros::Duration(100);
  	
  	zero_vel.linear.x = 0;
  	zero_vel.linear.y = 0;
  	zero_vel.linear.z = 0;

  	acc.resize(3);

  	// Topic names
	ee_state_topic = "/manos_cartesian_velocity_controller_sim/ee_state";
	ee_vel_command_topic = "/manos_cartesian_velocity_controller_sim/command_cart_vel";  		

	// Publishers
	pub = n.advertise<geometry_msgs::Twist>(ee_vel_command_topic, 100);
	robot_state_pub = n.advertise<trajectory_execution_msgs::PoseTwist>("/ee_state_topic", 100);
	dis_pub = n.advertise<std_msgs::Float64>("/response_topic", 100);
	control_points_pub = n.advertise<geometry_msgs::PointStamped>("trajectory_points_stamp", 100);	
	error_pub = n.advertise<geometry_msgs::TwistStamped>("error_topic", 100);
	vel_control_stamp_pub = n.advertise<geometry_msgs::TwistStamped>("vel_control_stamp_topic", 100);
	vis_human_pub = n.advertise<visualization_msgs::Marker>("/vis_human_topic", 100);
	vis_robot_pub = n.advertise<visualization_msgs::Marker>("/vis_robot_topic", 100);

	// Subscribers
	ros::Subscriber sub = n.subscribe(ee_state_topic, 100, state_callback);
	ros::Subscriber sub2 = n.subscribe("/trajectory_points", 100, trajectory_points_callback);
	
	ros::waitForShutdown();
}
