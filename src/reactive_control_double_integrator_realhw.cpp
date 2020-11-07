#include "reactive_control/reactive_control_double_integrator.h"

bool valid_z = true;

void human_motion_callback(const geometry_msgs::PointStampedConstPtr human_msg){
	received_point = true;
	// Check if valid initial point
	if (count <= 0){
		init_x = human_msg->point.x;
		init_y = human_msg->point.y;
		init_z = human_msg->point.z;
		if (sqrt(pow(init_x, 2) + pow(init_y, 2)) < self_col_dis and init_z < z_dis){
			count -= 1;
			ROS_WARN_STREAM("Invalid initial point leading to self collision. Give another initial point");
		}
		else if (sqrt(pow(init_x, 2) + pow(init_z, 2)) > extention_dis){
			count -= 1;
			ROS_WARN_STREAM("Invalid initial point leading to overextention. Give another initial point");
		}
		else{
			count = 0;
			xOffset = robot_pose->pose.position.x - human_msg->point.x;
			yOffset = robot_pose->pose.position.y - human_msg->point.y;
			zOffset = robot_pose->pose.position.z - human_msg->point.z;
			init_x += xOffset;
			init_y += yOffset;
			init_z += zOffset;
			ROS_INFO_STREAM("Valid initial point: " << init_x << " " << init_y << " " << init_z);
		}
	}
	else{
		if (count == 1){
			start_time = human_msg->header.stamp.toNSec();
			dis_x = human_msg->point.x + xOffset - init_x;
			dis_y = human_msg->point.y + yOffset - init_y;
			dis_z = human_msg->point.z + zOffset - init_z;
			ROS_WARN_STREAM("The initial distances are " << dis_x << ", " << dis_y << ", " << dis_z);
		}
		
		dis.data = sqrt(pow(desired_robot_position->point.x - robot_pose->pose.position.x, 2) 
			+ pow(desired_robot_position->point.y - robot_pose->pose.position.y, 2));
		dis_pub.publish(dis);
		marker_robot->points.push_back(robot_pose->pose.position);
		init_point = true;
	}

	count += 1;

	// Desired robot position
	des_x = human_msg->point.x + xOffset - dis_x;
	des_y = human_msg->point.y + yOffset - dis_y;
	des_z = human_msg->point.z + zOffset - dis_z;

	// Check if valid desired point
	if (count > 1){
		ROS_INFO("Control point dis: %f", sqrt(pow(des_x, 2) + pow(des_y, 2)));
		if (sqrt(pow(des_x, 2) + pow(des_y, 2)) < self_col_dis and des_z < z_dis){
			ROS_WARN_STREAM("Control point leading to self collision. Waiting for valid control point");
		}
		else if (sqrt(pow(des_x, 2) + pow(des_y, 2) + pow(des_z, 2)) > extention_dis){
			ROS_WARN_STREAM("Control point leading to overextention Waiting for valid control point");
		}
		else{
			ROS_INFO_STREAM("Valid control point");
			ROS_INFO_STREAM("Control points: " << desired_robot_position->point.x << " " << desired_robot_position->point.y << " " << desired_robot_position->point.z);

			// Transitioned human coordinates - Desired robot coordinates
			desired_robot_position->point.x = des_x;
			desired_robot_position->point.y = des_y;
			desired_robot_position->point.z = des_z;
			desired_robot_position->header.stamp = human_msg->header.stamp;

			human_position->point.x = des_x;
			human_position->point.y = des_y;
			human_position->point.z = des_z;
			human_position->header.stamp = robot_pose->header.stamp;

			control_points_pub.publish(*human_position);

			// Visualize human trajectory
			marker_human->header.stamp = ros::Time::now();
		    marker_human->points.push_back(desired_robot_position->point);
		  	vis_human_pub.publish(*marker_human);
		}
	}
}


void state_callback (const trajectory_execution_msgs::PoseTwist::ConstPtr state_msg){
	// Robot position
	robot_pose->pose.position.x = state_msg->pose.position.x;
	robot_pose->pose.position.y = state_msg->pose.position.y;
	robot_pose->pose.position.z = state_msg->pose.position.z;
	robot_pose->header.stamp = ros::Time::now();

	vel_duration = robot_pose->header.stamp.toSec() - robot_velocity->header.stamp.toSec();

	// Robot velocity
	robot_velocity->twist.linear.x = state_msg->twist.linear.x;
	robot_velocity->twist.linear.y = state_msg->twist.linear.y;
	robot_velocity->twist.linear.z = state_msg->twist.linear.z;
	robot_velocity->header.stamp = ros::Time::now();
	
	if (received_point){
		// Halt the robot motion if it tends to hit the table
		if (robot_pose->pose.position.z < 0.03){
			vel_control->linear.x = 0;
			vel_control->linear.y = 0;
			vel_control->linear.z = 0;
			pub.publish(*vel_control);
			ROS_ERROR_STREAM("Invalid z point. Halting motion");
		}
		// Get to the first point
		if (valid_z and count == 1){
			vel_control->linear.x = init_x - robot_pose->pose.position.x;
			vel_control->linear.y = init_y - robot_pose->pose.position.y;
			vel_control->linear.z = init_z - robot_pose->pose.position.z;
			vel_control->angular.x = 0;
			vel_control->angular.y = 0;
			vel_control->angular.z = 0;
			vel_control->linear.x = abs(vel_control->linear.x) > VEL_X_MAX_INIT ? VEL_X_MAX_INIT*abs(vel_control->linear.x)/vel_control->linear.x : vel_control->linear.x;
			vel_control->linear.y = abs(vel_control->linear.y) > VEL_Y_MAX_INIT ? VEL_Y_MAX_INIT*abs(vel_control->linear.y)/vel_control->linear.y : vel_control->linear.y;
			vel_control->linear.z = abs(vel_control->linear.z) > VEL_Z_MAX_INIT ? VEL_Z_MAX_INIT*abs(vel_control->linear.z)/vel_control->linear.z : vel_control->linear.z;
			pub.publish(*vel_control);
		}
		// Follow the human trajectory
		else if (valid_z and count != 0){
			// Positional error
			error->twist.linear.x = desired_robot_position->point.x - robot_pose->pose.position.x;
			error->twist.linear.y = desired_robot_position->point.y - robot_pose->pose.position.y;
			error->twist.linear.z = desired_robot_position->point.z - robot_pose->pose.position.z;
			error->header.stamp = ros::Time::now();
			error_pub.publish(*error);

			// Commanded acceleration
			acc[0] = -Kx*robot_velocity->twist.linear.x + Dx*error->twist.linear.x;
			acc[1] = -Ky*robot_velocity->twist.linear.y + Dy*error->twist.linear.y;
			acc[2] = -Kz*robot_velocity->twist.linear.z + Dz*error->twist.linear.z;
			
			
			// Store previous commanded velocities			
			vel_control_prev->linear.x = vel_control->linear.x;
			vel_control_prev->linear.y = vel_control->linear.y;
			vel_control_prev->linear.z = vel_control->linear.z;

			// Commanded velocity
			vel_control->linear.x += acc[0]*vel_duration;
			vel_control->linear.y += acc[1]*vel_duration;
			vel_control->linear.z += acc[2]*vel_duration;

			vel_control_stamp->twist.linear.x = vel_control->linear.x;
			vel_control_stamp->twist.linear.y = vel_control->linear.y;
			vel_control_stamp->twist.linear.z = vel_control->linear.z;
			vel_control_stamp->header.stamp = ros::Time::now();
			vel_control->linear.x = abs(vel_control->linear.x) > VEL_X_MAX ? VEL_X_MAX*abs(vel_control->linear.x)/vel_control->linear.x : vel_control->linear.x;
			vel_control->linear.y = abs(vel_control->linear.y) > VEL_Y_MAX ? VEL_Y_MAX*abs(vel_control->linear.y)/vel_control->linear.y : vel_control->linear.y;
			vel_control->linear.z = abs(vel_control->linear.z) > VEL_Z_MAX ? VEL_Z_MAX*abs(vel_control->linear.z)/vel_control->linear.z : vel_control->linear.z;
			
			// Publish velocites if not NaN
			if (!std::isnan(vel_control->linear.x) and !std::isnan(vel_control->linear.y) and !std::isnan(vel_control->linear.y)){
				ROS_INFO_STREAM("Valid commanded velocity");
				ROS_INFO_STREAM("The control time cycle is " << vel_duration);
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
			// Visualize robot trajectory
			vis_robot_pub.publish(*marker_robot);
		}
		if (init_point)
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
	n.param("reactive_control_node/self_col_dis", self_col_dis, 0.0f);
	n.param("reactive_control_node/z_dis", z_dis, 0.0f);

	// Extention distance
	n.param("reactive_control_node/extention_dis", extention_dis, 0.0f);

	// Rotation angle
	n.param("reactive_control_node/theta", theta, 0.0f);
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
  	

  	acc.resize(3);

  	// Topic names
	ee_state_topic = "/manos_cartesian_velocity_controller/ee_state";
	ee_vel_command_topic = "/manos_cartesian_velocity_controller/command_cart_vel";  		

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
	ros::Subscriber sub2 = n.subscribe("/trajectory_points", 100, human_motion_callback);
	
	ros::waitForShutdown();
}
