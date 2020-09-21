#include "reactive_control/reactive_control_double_integrator.h"


void human_motion_callback(const geometry_msgs::PointStampedConstPtr human_msg){
	received_point = true;
	if (count <= 0){
		init_x = human_msg->point.x;
		init_y = human_msg->point.y;
		init_z = human_msg->point.z;
		if (sqrt(pow(init_x, 2) + pow(init_y, 2)) < self_col_dis and init_z < z_dis){
			count -= 1;
			ROS_WARN("Invalid initial point leading to self collision.\nGive another initial point");
		}
		else if (sqrt(pow(init_x, 2) + pow(init_y, 2)) > extention_dis){
			count -= 1;
			ROS_WARN("Invalid initila point leading to overextention.\nGive another initial point");
		}
		else{
			count = 0;
			xOffset = robot_pose->pose.position.x - human_msg->point.x;
			yOffset = robot_pose->pose.position.y - human_msg->point.y;
			zOffset = robot_pose->pose.position.z - human_msg->point.z;
			init_x += xOffset;
			init_y += yOffset;
			init_z += zOffset;
			ROS_INFO("Valid initial point");
			std::cout << xOffset << " " << yOffset << " " << zOffset << std::endl;
		}
		time_init = human_msg->header.stamp.toSec();
	}
	else{
		if (count == 1){
			start_time = human_msg->header.stamp.toSec();
			dis_x = human_msg->point.x + xOffset - init_x;
			dis_y = human_msg->point.y + yOffset - init_y;
			dis_z = human_msg->point.z + zOffset - init_z;
			std::cout << dis_x << " " << dis_y << " " << dis_z << std::endl;
		}
		timenow = ros::Time::now();
		robot_pose->header.stamp = timenow;
		state_pub_low_f.publish(*robot_pose);
		
		vis_robot_pub.publish(*marker_robot);
		dis.data = sqrt(pow(desired_robot_position->point.x - robot_pose->pose.position.x, 2) 
			+ pow(desired_robot_position->point.y - robot_pose->pose.position.y, 2));
		dis_pub.publish(dis);
		marker_robot->points.push_back(robot_pose->pose.position);
		init_point = true;
		// keypoint_time = human_msg->header.stamp;
		// if (count == 1){
		// 	time_duration = keypoint_time.toSec() - time_init;
		// 	if (time_duration != 0){
		// 		// Human velocity - Desired robot velocity
		// 		if (human_vel){
		// 			desired_robot_velocity->twist.linear.x = (human_msg->point.x + xOffset - dis_x - init_x)/time_duration;
		// 			desired_robot_velocity->twist.linear.y = (human_msg->point.y + yOffset - dis_x - init_y)/time_duration;
		// 			desired_robot_velocity->twist.linear.z = (human_msg->point.z + zOffset - dis_x - init_z)/time_duration;
		// 		}
		// 		else{
		// 			desired_robot_velocity->twist.linear.x = 0;
		// 			desired_robot_velocity->twist.linear.y = 0;
		// 			desired_robot_velocity->twist.linear.z = 0;
		// 		}
		// 		desired_robot_velocity->header.stamp = ros::Time::now();
		// 	}
		// }
		// else{
		// 	time_duration = keypoint_time.toSec() - desired_robot_position->header.stamp.toSec();
		// 	time_duration_msg.data = time_duration;
		// 	time_pub.publish(time_duration_msg);
		// 	if (time_duration == 0){
		// 		ROS_INFO("Zero time duration");
		// 	}
		// 	else{
		// 		ROS_INFO("Valid time duration");
		// 		// Human velocity - Desired robot velocity
		// 		if (human_vel){
		// 			desired_robot_velocity->twist.linear.x = (human_msg->point.x + xOffset - dis_x - init_x)/time_duration;
		// 			desired_robot_velocity->twist.linear.y = (human_msg->point.y + yOffset - dis_x - init_y)/time_duration;
		// 			desired_robot_velocity->twist.linear.z = (human_msg->point.z + zOffset - dis_x - init_z)/time_duration;
		// 		}
		// 		else{
		// 			desired_robot_velocity->twist.linear.x = 0;
		// 			desired_robot_velocity->twist.linear.y = 0;
		// 			desired_robot_velocity->twist.linear.z = 0;
		// 		}
		// 		desired_robot_velocity->header.stamp = ros::Time::now();
		// 		des_vel_pub.publish(*desired_robot_velocity);
		// 	}
		// }
	}

	count += 1;

	// Transitioned human coordinates - Desired robot coordinates
	des_x = human_msg->point.x + xOffset - dis_x;
	des_y = human_msg->point.y + yOffset - dis_y;
	des_z = human_msg->point.z + zOffset - dis_z;

	if (count > 1){
		ROS_INFO("Control point dis: %f", sqrt(pow(des_x,2)+pow(des_y, 2)));
		if (sqrt(pow(des_x, 2)+pow(des_y, 2)) < self_col_dis and des_z < z_dis){
			ROS_WARN("Control point leading to self collision.\n Waiting for valid control point");
		}
		else if (sqrt(pow(des_x, 2)+pow(des_y, 2)) > extention_dis){
			ROS_WARN("Control point leading to extention\n Waiting for valid control point");
		}
		else{
			ROS_INFO("Valid control point");
			desired_robot_position->point.x = human_msg->point.x + xOffset - dis_x;
			desired_robot_position->point.y = human_msg->point.y + yOffset - dis_y;
			desired_robot_position->point.z = human_msg->point.z + zOffset - dis_z;
			desired_robot_position->header.stamp = human_msg->header.stamp;
			des_pos_pub.publish(*desired_robot_position);
			control_points_pub.publish(*desired_robot_position);
			marker_human->header.stamp = ros::Time::now();
		    marker_human->points.push_back(desired_robot_position->point);
		  	vis_human_pub.publish(*marker_human);
		}
	}
	// desired_robot_position->header.stamp = ros::Time::now();

}

void state_callback (const trajectory_execution_msgs::PoseTwist::ConstPtr state_msg){
	if (vel_flag){
		// Computed robot velocity from robot position (bug in simulation) 
		vel_duration = ros::Time::now().toSec() - robot_pose->header.stamp.toSec();
		robot_velocity->twist.linear.x = (state_msg->pose.position.x - robot_pose->pose.position.x)/vel_duration;
		robot_velocity->twist.linear.y = (state_msg->pose.position.y - robot_pose->pose.position.y)/vel_duration;
		robot_velocity->twist.linear.z = (state_msg->pose.position.z - robot_pose->pose.position.z)/vel_duration;
		robot_velocity->header.stamp = ros::Time::now();
	}

	if (not vel_flag)
		vel_flag = true;

	// Robot position
	robot_pose->pose.position.x = state_msg->pose.position.x;
	robot_pose->pose.position.y = state_msg->pose.position.y;
	robot_pose->pose.position.z = state_msg->pose.position.z;
	robot_pose->header.stamp = ros::Time::now();

	if (received_point){
		if (count == 1){
			vel_control->linear.x = (init_x - robot_pose->pose.position.x);
			vel_control->linear.y = (init_y - robot_pose->pose.position.y);
			vel_control->linear.z = (init_z - robot_pose->pose.position.z);
			vel_control->angular.x = 0;
			vel_control->angular.y = 0;
			vel_control->angular.z = 0;
			pub.publish(*vel_control);
		}
		else if (count != 0){
			real_vel_pub.publish(*robot_velocity);
			
			time_now = ros::Time::now();

			// Velocity component
			vel_command->linear.x = desired_robot_position->point.x - robot_pose->pose.position.x;
			vel_command->linear.y = desired_robot_position->point.y - robot_pose->pose.position.y;
			vel_command->linear.z = desired_robot_position->point.z - robot_pose->pose.position.z;
			commanded_twist->twist.linear.x = vel_command->linear.x;
			commanded_twist->twist.linear.y = vel_command->linear.y;
			commanded_twist->twist.linear.z = vel_command->linear.z;
			commanded_twist->header.stamp = time_now;
			com_vel_pub.publish(*commanded_twist);

			// Acceleration component
			acc_command->linear.x = -robot_velocity->twist.linear.x;
			acc_command->linear.y = -robot_velocity->twist.linear.y;
			acc_command->linear.z = -robot_velocity->twist.linear.z;
			commanded_acc->accel.linear.x = acc_command->linear.x;
			commanded_acc->accel.linear.y = acc_command->linear.y;
			commanded_acc->accel.linear.z = acc_command->linear.z;
			commanded_acc->header.stamp = time_now;
			com_acc_pub.publish(*commanded_acc);

			acc[0] = Kx*acc_command->linear.x + Dx*vel_command->linear.x;
			acc[1] = Ky*acc_command->linear.y + Dy*vel_command->linear.y;
			acc[2] = Kz*acc_command->linear.z + Dz*vel_command->linear.z;
			
			vel_control_prev->linear.x = vel_control->linear.x;
			vel_control_prev->linear.y = vel_control->linear.y;
			vel_control_prev->linear.z = vel_control->linear.z;

			vel_control->linear.x += acc[0]*vel_duration;
			vel_control->linear.y += acc[1]*vel_duration;
			vel_control->linear.z += acc[2]*vel_duration;


			if (!std::isnan(vel_control->linear.x) and !std::isnan(vel_control->linear.y) and !std::isnan(vel_control->linear.y)){
				// ROS_INFO("Valid commanded velocity");
				// std::cout << *vel_control << std::endl;
				pub.publish(*vel_control);
			}
			else{
				ROS_INFO("The commanded acceleration is %f, %f, %f", acc[0], acc[1], acc[2]);
				ROS_INFO("The control time cycle is %f", vel_duration);
				ROS_INFO("The commanded submodules are: \n");
				std::cout << *vel_command << std::endl;
				std::cout << *acc_command << std::endl;
				vel_control->linear.x = vel_control_prev->linear.x;
				vel_control->linear.y = vel_control_prev->linear.y;
				vel_control->linear.z = vel_control_prev->linear.z;
			}
		}
		if (init_point)
			state_pub_high_f.publish(*state_msg);
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "reactive_control_node");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(3);
	spinner.start();
	safe_vel_control->linear.x = 0;
	safe_vel_control->linear.y = 0;
	safe_vel_control->linear.z = 0;

	// Control gains
	n.param("reactive_control_node/Dx", Dx, 0.0f);
	n.param("reactive_control_node/Dy", Dy, 0.0f);
	n.param("reactive_control_node/Dz", Dz, 0.0f);
	n.param("reactive_control_node/Kx", Kx, 0.0f);
	n.param("reactive_control_node/Ky", Ky, 0.0f);
	n.param("reactive_control_node/Kz", Kz, 0.0f);
	
	// Motion transition
	// n.param("reactive_control_node/xOffset", xOffset, 0.0f);
	// n.param("reactive_control_node/yOffset", yOffset, 0.0f);
	// n.param("reactive_control_node/zOffset", zOffset, 0.0f);
	
	// Human velocity
	n.param("reactive_control_node/human_vel", human_vel, false);

	// Self-collision and Extention distances
	n.param("reactive_control_node/self_col_dis", self_col_dis, 0.0f);
	n.param("reactive_control_node/z_dis", z_dis, 0.0f);
	n.param("reactive_control_node/extention_dis", extention_dis, 0.0f);

	// Human marker - Rviz
	marker_human->header.frame_id = "base_link";
	marker_human->type = visualization_msgs::Marker::LINE_STRIP;
	marker_human->action = visualization_msgs::Marker::ADD;
	marker_human->scale.x = 0.005;
    marker_human->scale.y = 0.005;
    marker_human->scale.z = 0.005;
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
	marker_robot->scale.x = 0.005;
    marker_robot->scale.y = 0.005;
    marker_robot->scale.z = 0.005;
    marker_robot->color.r = 0.0f;
    marker_robot->color.g = 0.0f;
    marker_robot->color.b = 1.0f;
    marker_robot->color.a = 1.0;
  	marker_robot->lifetime = ros::Duration(100);
  	
  	acc.resize(3);

  	// Topic names
	ee_state_topic = "/manos_cartesian_velocity_controller_sim/ee_state";
	ee_vel_command_topic = "/manos_cartesian_velocity_controller_sim/command_cart_vel";	

	// Publishers
	pub = n.advertise<geometry_msgs::Twist>(ee_vel_command_topic, 100);
	state_pub_high_f = n.advertise<trajectory_execution_msgs::PoseTwist>("/ee_position_high_f", 100);
	state_pub_low_f = n.advertise<geometry_msgs::PoseStamped>("/ee_position_low_f", 100);
	time_pub = n.advertise<std_msgs::Float64>("/time_topic", 100);
	dis_pub = n.advertise<std_msgs::Float64>("/response_topic", 100);
	vis_human_pub = n.advertise<visualization_msgs::Marker>("/vis_human_topic", 100);
	vis_robot_pub = n.advertise<visualization_msgs::Marker>("/vis_robot_topic", 100);
	control_points_pub = n.advertise<geometry_msgs::PointStamped>("trajectory_points_stamp", 100);	
	com_vel_pub = n.advertise<geometry_msgs::TwistStamped>("com_vel_topic", 100);
	com_acc_pub = n.advertise<geometry_msgs::AccelStamped>("com_acc_topic", 100);
	des_pos_pub = n.advertise<geometry_msgs::PointStamped>("des_pos_topic", 100);
	des_vel_pub = n.advertise<geometry_msgs::TwistStamped>("des_vel_topic", 100);
	real_vel_pub = n.advertise<geometry_msgs::TwistStamped>("real_vel_topic", 100);
	sim_robot_vel_check_pub = n.advertise<geometry_msgs::Vector3>("sim_robot_check_topic", 100);

	// Subscribers
	ros::Subscriber sub = n.subscribe(ee_state_topic, 100, state_callback);
	ros::Subscriber sub2 = n.subscribe("/trajectory_points", 100, human_motion_callback);
	
	ros::waitForShutdown();
}
