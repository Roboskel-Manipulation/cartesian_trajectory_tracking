#include "reactive_control/reactive_control_single_integrator.h"

bool valid_z = true;

void human_motion_callback(const geometry_msgs::PointStampedConstPtr human_msg){
	received_point = true;
	// Check for valid (no overextension and no self-collision) initial point)
	if (count <= 0){
		init_x = human_msg->point.x;
		init_y = human_msg->point.y;
		init_z = human_msg->point.z;
		std::cout << sqrt(pow(init_x, 2) + pow(init_y, 2)) << std::endl; 
		
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
			ROS_INFO_STREAM("Valid initial point");
		}
	}
	else{
		if (count == 1){
			dis_x = human_msg->point.x + xOffset - init_x;
			dis_y = human_msg->point.y + yOffset - init_y;
			dis_z = human_msg->point.z + zOffset - init_z;
			ROS_WARN_STREAM("The initial distances are " << dis_x <<  " " << dis_y << " " << dis_z);
		}

		// Distances for spatial responsiveness
		dis.data = sqrt(pow(desired_robot_position->point.x - robot_pose->pose.position.x, 2) 
			+ pow(desired_robot_position->point.y - robot_pose->pose.position.y, 2));
		dis_pub.publish(dis);
		dis_max.data = sqrt(pow(human_msg->point.x + xOffset - robot_pose->pose.position.x, 2) 
			+ pow(human_msg->point.y + yOffset - robot_pose->pose.position.y, 2));
		dis_max_pub.publish(dis_max);
		marker_robot->points.push_back(robot_pose->pose.position);
		init_point = true;
	}

	count += 1;
	// Desired point 
	des_x = human_msg->point.x + xOffset - dis_x;
	des_y = human_msg->point.y + yOffset - dis_y;
	des_z = human_msg->point.z + zOffset - dis_z;

	// Check if desired point is valid
	if (count > 1){
		ROS_INFO_STREAM("Control point dis: " << sqrt(pow(des_x, 2) + pow(des_y, 2)));
		if (sqrt(pow(des_x, 2) + pow(des_y, 2)) < self_col_dis and des_z < z_dis){
			ROS_WARN_STREAM("Control point leading to self collision.\nWaiting for valid control point");
		}
		else if (sqrt(pow(des_x, 2) + pow(des_y, 2)) > extention_dis){
			ROS_WARN_STREAM("Control point leading to overextention\nWaiting for valid control point");
		}
		else{
			ROS_INFO_STREAM("Valid control point");
			// Transitioned human coordinates - Desired robot coordinates
			desired_robot_position->point.x = des_x;
			desired_robot_position->point.y = des_y;
			desired_robot_position->point.z = des_z;
			desired_robot_position->header.stamp = robot_pose->header.stamp;

			control_points_pub.publish(*desired_robot_position);

			// Visualize human trajectory
			marker_human->header.stamp = ros::Time::now();
		    marker_human->points.push_back(desired_robot_position->point);
		  	vis_human_pub.publish(*marker_human);
		}
	}
}

void state_callback (const trajectory_execution_msgs::PoseTwist::ConstPtr state_msg){
	// Robot current position
	robot_pose->pose.position.x = state_msg->pose.position.x;
	robot_pose->pose.position.y = state_msg->pose.position.y;
	robot_pose->pose.position.z = state_msg->pose.position.z;
	robot_pose->header.stamp = ros::Time::now();


	if (received_point){
		// If the robot tends to hit the table, halt its motion
		if (robot_pose->pose.position.z < 0.03){
			vel_control->linear.x = 0;
			vel_control->linear.y = 0;
			vel_control->linear.z = 0;
			pub.publish(*vel_control);
			ROS_ERROR_STREAM("Invalid z point. Halting motion");
		}		
		// Get to the first point
		if (valid_z and count == 1){
			vel_control->linear.x = (init_x - robot_pose->pose.position.x);
			vel_control->linear.y = (init_y - robot_pose->pose.position.y);
			vel_control->linear.z = (init_z - robot_pose->pose.position.z);
			vel_control->angular.x = 0;
			vel_control->angular.y = 0;
			vel_control->angular.z = 0;
			vel_control->linear.x = abs(vel_control->linear.x) > VEL_X_MAX_INIT ? VEL_X_MAX_INIT*vel_control->linear.x/abs(vel_control->linear.x) : vel_control->linear.x;
			vel_control->linear.y = abs(vel_control->linear.y) > VEL_Y_MAX_INIT ? VEL_Y_MAX_INIT*vel_control->linear.y/abs(vel_control->linear.y) : vel_control->linear.y;
			vel_control->linear.z = abs(vel_control->linear.z) > VEL_Z_MAX_INIT ? VEL_Z_MAX_INIT*vel_control->linear.z/abs(vel_control->linear.z) : vel_control->linear.z;
			pub.publish(*vel_control);
		}
		// Follow the human trajectory
		else if (valid_z and count != 0){			
			// Construct gains based on euclidean distance
			// D = D_eucl * ||x_h - x_r||
			if (eucl_flag){
				v1->push_back(robot_pose->pose.position.x);
				v1->push_back(robot_pose->pose.position.y);
				v1->push_back(robot_pose->pose.position.z);
				v2->push_back(desired_robot_position->point.x);
				v2->push_back(desired_robot_position->point.y);
				v2->push_back(desired_robot_position->point.z);
				dis_points = euclidean_distance(v1, v2);
				Dx = Dy = Dz = D_eucl*dis_points;
				v1->clear();
				v2->clear();
			}
			// Construct gains based on exponential potential
			// Intuitively, increase the gain as long as the distance decreases
			else if (exp_flag){
				x_error = abs(robot_pose->pose.position.x - desired_robot_position->point.x);
				y_error = abs(robot_pose->pose.position.y - desired_robot_position->point.y);
				z_error = abs(robot_pose->pose.position.z - desired_robot_position->point.z);
				Dx = Ka/(1+exp(Ka_exp*(x_error-min_dis)))+Kb/(1+exp(-Kb_exp*(x_error-max_dis)))+c;
				Dy = Ka/(1+exp(Ka_exp*(y_error-min_dis)))+Kb/(1+exp(-Kb_exp*(y_error-max_dis)))+c;
				Dz = Ka/(1+exp(Ka_exp*(z_error-min_dis)))+Kb/(1+exp(-Kb_exp*(z_error-max_dis)))+c;
				ROS_INFO("The distance is: %f", euclidean_distance(v1, v2));
			}
			// Publish the gains
			gain_array.data.push_back(dis_points);
			gain_array.data.push_back(Dx);
			gain_array.data.push_back(Dy);
			gain_array.data.push_back(Dz);
			gain_array.data.push_back(ros::Time::now().toSec());
			gain_pub.publish(gain_array);
			gain_array.data.clear();

			// Spatial error
			error->twist.linear.x = desired_robot_position->point.x - robot_pose->pose.position.x;
			error->twist.linear.y = desired_robot_position->point.y - robot_pose->pose.position.y;
			error->twist.linear.z = desired_robot_position->point.z - robot_pose->pose.position.z;
			error->header.stamp = ros::Time::now();
			error_pub.publish(*error);

			// Velocity commands
			vel_control->linear.x = Dx*error->twist.linear.x;
			vel_control->linear.y = Dy*error->twist.linear.y;
			vel_control->linear.z = Dz*error->twist.linear.z;
			vel_control_stamp->twist.linear.x = vel_control->linear.x;
			vel_control_stamp->twist.linear.y = vel_control->linear.y;
			vel_control_stamp->twist.linear.z = vel_control->linear.z;
			vel_control_stamp->header.stamp = robot_pose->header.stamp;
			vel_control_stamp_pub.publish(*vel_control_stamp);
			pub.publish(*vel_control);

			// Visualize the robot trajectory in Rviz
			vis_robot_pub.publish(*marker_robot);

			// Publish the distance between human and robot
			dis_all.data = sqrt(pow(desired_robot_position->point.x - robot_pose->pose.position.x, 2) 
				+ pow(desired_robot_position->point.y - robot_pose->pose.position.y, 2));
			dis_all_pub.publish(dis_all);
		}
		if (init_point){
			robot_state_pub.publish(*state_msg);
		}
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

	// Self collision distances
	n.param("reactive_control_node/self_col_dis", self_col_dis, 0.0f);
	n.param("reactive_control_node/z_dis", z_dis, 0.0f);

	// Extention distance
	n.param("reactive_control_node/extention_dis", extention_dis, 0.0f);

	// Gain flags
	n.param("reactive_control_node/eucl_flag", eucl_flag, false);
	n.param("reactive_control_node/D_eucl", D_eucl, 10.0f);
	
	// Exponential dynamical field
	n.param("reactive_control_node/exp_flag", exp_flag, false);	
	n.param("reactive_control_node/Ka", Ka, 1.0f);	
	n.param("reactive_control_node/Kb", Kb, 1.0f);	
	n.param("reactive_control_node/Ka_exp", Ka_exp, 1.0f);	
	n.param("reactive_control_node/Kb_exp", Kb_exp, 1.0f);	
	n.param("reactive_control_node/min_dis", min_dis, 1.0f);	
	n.param("reactive_control_node/max_dis", max_dis, 1.0f);	
	n.param("reactive_control_node/c", c, 1.0f);

	// Simulation flag
	n.param("reactive_control_node/sim", sim, true);

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
	marker_robot->scale.x = 0.01;
    marker_robot->scale.y = 0.01;
    marker_robot->scale.z = 0.01;
    marker_robot->color.r = 0.0f;
    marker_robot->color.g = 0.0f;
    marker_robot->color.b = 1.0f;
    marker_robot->color.a = 1.0;
  	marker_robot->lifetime = ros::Duration(100);

  	// Topics according to sim flag
  	if (sim){
  		ee_state_topic = "/manos_cartesian_velocity_controller_sim/ee_state";
  		ee_vel_command_topic = "/manos_cartesian_velocity_controller_sim/command_cart_vel";	
  	}
  	else{
  		ee_state_topic = "/manos_cartesian_velocity_controller/ee_state";
  		ee_vel_command_topic = "/manos_cartesian_velocity_controller/command_cart_vel";  		
  	}

  	// Publishers
	pub = n.advertise<geometry_msgs::Twist>(ee_vel_command_topic, 100);
	robot_state_pub = n.advertise<trajectory_execution_msgs::PoseTwist>("/ee_state_topic", 100);
	control_points_pub = n.advertise<geometry_msgs::PointStamped>("trajectory_points_stamp", 100);	
	dis_pub = n.advertise<std_msgs::Float64>("/response_topic", 100);
	dis_all_pub = n.advertise<std_msgs::Float64>("/distance_topic", 100);
	dis_max_pub = n.advertise<std_msgs::Float64>("/dis_max_topic", 100);
	gain_pub = n.advertise<std_msgs::Float64MultiArray>("/gain_topic", 100);
	error_pub = n.advertise<geometry_msgs::TwistStamped>("/error_topic", 100);
	vel_control_stamp_pub = n.advertise<geometry_msgs::TwistStamped>("vel_control_stamp_topic", 100);
	vis_human_pub = n.advertise<visualization_msgs::Marker>("/vis_human_topic", 100);
	vis_robot_pub = n.advertise<visualization_msgs::Marker>("/vis_robot_topic", 100);

	// Subscribers
	ros::Subscriber sub = n.subscribe(ee_state_topic, 100, state_callback);
	ros::Subscriber sub2 = n.subscribe("/trajectory_points", 100, human_motion_callback);
	
	ros::waitForShutdown();
}