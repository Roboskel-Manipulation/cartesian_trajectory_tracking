#include "reactive_control/reactive_control.h"

float euclidean_distance (std::shared_ptr<std::vector<float>> v1, std::shared_ptr<std::vector<float>> v2){
	float temp = 0;
	for (short int i=0; i<v1->size(); i++){
		temp += pow((v1->at(i) - v2->at(i)), 2);
	}
	return sqrt(temp);
}

void human_motion_callback(const geometry_msgs::PointConstPtr human_msg){
	ROS_INFO("Received point");
	received_point = true;

	if (count == 0){
		init_x = human_msg->x + xOffset;
		init_y = human_msg->y + yOffset;
		init_z = human_msg->z + zOffset;
	}
	else{
		timenow = ros::Time::now();
		desired_robot_position->header.stamp = timenow;
		robot_state->header.stamp = timenow;
		state_pub_low_f.publish(*robot_state);
	  	vis_robot_pub.publish(*marker_robot);
		if (temp_z > 10){
			dis.data = sqrt(pow(desired_robot_position->point.x - robot_state->point.x, 2) 
				+ pow(desired_robot_position->point.y - robot_state->point.y, 2));
			dis_pub.publish(dis);
		}
		marker_robot->points.push_back(robot_state->point);
		init_point = true;
	}

	count += 1;
	std::cout << count << std::endl;
	// desired_robot_position->point.x = human_msg->keypoints[i].points.point.x + 0.6;
	// desired_robot_position->point.y = human_msg->keypoints[i].points.point.y + 0.5;
	// desired_robot_position->point.z = human_msg->keypoints[i].points.point.z;
	// desired_robot_position->header.stamp = human_msg->keypoints[i].points.header.stamp;
	desired_robot_position->point.x = human_msg->x + xOffset;
	desired_robot_position->point.y = human_msg->y + yOffset;
	temp_z = human_msg->z;
	if (temp_z > 10){
		desired_robot_position->point.z = human_msg->z - 10 + zOffset;
	}
	else{
		desired_robot_position->point.z = human_msg->z + zOffset;		
	}
	desired_robot_position->header.stamp = ros::Time::now();
	control_points_pub.publish(*desired_robot_position);
	marker_human->header.stamp = ros::Time::now();

    marker_human->points.push_back(desired_robot_position->point);
  	vis_human_pub.publish(*marker_human);
	// std::cout << count << std::endl;	
	if (var){
		v1->push_back(robot_state->point.x);
		v1->push_back(robot_state->point.y);
		v1->push_back(robot_state->point.z);
		v2->push_back(desired_robot_position->point.x);
		v2->push_back(desired_robot_position->point.y);
		v2->push_back(desired_robot_position->point.z);
		D = var_gain*euclidean_distance(v1, v2);
		v1->clear();
		v2->clear();
		ROS_INFO("The gain is: %f", D);
		D_v.push_back(D);
		if (D_v.size() > 1){
			std::cout << std::accumulate(D_v.begin(), D_v.end(), 0.0)/D_v.size() << std::endl;
		}
	}
}

void state_callback (const trajectory_execution_msgs::PoseTwist::ConstPtr state_msg){
	robot_state->point.x = state_msg->pose.position.x;
	robot_state->point.y = state_msg->pose.position.y;
	robot_state->point.z = state_msg->pose.position.z;
	robot_state->header.stamp = ros::Time::now();

	if (received_point){
		if (count == 1){
			vel_control->linear.x = (desired_robot_position->point.x - robot_state->point.x);
			vel_control->linear.y = (desired_robot_position->point.y - robot_state->point.y);
			vel_control->linear.z = (desired_robot_position->point.z - robot_state->point.z);
			vel_control->angular.x = 0;
			vel_control->angular.y = 0;
			vel_control->angular.z = 0;
			pub.publish(*vel_control);
		}
		else{
			vel_control->linear.x = D*(desired_robot_position->point.x - robot_state->point.x);
			vel_control->linear.y = D*(desired_robot_position->point.y - robot_state->point.y);
			vel_control->linear.z = D*(desired_robot_position->point.z - robot_state->point.z);
			vel_control->angular.x = 0;
			vel_control->angular.y = 0;
			vel_control->angular.z = 0;
			pub.publish(*vel_control);
		}
		if (abs(robot_state->point.x - init_x) < 0.005
		 and abs(robot_state->point.y - init_y) < 0.005 
		 and abs(robot_state->point.z - init_z) < 0.005
		 and not init_point){
		 	ROS_INFO("Reached the first point");
		}
		if (init_point){
			state_pub_high_f.publish(*state_msg);
		}
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
	n.param("reactive_control_node/D", D, 0.0f);
	n.param("reactive_control_node/xOffset", xOffset, 0.0f);
	n.param("reactive_control_node/yOffset", yOffset, 0.0f);
	n.param("reactive_control_node/zOffset", zOffset, 0.0f);
	n.param("reactive_control_node/var", var, false);
	n.param("reactive_control_node/var_gain", var_gain, 10.0f);
	n.param("reactive_control_node/sim", sim, true);
	
	marker_human->header.frame_id = "base_link";
	marker_human->type = visualization_msgs::Marker::LINE_STRIP;
	marker_human->action = visualization_msgs::Marker::ADD;
	marker_human->scale.x = 0.01;
    marker_human->scale.y = 0.01;
    marker_human->scale.z = 0.01;
    marker_human->color.r = 0.0f;
    marker_human->color.g = 1.0f;
    marker_human->color.b = 0.0f;
    marker_human->color.a = 1.0;
  	marker_human->lifetime = ros::Duration(100);
	
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
  	
  	if (sim){
  		ee_state_topic = "/manos_cartesian_velocity_controller_sim/ee_state";
  		ee_vel_command_topic = "/manos_cartesian_velocity_controller_sim/command_cart_vel";	
  	}
  	else{
  		ee_state_topic = "/manos_cartesian_velocity_controller/ee_state";
  		ee_vel_command_topic = "/manos_cartesian_velocity_controller/command_cart_vel";  		
  	}

	pub = n.advertise<geometry_msgs::Twist>(ee_vel_command_topic, 100);
	state_pub_high_f = n.advertise<trajectory_execution_msgs::PoseTwist>("/ee_position_high_f", 100);
	state_pub_low_f = n.advertise<geometry_msgs::PointStamped>("/ee_position_low_f", 100);
	dis_pub = n.advertise<std_msgs::Float64>("/response_topic", 100);
	vis_human_pub = n.advertise<visualization_msgs::Marker>("/vis_human_topic", 100);
	vis_robot_pub = n.advertise<visualization_msgs::Marker>("/vis_robot_topic", 100);
	control_points_pub = n.advertise<geometry_msgs::PointStamped>("trajectory_points_stamp", 100);	
	ros::Subscriber sub = n.subscribe(ee_state_topic, 100, state_callback);
	ros::Subscriber sub2 = n.subscribe("/trajectory_points", 100, human_motion_callback);
	
	ros::waitForShutdown();
}