#include "reactive_control/reactive_control_double_integrator.h"

extern void control_points_callback(const geometry_msgs::PointStampedConstPtr control_point);

// Velcocity control callback 
void ee_state_callback (const cartesian_state_msgs::PoseTwist::ConstPtr state_msg){
	
	control_cycle_duration = ros::Time::now().toSec() - robot_pose->header.stamp.toSec();
	
	// Current robot pose
	robot_pose->pose.position.x = state_msg->pose.position.x;
	robot_pose->pose.position.y = state_msg->pose.position.y;
	robot_pose->pose.position.z = state_msg->pose.position.z;
	robot_pose->header.stamp = ros::Time::now();

	// Follow the human trajectory
	if (motion_started){
		// Spatial error
		spatial_error->twist.linear.x = desired_robot_position->point.x - state_msg->pose.position.x;
		spatial_error->twist.linear.y = desired_robot_position->point.y - state_msg->pose.position.y;
		spatial_error->twist.linear.z = desired_robot_position->point.z - state_msg->pose.position.z;
		spatial_error->header.stamp = ros::Time::now();
		spatial_error_pub.publish(*spatial_error);

		// Commanded acceleration
		acc->accel.linear.x = -Kx*state_msg->twist.linear.x + Dx*spatial_error->twist.linear.x;
		acc->accel.linear.y = -Ky*state_msg->twist.linear.y + Dy*spatial_error->twist.linear.y;
		acc->accel.linear.z = -Kz*state_msg->twist.linear.z + Dz*spatial_error->twist.linear.z;
		
		// Commanded velocity
		vel_control->linear.x += acc->accel.linear.x*control_cycle_duration;
		vel_control->linear.y += acc->accel.linear.y*control_cycle_duration;
		vel_control->linear.z += acc->accel.linear.z*control_cycle_duration;

		// Commanded velocity with time info for debugging purposes
		vel_control_stamp->twist.linear.x = vel_control->linear.x;
		vel_control_stamp->twist.linear.y = vel_control->linear.y;
		vel_control_stamp->twist.linear.z = vel_control->linear.z;
		vel_control_stamp->header.stamp = ros::Time::now();

		// Check if the distance from the table is more than 3cm, otherwise halt the motion
		if (state_msg->pose.position.z > 0.03){
			// Publish velocites if not NaN
			if (!std::isnan(vel_control->linear.x) and !std::isnan(vel_control->linear.y) and !std::isnan(vel_control->linear.y)){
				// ROS_INFO_STREAM("Valid commanded velocity");
				// ROS_INFO_STREAM("The control time cycle is " << control_cycle_duration);
				command_stamp_pub.publish(*vel_control_stamp);
				command_pub.publish(*vel_control);
			}
			else{
				ROS_INFO_STREAM("The commanded acceleration is " << acc->accel.linear.x << " " << acc->accel.linear.y << " " << acc->accel.linear.z);
				ROS_INFO_STREAM("The control time cycle is " << control_cycle_duration);
				ROS_INFO_STREAM("Gonna publish previous velocities");
			}
		}
		else{
			// Halt the robot motion if it close to the table
			command_pub.publish(zero_vel);
		}
		// Publish the ee state when the motion starts
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

	// Check robot limits
	n.param("reactive_control_node/check_robot_limits", check_robot_limits, true);
	
	// Self collision distances
	n.param("reactive_control_node/self_collision_limit", self_collision_limit, 0.0f);
	n.param("reactive_control_node/z_limit", z_limit, 0.0f);

	// Extention distance
	n.param("reactive_control_node/overextension_limit", overextension_limit, 0.0f);

	// Distance between consecutive valid points
	n.param("reactive_control_node/consecutive_points_distance", consecutive_points_distance, 0.0f);

	// Rotation angle
	// n.param("reactive_control_node/theta", theta, 0.0f);
	// theta = theta * M_PI / 180;
	
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
  	
  	// Topic names
  	n.param("reactive_control_node/state_topic", ee_state_topic, std::string("/ur3_cartesian_velocity_controller/ee_state"));
  	n.param("reactive_control_node/command_topic", ee_vel_command_topic, std::string("/ur3_cartesian_velocity_controller/command_cart_vel"));

	// Publishers
	command_pub = n.advertise<geometry_msgs::Twist>(ee_vel_command_topic, 100);
	robot_state_pub = n.advertise<cartesian_state_msgs::PoseTwist>("/ee_state_topic", 100);
	spatial_error_pub = n.advertise<geometry_msgs::TwistStamped>("/spatial_error_topic", 100);
	command_stamp_pub = n.advertise<geometry_msgs::TwistStamped>("/vel_command_stamp_topic", 100);
	vis_human_pub = n.advertise<visualization_msgs::Marker>("/vis_human_topic", 100);
	vis_robot_pub = n.advertise<visualization_msgs::Marker>("/vis_robot_topic", 100);

	// Subscribers
	ros::Subscriber ee_state_sub = n.subscribe(ee_state_topic, 100, ee_state_callback);
	ros::Subscriber control_points_sub = n.subscribe("/control_points_topic", 100, control_points_callback);

	ros::waitForShutdown();
}
