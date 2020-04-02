#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_execution_msgs/PoseTwist.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include <geometry_msgs/PointStamped.h>

#include <memory>
#include <math.h>

ros::Publisher pub, state_pub;
ros::Time beginTime;
ros::Time currentTime;

// std::shared_ptr<trajectory_execution_msgs::PoseTwist> robot_state = boost::make_shared<trajectory_execution_msgs::PoseTwist>();
geometry_msgs::PointStampedPtr desired_robot_position = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::PointStampedPtr robot_state = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::TwistPtr vel_control = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr safe_vel_control = boost::make_shared<geometry_msgs::Twist>();
std::shared_ptr<std::vector<float>> v1 = std::make_shared<std::vector<float>>();
std::shared_ptr<std::vector<float>> v2 = std::make_shared<std::vector<float>>();

int count = 0;
float D, sleep_rate, init_x, init_y, init_z, xOffset, yOffset;
bool state_flag = false;
bool init_flag = false;



float euclidean_distance (std::shared_ptr<std::vector<float>> v1, std::shared_ptr<std::vector<float>> v2){
	float temp = 0;
	for (short int i=0; i<v1->size(); i++){
		temp += pow((v1->at(i) - v2->at(i)), 2);
	}
	return sqrt(temp);
}

void human_motion_callback(const geometry_msgs::PointConstPtr human_msg){
	desired_robot_position->point.x = human_msg->x + xOffset;
	desired_robot_position->point.y = human_msg->y + yOffset;
	desired_robot_position->point.z = human_msg->z;
	desired_robot_position->header.stamp = ros::Time::now();
	// desired_robot_position->point.x = human_msg->keypoints[i].points.point.x + 0.6;
	// desired_robot_position->point.y = human_msg->keypoints[i].points.point.y + 0.5;
	// desired_robot_position->point.z = human_msg->keypoints[i].points.point.z;
	// desired_robot_position->header.stamp = human_msg->keypoints[i].points.header.stamp;
	count += 1;

	if (init_flag){
		v1->push_back(robot_state->point.x);
		v1->push_back(robot_state->point.y);
		v1->push_back(robot_state->point.z);
		v2->push_back(desired_robot_position->point.x);
		v2->push_back(desired_robot_position->point.y);
		v2->push_back(desired_robot_position->point.z);
		D = 10*euclidean_distance(v1, v2);
		v1->clear();
		v2->clear();
		ROS_INFO("%f", D);
		vel_control->linear.x = D*(desired_robot_position->point.x - robot_state->point.x);
		vel_control->linear.y = D*(desired_robot_position->point.y - robot_state->point.y);
		vel_control->linear.z = D*(desired_robot_position->point.z - robot_state->point.z);
		vel_control->angular.x = 0;
		vel_control->angular.y = 0;
		vel_control->angular.z = 0;
		state_pub.publish(*robot_state);
				
		currentTime = ros::Time::now();
		// ROS_INFO("Check for valid state...");
		// if (currentTime-beginTime > ros::Duration(15) and (robot_state->point.x > 0.45 or robot_state->point.y >0.45)){
		// 	// ROS_INFO("Published safe velocity");
		// 	pub.publish(safe_vel_control);
		// 	return;
		// }
		pub.publish(*vel_control);
		// if(currentTime-beginTime > ros::Duration(10)){
		// 	// ROS_INFO("Published motion following velocity");
		// 	// desired_robot_position->point.x = 0;
		// 	// desired_robot_position->point.y = 0;
		// 	// desired_robot_position->point.z = 0;
		// }
		// ros::Duration(sleep_rate).sleep();
	}
}

void state_callback (const trajectory_execution_msgs::PoseTwist::ConstPtr state_msg){
	// state_flag=false;
	robot_state->point.x = state_msg->pose.position.x;
	robot_state->point.y = state_msg->pose.position.y;
	robot_state->point.z = state_msg->pose.position.z;

	if (not init_flag){
		vel_control->linear.x = (init_x - robot_state->point.x);
		vel_control->linear.y = (init_y - robot_state->point.y);
		vel_control->linear.z = (init_z - robot_state->point.z);
		vel_control->angular.x = 0;
		vel_control->angular.y = 0;
		vel_control->angular.z = 0;
		pub.publish(*vel_control);
		if (abs(robot_state->point.x - init_x) <= 0.005 and abs(robot_state->point.y - init_y) <= 0.005 and abs(robot_state->point.z - init_z) <= 0.005){
			ROS_INFO("Reached first point");
			init_flag = true;
		}
	}


	// state_flag=true;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "reactive_control_node");
	ros::NodeHandle n;
	safe_vel_control->linear.x = 0;
	safe_vel_control->linear.y = 0;
	safe_vel_control->linear.z = 0;
	n.param("reactive_control_node/D", D, 0.0f);
	n.param("reactive_control_node/sleep_rate", sleep_rate, 0.0f);
	n.param("reactive_control_node/init_x", init_x, 0.0f);
	n.param("reactive_control_node/init_y", init_y, 0.0f);
	n.param("reactive_control_node/init_z", init_z, 0.0f);
	n.param("reactive_control_node/xOffset", xOffset, 0.0f);
	n.param("reactive_control_node/yOffset", yOffset, 0.0f);
	
	beginTime = ros::Time::now();
	pub = n.advertise<geometry_msgs::Twist>("/manos_cartesian_velocity_controller_sim/command_cart_vel", 100);
	state_pub = n.advertise<geometry_msgs::PointStamped>("/ee_position", 100);
	ros::Subscriber sub = n.subscribe("/manos_cartesian_velocity_controller_sim/ee_state", 100, state_callback);
	ros::Subscriber sub2 = n.subscribe("/raw_points", 100, human_motion_callback);

	ros::spin();
}