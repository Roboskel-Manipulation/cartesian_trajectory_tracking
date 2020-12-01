#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_execution_msgs/PoseTwist.h>
#include <vector>

ros::Publisher pub;
geometry_msgs::Twist cmd;
std::vector<float> static_vel = {-0.3, 0, 0};
std::vector<float> desired_pos = {0.2, 0, 0};
std::vector<float> acc(3);
bool init = true;

void state_callback(const trajectory_execution_msgs::PoseTwist::ConstPtr state){
	if (init){
		desired_pos[1] = state->pose.position.y;
		desired_pos[2] = state->pose.position.z;
		init = false;
	}
	std::cout << desired_pos[1] << " " << desired_pos[2] << std::endl;
	// Single Integrator
	// cmd.linear.x = static_vel[0] + 2*(desired_pos[0]-state->pose.position.x);
	// cmd.linear.y = static_vel[1] + 2*(desired_pos[1]-state->pose.position.y);
	// cmd.linear.z = static_vel[2] + 2*(desired_pos[2]-state->pose.position.z);

	// Double Integrator
	acc[0] = -1*state->twist.linear.x + 1*(desired_pos[0]-state->pose.position.x);
	acc[1] = -1*state->twist.linear.y + 1*(desired_pos[1]-state->pose.position.y);
	acc[2] = -1*state->twist.linear.z + 1*(desired_pos[2]-state->pose.position.z);

	cmd.linear.x += acc[0]*0.008;
	cmd.linear.y += acc[1]*0.008;
	cmd.linear.z += acc[2]*0.008;
	pub.publish(cmd);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "check_si");
	ros::NodeHandle nh;

	pub = nh.advertise<geometry_msgs::Twist>("/manos_cartesian_velocity_controller_sim/command_cart_vel", 10);
	ros::Duration(2).sleep();
	ros::Subscriber sub = nh.subscribe("/manos_cartesian_velocity_controller_sim/ee_state", 10, state_callback);

	ros::spin();
}