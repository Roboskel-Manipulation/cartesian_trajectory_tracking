#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "jog_arm_check");
	ros::NodeHandle nh_;

	ros::Publisher pub = nh_.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 10);

	ros::Duration(2).sleep();

	geometry_msgs::TwistStamped cmd;
	while (ros::ok()){
		cmd.header.stamp = ros::Time::now();
		// cmd.header.frame_id = "world";
		cmd.twist.linear.x = 0;
		cmd.twist.linear.y = 0;
		cmd.twist.linear.z = 0.1;
		pub.publish(cmd);
	}

	ros::spin();
}