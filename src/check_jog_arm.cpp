#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>

#include <numeric>
#include <vector>

class Listener{
private:
	std::vector<float> v;
	tf::TransformListener ln;
public:
	Listener();
	void callback(const geometry_msgs::TwistStamped vel);
};


Listener::Listener(){
	ros::NodeHandle n;
	tf::TransformListener ln;
	ln.waitForTransform("base_link", "ee_link", ros::Time(0), ros::Duration(3.0));
}

void Listener::callback(const geometry_msgs::TwistStamped vel){
	tf::StampedTransform transform;
	ros::Time now = ros::Time::now();
	// ln.waitForTransform("/base_link", "/ee_link", now, ros::Duration(0.1));
	ln.lookupTransform("/base_link", "/ee_link", ros::Time(0), transform);
	ROS_INFO_STREAM("Times: " << transform.stamp_.toNSec() << " " << vel.header.stamp.toNSec());
	ROS_INFO_STREAM("Time difference: " << abs(transform.stamp_.toSec() - vel.header.stamp.toSec()));
	v.push_back(abs(transform.stamp_.toSec() - vel.header.stamp.toSec()));
	if (v.size() >= 100){
		ROS_WARN_STREAM("Mean of time diffs: " << std::accumulate(v.begin(), v.end(), 0.0)/v.size());
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "check_jog_arm");
	ros::NodeHandle nh_;

	Listener listener;
	ros::Subscriber sub = nh_.subscribe("/tool_velocity", 100, &Listener::callback, &listener);
	ros::spin();
}