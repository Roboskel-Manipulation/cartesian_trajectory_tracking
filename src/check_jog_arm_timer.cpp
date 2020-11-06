#include <ros/ros.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>


// class Listener{
// private:
// 	ros::Publisher pub;
// 	std::shared_ptr<tf::TransformListener> ln_;
// 	std::shared_ptr<tf::StampedTransform> transform_;
// 	std_msgs::Float64 data;
// public:
// 	Listener();
// 	void callback1(const geometry_msgs::TwistStamped vel);
// 	friend void callback(const ros::TimerEvent&, Listener l);
// };

// Listener::Listener(){
// 	ros::NodeHandle n;
// 	pub = n.advertise<std_msgs::Float64>("times_topic", 100);
// 	ln_ = std::make_shared<tf::TransformListener>();
// 	transform_ = std::make_shared<tf::StampedTransform>();
// 	ln_->waitForTransform("base_link", "ee_link", ros::Time(0), ros::Duration(10));
// 	ROS_INFO_STREAM("Transform is ready");
// }

// void Listener::callback1(const geometry_msgs::TwistStamped vel){
// 	float time_diff = abs(vel.header.stamp.toSec() - transform_->stamp_.toSec());
// 	data.data = time_diff;
// 	pub.publish(data);
// 	ROS_INFO_STREAM("Time diffs: " << time_diff);
// }

// void callback(const ros::TimerEvent&, Listener l){
// 	l.ln_->lookupTransform("base_link", "ee_link", ros::Time(0), *l.transform_);
// }


// int main(int argc, char** argv){
// 	ros::init(argc, argv, "check_jog_arm");
// 	ros::NodeHandle nh_;
// 	Listener l;

// 	ros::Timer tf_timer = nh_.createTimer(ros::Duration(0.001), boost::bind(callback, _1, l));
// 	ros::Subscriber sub = nh_.subscribe("/tool_velocity", 100, &Listener::callback1, &l);

// 	ros::spin();
// }

int main(int argc, char** argv){
	ros::init(argc, argv, "check_jog_arm_timer");
	ros::NodeHandle nh;

	tf::TransformListener ln;
	tf::StampedTransform transform;
	geometry_msgs::PoseStamped pose;
	ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("ee_pose", 100);

	ln.waitForTransform("world", "ee_link", ros::Time(0), ros::Duration(2));

	while(ros::ok()){
		ln.lookupTransform("world", "ee_link", ros::Time(0), transform);
		pose.pose.position.x = transform.getOrigin().x();
		pose.pose.position.y = transform.getOrigin().y();
		pose.pose.position.z = transform.getOrigin().z();
		pose.header.stamp = transform.stamp_;
		pub.publish(pose);
		ros::Duration(0.008).sleep();
	}
	ros::spin();
}
