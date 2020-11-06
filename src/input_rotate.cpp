#include <ros/ros.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include <geometry_msgs/PointStamped.h>

#include <math.h>
#include <cmath>

ros::Publisher pub;
float theta;
geometry_msgs::PointStampedPtr point = boost::make_shared<geometry_msgs::PointStamped>();

void callback(const keypoint_3d_matching_msgs::Keypoint3d_list keypoints){
	for (int i=0; i < keypoints.keypoints.size(); ++i){
		if (keypoints.keypoints[i].name == "RWrist"){
			float x = keypoints.keypoints[i].points.point.x;
			float y = keypoints.keypoints[i].points.point.y;
			float r = sqrt(pow(x, 2) + pow(y, 2));
			point->header.stamp = ros::Time::now();
			point->point.x = r * cos(atan(y/x) - theta);
			point->point.y = r * sin(atan(y/x) - theta);
			pub.publish(*point);
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "input_rotate");
	ros::NodeHandle nh;

	nh.param("reactive_node/theta", theta, 0.0f);
	theta = theta * M_PI / 180;
	ros::Subscriber sun = nh.subscribe("trajectory_points", 10, callback);
	pub = nh.advertise<geometry_msgs::PointStamped>("trajectory_points1", 10);

	ros::spin();
}