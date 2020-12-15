#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

// check robot limits variables
extern float self_collision_limit, z_limit, overextension_limit, consecutive_points_distance;
extern bool limit_flag, received_point;
extern geometry_msgs::PointPtr last_valid_point;

// Desired trajectory points used by the controller
extern geometry_msgs::PointStampedPtr desired_robot_position;

// Control points Publisher
extern ros::Publisher control_points_pub;

// Variables used in the trajectory points callback
extern bool init_point_flag, second_point_flag;
extern float xOffset, yOffset, zOffset;
extern geometry_msgs::PoseStampedPtr robot_pose;
extern geometry_msgs::PointPtr init_point, jump_dis, candidate_point;