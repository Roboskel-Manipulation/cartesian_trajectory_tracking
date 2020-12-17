#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// check robot limits variables
float self_collision_limit, z_limit, overextension_limit, consecutive_points_distance;
bool limit_flag, motion_started = false;
float xOffset_limit=0, yOffset_limit=0, zOffset_limit=0;

// Control points Publisher
ros::Publisher control_points_pub, vis_human_pub, vis_robot_pub;

// Variables used in the trajectory points callback
bool init_point_flag = true, second_point_flag = true;
float xOffset, yOffset, zOffset;
geometry_msgs::PoseStampedPtr robot_pose = boost::make_shared<geometry_msgs::PoseStamped>();

// First trajectory point
geometry_msgs::PointPtr init_point = boost::make_shared<geometry_msgs::Point>();

// Current control point (valid trajectory point)
geometry_msgs::PointStampedPtr desired_robot_position = boost::make_shared<geometry_msgs::PointStamped>();

// Last control point (valid trajectory point)
geometry_msgs::PointPtr last_valid_point = boost::make_shared<geometry_msgs::Point>();

// Current trajectory point
geometry_msgs::PointPtr candidate_point = boost::make_shared<geometry_msgs::Point>();

// Distance between first and second trajectory points
geometry_msgs::PointPtr jump_dis = boost::make_shared<geometry_msgs::Point>();

// Marker messages for RViz visualization
visualization_msgs::MarkerPtr marker_human = boost::make_shared<visualization_msgs::Marker>();
visualization_msgs::MarkerPtr marker_robot = boost::make_shared<visualization_msgs::Marker>();