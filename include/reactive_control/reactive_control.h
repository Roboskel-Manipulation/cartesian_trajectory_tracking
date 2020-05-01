#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_execution_msgs/PoseTwist.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <memory>
#include <math.h>

ros::Publisher pub, state_pub, vis_human_pub, vis_robot_pub, dis_pub;

// std::shared_ptr<trajectory_execution_msgs::PoseTwist> robot_state = boost::make_shared<trajectory_execution_msgs::PoseTwist>();
geometry_msgs::PointStampedPtr desired_robot_position = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::PointStampedPtr robot_state = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::TwistPtr vel_control = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr safe_vel_control = boost::make_shared<geometry_msgs::Twist>();
std::shared_ptr<std::vector<float>> v1 = std::make_shared<std::vector<float>>();
std::shared_ptr<std::vector<float>> v2 = std::make_shared<std::vector<float>>();
visualization_msgs::MarkerPtr marker_human = boost::make_shared<visualization_msgs::Marker>();
visualization_msgs::MarkerPtr marker_robot = boost::make_shared<visualization_msgs::Marker>();

int count = 0;
float D, xOffset, yOffset, zOffset, var_gain;
bool received_point = false;
bool var, sim, init_point = false;
float init_x, init_y, init_z;
std_msgs::Float64 dis;
std::string ee_state_topic, ee_vel_command_topic;
