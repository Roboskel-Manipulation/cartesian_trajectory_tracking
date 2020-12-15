#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_execution_msgs/PoseTwist.h>

#include <tf/transform_datatypes.h>
#include <numeric>
#include <memory>
#include <math.h>


// Publishers
ros::Publisher command_pub, command_stamp_pub, spatial_error_pub, robot_state_pub, vis_human_pub, vis_robot_pub, control_points_pub;

// Current ee pose
geometry_msgs::PoseStampedPtr robot_pose = boost::make_shared<geometry_msgs::PoseStamped>();

// Commaded acceleration
geometry_msgs::AccelStampedPtr acc = boost::make_shared<geometry_msgs::AccelStamped>();

// Commanded velocity
geometry_msgs::TwistPtr vel_control = boost::make_shared<geometry_msgs::Twist>();

// Stamped commanded velocity for debugging
geometry_msgs::TwistStampedPtr vel_control_stamp = boost::make_shared<geometry_msgs::TwistStamped>();

// Spatial Error (Desired position - current position)
geometry_msgs::TwistStampedPtr spatial_error = boost::make_shared<geometry_msgs::TwistStamped>();

// First trajectory point
geometry_msgs::PointPtr init_point = boost::make_shared<geometry_msgs::Point>();

// Current control point (valid trajectory point)
geometry_msgs::PointStampedPtr desired_robot_position = boost::make_shared<geometry_msgs::PointStamped>();

// Last control point (valid trajectory point)
geometry_msgs::PointPtr last_valid_point = boost::make_shared<geometry_msgs::Point>();

// Current trajectory point
geometry_msgs::PointPtr candidate_point = boost::make_shared<geometry_msgs::Point>();

// distance between first and second trajectory points
geometry_msgs::PointPtr jump_dis = boost::make_shared<geometry_msgs::Point>();

// Zero velocity used for halting robot motion
geometry_msgs::Twist zero_vel;

// Marker messages for RViz visualization
visualization_msgs::MarkerPtr marker_human = boost::make_shared<visualization_msgs::Marker>();
visualization_msgs::MarkerPtr marker_robot = boost::make_shared<visualization_msgs::Marker>();

// Variables for checking trajectory points for self-collision or overextension
float self_collision_limit, z_limit, overextension_limit, consecutive_points_distance;
bool limit_flag, init_point_flag = true, second_point_flag = true;

// Gain variables
float Dx, Dy, Dz, Kx, Ky, Kz;

// Translation offset between human workspace and robot workspace
float xOffset, yOffset, zOffset;

// Produce velocity commands if received_point == true
bool received_point = false;

// Topic names
std::string ee_state_topic, ee_vel_command_topic;

float control_cycle_duration;
