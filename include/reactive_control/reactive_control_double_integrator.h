#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <cartesian_state_msgs/PoseTwist.h>

#include <tf/transform_datatypes.h>
#include <numeric>
#include <memory>
#include <math.h>


// Publishers
ros::Publisher command_pub, command_stamp_pub, spatial_error_pub, robot_state_pub;
extern ros::Publisher vis_human_pub, vis_robot_pub, control_points_pub;

// Current ee pose
extern geometry_msgs::PoseStampedPtr robot_pose;

// Commaded acceleration
geometry_msgs::AccelStampedPtr acc = boost::make_shared<geometry_msgs::AccelStamped>();

// Commanded velocity
geometry_msgs::TwistPtr vel_control = boost::make_shared<geometry_msgs::Twist>();

// Stamped commanded velocity for debugging
geometry_msgs::TwistStampedPtr vel_control_stamp = boost::make_shared<geometry_msgs::TwistStamped>();

// Spatial Error (Desired position - current position)
geometry_msgs::TwistStampedPtr spatial_error = boost::make_shared<geometry_msgs::TwistStamped>();


// Zero velocity used for halting robot motion
geometry_msgs::Twist zero_vel;


// Gain variables
float Dx, Dy, Dz, Kx, Ky, Kz;

// Topic names
std::string ee_state_topic, ee_vel_command_topic;

float control_cycle_duration;

// Defined in utils.h
extern geometry_msgs::PointPtr last_valid_point, init_point, jump_dis, candidate_point;
extern geometry_msgs::PointStampedPtr desired_robot_position;

extern visualization_msgs::MarkerPtr marker_human;
extern visualization_msgs::MarkerPtr marker_robot;

extern float self_collision_limit, z_limit, overextension_limit, consecutive_points_distance;
extern bool limit_flag, init_point_flag, second_point_flag;

extern float xOffset, yOffset, zOffset;

extern bool motion_started;

extern bool check_robot_limits;