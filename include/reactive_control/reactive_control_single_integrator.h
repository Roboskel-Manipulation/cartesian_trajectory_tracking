#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <numeric>
#include <memory>
#include <math.h>

// Publishers
ros::Publisher command_pub, command_stamp_pub, spatial_error_pub, robot_state_pub;
extern ros::Publisher vis_human_pub, vis_robot_pub, control_points_pub;

// Current ee pose
extern geometry_msgs::PoseStampedPtr robot_pose;


// desired_position and current_position variables are used if variable gains based on euclidean distance are used
// Desired robot position
geometry_msgs::PointPtr desired_position = boost::make_shared<geometry_msgs::Point>();

// Current robot position
geometry_msgs::PointPtr current_position = boost::make_shared<geometry_msgs::Point>();

// Commanded velocity
geometry_msgs::TwistPtr vel_control = boost::make_shared<geometry_msgs::Twist>();

// Stamped commanded velocity for debugging
geometry_msgs::TwistStampedPtr vel_control_stamp = boost::make_shared<geometry_msgs::TwistStamped>();

// Spatial Error (desired - real)
geometry_msgs::TwistStampedPtr spatial_error = boost::make_shared<geometry_msgs::TwistStamped>();

// Zero velocity used for halting robot motion
geometry_msgs::Twist zero_vel;

// Gain variables
float D, Dx, Dy, Dz;

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

// Topic names
std::string ee_state_topic, ee_vel_command_topic;

float control_cycle_duration;

float dis_points;

// Euclidean distance dynamical system
bool eucl_flag;
float D_eucl;

// Exponential dynamical system
bool exp_flag;
float Ka, Kb, min_dis, max_dis, Ka_exp, Kb_exp, c, x_error, y_error, z_error;

// Check sim flag and topic names
int pos=0, idx;


