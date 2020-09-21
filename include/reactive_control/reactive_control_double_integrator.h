#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_execution_msgs/PoseTwist.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/AccelStamped.h>

#include <tf/transform_datatypes.h>
#include <numeric>
#include <memory>
#include <math.h>

#define VEL_X_MAX 1
#define VEL_Y_MAX 1
#define VEL_Z_MAX 1
#define VEL_X_MAX_INIT 0.1
#define VEL_Y_MAX_INIT 0.1
#define VEL_Z_MAX_INIT 0.1

ros::Publisher final_command_pub, sim_robot_vel_check_pub, real_vel_pub, des_vel_pub, des_pos_pub, pub, com_vel_pub, com_acc_pub, gain_pub, state_pub_high_f, state_pub_low_f, vis_human_pub, vis_robot_pub, dis_pub, dis_all_pub, dis_max_pub, control_points_pub;

geometry_msgs::PointStampedPtr desired_robot_position = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::TwistStampedPtr desired_robot_velocity = boost::make_shared<geometry_msgs::TwistStamped>();

geometry_msgs::PoseStampedPtr robot_pose = boost::make_shared<geometry_msgs::PoseStamped>();
geometry_msgs::TwistStampedPtr robot_velocity = boost::make_shared<geometry_msgs::TwistStamped>();

geometry_msgs::PointStampedPtr human_position = boost::make_shared<geometry_msgs::PointStamped>();

geometry_msgs::TwistPtr vel_control = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistStampedPtr  command_control = boost::make_shared<geometry_msgs::TwistStamped>();
geometry_msgs::TwistPtr safe_vel_control = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr vel_command = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::AccelPtr acc_command = boost::make_shared<geometry_msgs::Accel>();
geometry_msgs::TwistPtr vel_control_prev = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistStampedPtr commanded_twist = boost::make_shared<geometry_msgs::TwistStamped>();
geometry_msgs::AccelStampedPtr commanded_acc = boost::make_shared<geometry_msgs::AccelStamped>();

std::shared_ptr<std::vector<float>> v1 = std::make_shared<std::vector<float>>();
std::shared_ptr<std::vector<float>> v2 = std::make_shared<std::vector<float>>();

visualization_msgs::MarkerPtr marker_human = boost::make_shared<visualization_msgs::Marker>();
visualization_msgs::MarkerPtr marker_robot = boost::make_shared<visualization_msgs::Marker>();

ros::Time timenow, time_now;

int count = 0;
float D, Dx, Dy, Dz, xOffset, yOffset, zOffset, var_gain, xGoal, yGoal, zGoal;
bool received_point = false;
bool var, sim, init_point = false, vel_flag=false, human_vel;
float init_x, init_y, init_z, temp_z;
std_msgs::Float64 dis;
std_msgs::Float64 gain;
std::string ee_state_topic, ee_vel_command_topic;
std::vector<float> D_v;
float start_time, end_time, sum_time;
std_msgs::Float64 time_duration_msg;
std::vector<float> acc;
float vel_duration;
float Kx, Ky, Kz, dis_x, dis_y, dis_z;
float time_duration, time_init;
ros::Time keypoint_time;
ros::Publisher time_pub;
float self_col_dis, z_dis, extention_dis;
float des_x, des_y, des_z;
