#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_execution_msgs/PoseTwist.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf/transform_datatypes.h>
#include <numeric>
#include <memory>
#include <math.h>

#define VEL_X_MAX 0.4
#define VEL_Y_MAX 0.4
#define VEL_Z_MAX 0.4
#define VEL_X_MAX_INIT 0.1
#define VEL_Y_MAX_INIT 0.1
#define VEL_Z_MAX_INIT 0.1

ros::Publisher pub, command_pub, error_pub, gain_pub, state_pub_high_f, state_pub_low_f, vis_human_pub, vis_robot_pub, dis_pub, dis_all_pub, dis_max_pub, control_points_pub;

geometry_msgs::PointStampedPtr desired_robot_position = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::PoseStampedPtr robot_pose = boost::make_shared<geometry_msgs::PoseStamped>();
geometry_msgs::TwistPtr vel_control = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::Vector3StampedPtr error = boost::make_shared<geometry_msgs::Vector3Stamped>();
std::shared_ptr<std::vector<float>> v1 = std::make_shared<std::vector<float>>();
std::shared_ptr<std::vector<float>> v2 = std::make_shared<std::vector<float>>();
visualization_msgs::MarkerPtr marker_human = boost::make_shared<visualization_msgs::Marker>();
visualization_msgs::MarkerPtr marker_robot = boost::make_shared<visualization_msgs::Marker>();

// init point and offset
float init_x, init_y, init_z, xOffset, yOffset, zOffset;

// Spatial responsiveness
std_msgs::Float64 dis, dis_all, dis_max;
float dis_points;

// Euclidean distance dynamical system
bool eucl_flag;
float D_eucl;

// Exponential dynamical system
bool exp_flag;
float D, Dx, Dy, Dz, Ka, Kb, min_dis, max_dis, Ka_exp, Kb_exp, c, x_error, y_error, z_error;

// Gain variables for publication
std_msgs::Float64 gain;
std_msgs::Float64MultiArray gain_array;

// Real HW or sim
bool sim;
std::string ee_state_topic, ee_vel_command_topic;

int count = 0;
bool received_point = false;
bool init_point = false;
float dis_x, dis_y, dis_z;
float self_col_dis, z_dis, extention_dis, des_x, des_y, des_z;

float euclidean_distance (std::shared_ptr<std::vector<float>> v1, std::shared_ptr<std::vector<float>> v2){
	float temp = 0;
	for (short int i=0; i<v1->size(); i++){
		temp += pow((v1->at(i) - v2->at(i)), 2);
	}
	return sqrt(temp);
}
