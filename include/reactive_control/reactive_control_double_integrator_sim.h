#include <reactive_control/reactive_control_double_integrator.h>

ros::Publisher real_robot_vel_pub;

// Robot velocity
geometry_msgs::TwistStampedPtr robot_velocity = boost::make_shared<geometry_msgs::TwistStamped>();

// Compute robot velocities if two or more state msgs are available
bool vel_flag = false; 