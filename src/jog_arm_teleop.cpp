#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

geometry_msgs::PointStampedPtr desired_robot_position = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::PointStampedPtr robot_position = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::PointStampedPtr human_position = boost::make_shared<geometry_msgs::PointStamped>();
visualization_msgs::MarkerPtr marker_human = boost::make_shared<visualization_msgs::Marker>();
visualization_msgs::MarkerPtr marker_robot = boost::make_shared<visualization_msgs::Marker>();

float init_x, init_y, init_z;
float xOffset, yOffset, zOffset;
int count = 0;
float dis_x, dis_y, dis_z;
float des_x, des_y, des_z;
bool received_point = false;

ros::Publisher vis_human_pub, vis_robot_pub, control_points_pub;

void human_motion_callback(const geometry_msgs::PointStampedConstPtr human_msg){
	received_point = true;
	if (count == 0){
		init_x = human_msg->point.x;
		init_y = human_msg->point.y;
		init_z = human_msg->point.z;
		xOffset = robot_position->point.x - human_msg->point.x;
		yOffset = robot_position->point.y - human_msg->point.y;
		zOffset = robot_position->point.z - human_msg->point.z;
		init_x += xOffset;
		init_y += yOffset;
		init_z += zOffset;
		ROS_INFO_STREAM("Valid initial point: " << init_x << " " << init_y << " " << init_z);
	}
	else if (count == 1){
		dis_x = human_msg->point.x + xOffset - init_x;
		dis_y = human_msg->point.y + yOffset - init_y;
		dis_z = human_msg->point.z + zOffset - init_z;
		ROS_WARN_STREAM("The initial distances are " << dis_x << ", " << dis_y << ", " << dis_z);
	}

	count += 1;

	// Desired robot position
	des_x = human_msg->point.x + xOffset - dis_x;
	des_y = human_msg->point.y + yOffset - dis_y;
	des_z = human_msg->point.z + zOffset - dis_z;

	if (count > 1){
		// Transitioned human coordinates - Desired robot coordinates
		desired_robot_position->point.x = des_x;
		desired_robot_position->point.y = des_y;
		desired_robot_position->point.z = des_z;
		desired_robot_position->header.stamp = human_msg->header.stamp;
		ROS_INFO_STREAM("Human msg is " << ros::Time::now().toSec() - human_msg->header.stamp.toSec() << " secs old");
		human_position->point.x = des_x;
		human_position->point.y = des_y;
		human_position->point.z = des_z;
		human_position->header.stamp = robot_position->header.stamp;

		control_points_pub.publish(*human_position);

		marker_human->header.stamp = ros::Time::now();
	    marker_human->points.push_back(desired_robot_position->point);
	  	vis_human_pub.publish(*marker_human);

	  	marker_robot->header.stamp = ros::Time::now();
		marker_robot->points.push_back(robot_position->point);
		vis_robot_pub.publish(*marker_robot);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "jog_arm_teleop");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(3);
	spinner.start();

	float Dx, Dy, Dz;
	// Control gains
	nh.param("jog_arm_teleop/Dx", Dx, 0.0f);
	nh.param("jog_arm_teleop/Dy", Dy, 0.0f);
	nh.param("jog_arm_teleop/Dz", Dz, 0.0f);

	// Human marker - Rviz
	marker_human->header.frame_id = "base_link";
	marker_human->type = visualization_msgs::Marker::LINE_STRIP;
	marker_human->action = visualization_msgs::Marker::ADD;
	marker_human->scale.x = 0.005;
    marker_human->scale.y = 0.005;
    marker_human->scale.z = 0.005;
    marker_human->color.r = 0.0f;
    marker_human->color.g = 1.0f;
    marker_human->color.b = 0.0f;
    marker_human->color.a = 1.0;
  	marker_human->lifetime = ros::Duration(100);
	
	// Robot marker - Rviz
	marker_robot->header.frame_id = "base_link";
	marker_robot->header.stamp = ros::Time::now();
	marker_robot->type = visualization_msgs::Marker::LINE_STRIP;
	marker_robot->action = visualization_msgs::Marker::ADD;
	marker_robot->scale.x = 0.005;
    marker_robot->scale.y = 0.005;
    marker_robot->scale.z = 0.005;
    marker_robot->color.r = 0.0f;
    marker_robot->color.g = 0.0f;
    marker_robot->color.b = 1.0f;
    marker_robot->color.a = 1.0;
  	marker_robot->lifetime = ros::Duration(100);
  	
  	ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 100);
  	ros::Publisher eval_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("/command_topic", 100);
  	ros::Publisher ee_position_pub = nh.advertise<geometry_msgs::PointStamped>("ee_position_topic", 100);
  	ros::Publisher error_pub = nh.advertise<geometry_msgs::TwistStamped>("error_topic", 100);
  	control_points_pub = nh.advertise<geometry_msgs::PointStamped>("trajectory_points_stamp", 100);
  	vis_human_pub = nh.advertise<visualization_msgs::Marker>("vis_human_topic", 100);
  	vis_robot_pub = nh.advertise<visualization_msgs::Marker>("vis_robot_topic", 100);

  	tf::TransformListener ln;
  	geometry_msgs::TwistStampedPtr error = boost::make_shared<geometry_msgs::TwistStamped>();
  	geometry_msgs::TwistStampedPtr cmd = boost::make_shared<geometry_msgs::TwistStamped>();
  	ros::Duration sleep_rate(0.008);

  	bool tf_flag = ln.waitForTransform("base_link", "ee_link", ros::Time(0), ros::Duration(2));
  	if (tf_flag)
  		ROS_INFO_STREAM("Tf tree constructed properly");
  	else
  		ROS_ERROR_STREAM("Something is off with the tf");

  	ros::Subscriber sub = nh.subscribe("trajectory_points", 100, human_motion_callback);
  	while (ros::ok()){
  		tf::StampedTransform transform;
  		ln.lookupTransform("base_link", "ee_link", ros::Time(0), transform);
  		ROS_INFO_STREAM("Transform is " << ros::Time::now().toSec() - transform.stamp_.toSec() << " secs old.");
  		robot_position->header.stamp = transform.stamp_;
  		robot_position->point.x = transform.getOrigin().x();
  		robot_position->point.y = transform.getOrigin().y();
  		robot_position->point.z = transform.getOrigin().z();
  		if (received_point){
			if (count == 1){
				std::cout << *robot_position << std::endl;
				error->twist.linear.x = init_x - robot_position->point.x;
				error->twist.linear.y = init_y - robot_position->point.y;
				error->twist.linear.z = init_z - robot_position->point.z;
				error->header.stamp = ros::Time::now();
			}
			else{
				error->twist.linear.x = desired_robot_position->point.x - robot_position->point.x;
				error->twist.linear.y = desired_robot_position->point.y - robot_position->point.y;
				error->twist.linear.z = desired_robot_position->point.z - robot_position->point.z;
				error->header.stamp = ros::Time::now();			
			}
			// std::cout << "START" << std::endl;
			// std::cout << *desired_robot_position << std::endl;
			// std::cout << *error << std::endl;
			error_pub.publish(*error);
			if (count >= 2){
				ee_position_pub.publish(*robot_position);
				eval_cmd_pub.publish(*cmd);
			}

			cmd->header.stamp = ros::Time::now();
			cmd->twist.linear.x = Dx*error->twist.linear.x;
			cmd->twist.linear.y = Dy*error->twist.linear.y;
			cmd->twist.linear.z = Dz*error->twist.linear.z;
			pub.publish(*cmd);
  		}
		sleep_rate.sleep();
  	}
  	ros::waitForShutdown();
}