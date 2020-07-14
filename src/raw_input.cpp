#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <keypoint_3d_matching_msgs/Keypoint3d.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char** argv){
	ros::init(argc, argv, "raw_input");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("trajectory_points", 100);
	ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("vis_raw", 100);
	ros::Duration(0.5).sleep();

	std::ifstream file(argv[1]);
	std::string line;
	geometry_msgs::PointStamped point;
	std::vector<float> times;
	bool init_point = false;
	bool x_found=false, y_found=false, z_found=false, wrist=false;
	float sleep_rate=0.047, time_point;
	while (std::getline(file, line)){
		if (line.find("RW") != std::string::npos){
			wrist = true;
		}
		else if (line.find("LW") != std::string::npos){
			wrist = false;
		}
		if (line.find('x') != std::string::npos){
			std::vector<char> arr;
			for (short int i=11; i<line.size()-1; i++){
				arr.push_back(line[i]);
			}
			std::string arr_str(arr.begin(), arr.end());
			try{
				if (wrist){
					point.point.x = std::stof(arr_str);
					x_found = true;
				}
			}
			catch (int e){
				std::cout << e << std::endl;
			}
		}
		if (line.find('y') != std::string::npos and x_found){
			std::vector<char> arr;
			for (short int i=11; i<line.size()-1; i++){
				arr.push_back(line[i]);
			}
			std::string arr_str(arr.begin(), arr.end());
			try{
				if (wrist){
					point.point.y = std::stof(arr_str);
					y_found = true;
				}
			}
			catch (int e){
				std::cout << e << std::endl;
			}
		}
		if (line.find('z') != std::string::npos and x_found and y_found){
			std::vector<char> arr;
			for (short int i=11; i<line.size()-1; i++){
				arr.push_back(line[i]);
			}
			std::string arr_str(arr.begin(), arr.end());
			try{
				if (wrist){
					point.point.z = std::stof(arr_str);
					z_found = true;
					time_point = ros::Time::now().toSec();
				}
			}
			catch (int e){
				std::cout << e << std::endl;
			}
		}
		if (not init_point and x_found and y_found and z_found){
			std::cout << point << std::endl;
			init_point = true;
			pub.publish(point);
			ros::Duration(10).sleep();
			x_found = false;
			y_found = false;
			z_found = false;
		}
		else if (x_found and y_found and z_found){
			pub.publish(point);
			std::cout << sleep_rate << std::endl;
			ros::Duration(sleep_rate).sleep();
			x_found = false;
			y_found = false;
			z_found = false;
			if (times.size() >= 1){
				sleep_rate = time_point - times[times.size()-1];
				times.push_back(time_point);
			}
			else{
				sleep_rate = 0.047;
				times.push_back(time_point);
			}
		}
	}
}

