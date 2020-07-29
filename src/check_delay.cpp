#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_execution_msgs/PointArray.h>
// #include <std_msgs/double64MultiArray.h>
// #include <std_msgs/double64.h>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <unistd.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "check_delay");
	ros::NodeHandle nh;
	std::ifstream file;
	file.open(argv[1]);
	std::string s;
	if (file.is_open()){
		std::vector<std::vector<double>> points;
		while (std::getline(file, s)){
			std::vector<char> v;
			std::vector<double> v_point;
			for (auto& c:s){
				if (c != ','){
					v.push_back(c);
				}
				else{

					std::string point_obj(v.begin(), v.end());
					std::cout << point_obj << std::endl;
					std::cout << std::stod(point_obj) << std::endl;
					v_point.push_back(std::stof(point_obj));
					
					v.clear();
				}
			}
			points.push_back(v_point);
		}

		std::vector<double> v;
		for (auto& a:points){
			// std::cout << a[3] << std::endl;
			v.push_back(a[3]);
		}

		std::vector<double> yaml, now;
		for (auto& i:v){
			if (yaml.size() >= 1){
				ros::Duration(i-yaml[yaml.size()-1]).sleep();
			}
			yaml.push_back(i);
			now.push_back(ros::Time::now().toSec());
		}

		for (int i=0; i<yaml.size(); i++){
			// yaml[i] = yaml[i] - yaml[0];
			// now[i] = now[i] - now[0];
			// std::cout << yaml[i] << " " << now[i] << std::endl;
		}

		
		// for (auto& a : x)
		// 	std::cout << a << std::endl;
	}	
}