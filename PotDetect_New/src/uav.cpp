#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/String.h"

using namespace std;
using namespace cv;

Point pTrackPos;

void informCallback(const std_msgs::String::ConstPtr& msg){
	const char* data=msg->data.c_str();
	if(strcmp(data, "quit")==0){
		ros::shutdown();
	}
}

void ctrlInfoCallback(const std_msgs::String::ConstPtr& msg){
	const char* data=msg->data.c_str();
	if(strcmp(data,"no-target")==0){
		cout<<"no target"<<endl;
		return;
	}
	stringstream ctrl_ss(data);
	int dx,dy,dist;
	char ch;
	ctrl_ss>>dx;
	ctrl_ss>>ch;
	ctrl_ss>>dy;
	ctrl_ss>>ch;
	ctrl_ss>>dist;
	cout<<dx<<","<<dy<<","<<dist<<endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "uav");
	ros::NodeHandle n_handle;
	ros::Subscriber ctrl_sub=n_handle.subscribe("ctrl_info", 500, ctrlInfoCallback);
	ros::Subscriber info_sub=n_handle.subscribe("inform", 500, informCallback);
	ros::spin();
}