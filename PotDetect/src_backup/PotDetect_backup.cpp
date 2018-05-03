 /*
 * main.cpp
 *
 *      Author: JYF
 */

#include "PotDetect/impl.h"
#include <iostream>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include "image_transport/image_transport.h"  
#include "cv_bridge/cv_bridge.h"  
#include "sensor_msgs/image_encodings.h" 
#include "std_msgs/String.h"


#define FILTER_LEN	2*LIFE
using namespace std;
using namespace cv;

double max_area;
double min_area;

Mat frame;
bool b_NewFrame;
bool b_FrameFinished;

void getImageCallback(const sensor_msgs::ImageConstPtr& msg){
	try{
		frame=cv_bridge::toCvShare(msg, "bgr8")->image;
		b_NewFrame=true;
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void getInfoCallback(const std_msgs::String::ConstPtr& msg){
	const char* data=msg->data.c_str();
	cout<<"received data: "<<data<<endl;
	if(strcmp(data, "quit")==0){
		b_FrameFinished=true;
	}
	else if(strcmp(data, "ok")==0){
		b_FrameFinished=false;
	}
}

int main(int argc, char** argv){

	Mat lastFrame;
	Mat filtered_frame;
	MySVM mSVM;

	int interval=10;
	long c=0;
	char key=0;

	vector<Target> targets;
	vector<Target> valid_targets;
	string winName="pot";
	
	char first_frame=1;
	int center_x=0,center_y=0;
	char b_tracking=0;
	int filter_len=FILTER_LEN;
	int window_len=0;
	int ksize=3;
	int start=0;

	namedWindow(winName);
	b_NewFrame=false;
	b_FrameFinished=false;
	get_svm(mSVM);

	ros::init(argc, argv, "PotDetect");
	ros::NodeHandle n_handle;
	image_transport::ImageTransport it(n_handle);
	image_transport::Subscriber image_sub=it.subscribe("frame", 1, getImageCallback);
	ros::Subscriber inform_sub=n_handle.subscribe("frame_info", 1000, getInfoCallback);
	//ros::Rate loop_rate(5);
	ros::spin();
	
	/*
	while(n_handle.ok()){
		if(b_FrameFinished){	break;	}
		if(not b_NewFrame){	
			loop_rate.sleep();
			continue;	
		}
		b_NewFrame=false;
		blur(frame,frame,Size(ksize,ksize));
		Mat dummy;
		frame.copyTo(dummy);

		if(first_frame){
			first_frame=0;
			center_x=frame.cols/2;
			center_y=frame.rows/2;
			filtered_frame.create(dummy.size(),CV_8UC1);
		}
		if(start){
			if(filter_len>0){
				detect_bbox(lastFrame, targets, mSVM, false);
				//if(valid_targets.size()>0){	draw_bbox(dummy, valid_targets, 1);	}
			}
			else{
				filter_len=FILTER_LEN+1;
				filtered_frame.setTo(0);
				detect_bbox(frame, targets, mSVM, true);
				valid_targets=targets;
				draw_bbox(dummy, targets, 1);
			}
		}
		/*
		int max_dist=0;
		if(b_tracking){
			//start_track
			;
		}
		else{
			for(unsigned int i=0;i<targets.size();++i){
				Target& t=targets[i];
				int dist=abs(t.location.x-center_x)+abs(t.location.y-center_y);
				if(dist>max_dist){
					max_dist=dist;
				}
			}
		}
		
		imshow(winName, dummy);
		//key=waitKey(interval);
		frame.copyTo(lastFrame);
		if(start){	--filter_len;	}
		if(start==0){	++start;	}
		++c;
		ros::spinOnce();
		loop_rate.sleep();
	}
	//cout<<"Real frame count: "<<c<<endl;
	destroyWindow(winName);
	*/
	return 0;
}
