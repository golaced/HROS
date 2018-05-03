 /*
 * main.cpp
 *
 *      Author: JYF
 */

#include "PotDetect/impl.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include "image_transport/image_transport.h"  
#include "cv_bridge/cv_bridge.h"  
#include "sensor_msgs/image_encodings.h" 
#include "std_msgs/String.h"
#include <imav/Barrel.h>
#include <imav/BarrelList.h>
#include <opencv2/opencv.hpp>

#define FILTER_LEN	2*LIFE
using namespace std;
using namespace cv;

double max_area;
double min_area;

Mat frame;
Mat lastFrame;
Mat filtered_frame;
MySVM mSVM;
vector<Target> targets;
vector<Target> valid_targets;
string winName="pot";
Point pTrackPos;

char first_frame=1;
int center_x=0,center_y=0;
char b_tracking=0;
int filter_len=FILTER_LEN;
int window_len=0;
int ksize=3;
int start=0;
long c=0;
bool b_NewFrame;
bool b_FrameFinished;
bool bTracking;

ros::Publisher feedback_pub;
ros::Publisher ctrl_pub;

ros::Publisher barrelList_pub;
cv::VideoWriter videowriter;

void string2msg(const char* info, std_msgs::String& msg){
	stringstream ss;
	ss<<info;
	msg.data=ss.str();
}

void Targets2Barrels(vector<Target>& targets, imav::BarrelList& barrellist)
{
	for (size_t i = 0; i < targets.size(); i++)
	{
		imav::Barrel barrel;
		barrel.left = targets[i].location.x;
		barrel.top = targets[i].location.y;
		barrel.right = barrel.left+targets[i].location.width;
		barrel.bottom = barrel.top+targets[i].location.height;
		barrel.area = (barrel.right-barrel.left)*(barrel.bottom-barrel.top);
		barrel.color = imav::Barrel::COLOR_BLUE;
		barrellist.barrels.push_back(barrel);
	}
}
void getImageCallback(const sensor_msgs::ImageConstPtr& msg){
	try{
		frame=cv_bridge::toCvShare(msg, "bgr8")->image;
		//cout<<frame.rows<<", "<<frame.cols<<endl;
		if(frame.rows==0 or frame.cols==0)
			return;
		b_NewFrame=true;
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
				if(valid_targets.size()>0){	draw_bbox(dummy, valid_targets, 1);	}
			}
			else{
				filtered_frame.setTo(0);
				detect_bbox(frame, targets, mSVM, true);
				valid_targets=targets;
				draw_bbox(dummy, targets, 1);	
			}
		}
		std_msgs::String feedback_msg;
		string2msg("ok", feedback_msg);
		feedback_pub.publish(feedback_msg);
/////////////////////////////

		imav::BarrelList barrelList;
		Targets2Barrels(targets, barrelList);
		barrelList_pub.publish(barrelList);
		
		videowriter<<dummy;

/////////////////////////////
		//imshow(winName, dummy);
		//waitKey(1);
		if(start and filter_len==0){
			filter_len=FILTER_LEN+1;
			int max_dist=0;
			int min_dist=1e4;
			stringstream ctrl_ss;
			if(bTracking){
				int dx,dy,dist;
				for(unsigned int i=0;i<targets.size();++i){
					Target& t=targets[i];
					int cur_cx=t.location.x+(t.location.width>>1);
					int cur_cy=t.location.y+(t.location.height>>1);
					dist=abs(cur_cx-pTrackPos.x)+abs(cur_cy-pTrackPos.y);
					if(dist<min_dist){
						dx=center_x-cur_cx;
						dy=center_y-cur_cy;
						pTrackPos.x=cur_cx;
						pTrackPos.y=cur_cy;
					}
				}
				ctrl_ss<<dx<<","<<dy<<","<<dist;
				std_msgs::String ctrl_msg;
				string2msg(ctrl_ss.str().c_str(), ctrl_msg);
				ctrl_pub.publish(ctrl_msg);
			}
			else{
				for(unsigned int i=0;i<targets.size();++i){
					Target& t=targets[i];
					int dist=abs(t.location.x-center_x)+abs(t.location.y-center_y);
					if(dist>max_dist){
						max_dist=dist;
						pTrackPos.x=t.location.x+(t.location.width>>1);
						pTrackPos.y=t.location.y+(t.location.height>>1);
					}
				}
				string2msg("next", feedback_msg);
				feedback_pub.publish(feedback_msg);
			}
		}
		frame.copyTo(lastFrame);
		if(start){	--filter_len;	}
		if(start==0){	++start;	}
		++c;
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void getInfoCallback(const std_msgs::String::ConstPtr& msg){
	const char* data=msg->data.c_str();
	//cout<<"received data: "<<data<<endl;
	if(strcmp(data, "quit")==0){
		b_FrameFinished=true;
		cout<<"Real frame count: "<<c<<endl;
		destroyWindow(winName);
		ros::shutdown();
	}
	else if(strcmp(data, "ok")==0){
		b_FrameFinished=false;
	}
	else if(strcmp(data, "track")==0){
		//bTracking=true;
		cout<<"command for track"<<endl;
	}
	//first send the next command
	//ensure the new target position produced
	//then send track command
	else if(strcmp(data, "next")==0){
		bTracking=false;
	}
}

int __main(int argc, char** argv){
	//feat_train_svm(1e6);
	
	string path="/mnt/I/TestOpenCV/Videos/pot_train/samples/red_pot.jpg";
	Mat image=imread(path,0);
	MySVM mSVM=get_svm();
	cout<<"SVM load finish"<<endl;
	int res=feat_detect_svm(image, mSVM);
	cout<<"result: "<<res<<endl;
	
	return 0;

}
int main(int argc, char** argv){
	int interval=10;
	char key=0;
	videowriter.open("/home/odroid/workspace/video/1000.avi", CV_FOURCC('D', 'I', 'V', 'X'), 15, Size(640, 480));

	//namedWindow(winName);
	b_NewFrame=false;
	b_FrameFinished=false;	
	bTracking=false;
	pTrackPos.x=center_x;
	pTrackPos.y=center_y;
	ROS_INFO("geting_svm");
	mSVM=get_svm();
	ROS_INFO("get_svm success");
	startWindowThread();
	ros::init(argc, argv, "PotDetect");
	ros::NodeHandle n_handle;
	image_transport::ImageTransport it(n_handle);
	image_transport::Subscriber image_sub=it.subscribe("frame", 1, getImageCallback);
	ros::Subscriber inform_sub=n_handle.subscribe("inform", 500, getInfoCallback);
	feedback_pub=n_handle.advertise<std_msgs::String>("feedback_info",500);
	ctrl_pub=n_handle.advertise<std_msgs::String>("ctrl_info",100);
	
	barrelList_pub = n_handle.advertise<imav::BarrelList>("imav/PotDetect_Barrel",1);
	
	ros::Rate loop_rate(30);
	
	ros::spin();
	return 0;
}
