/*
 * main.cpp
 *
 *      Author: JYF
 */

#include "PotDetect_New/impl.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <imav/GameMode.h>
#include <imav/Barrel.h>
#include <imav/BarrelList.h>
#include <imav/imavFunctions.h>

#define FILTER_LEN 2 * LIFE
using namespace std;
using namespace cv;

double max_area;
double min_area;
Logistic log_hog;
Logistic log_hist;
int histFeatLen;
int hogFeatLen;
int pixFeatLen;
int bowFeatLen;

Mat lastFrame;
Mat filtered_frame;

vector<Target> targets;
vector<Target> valid_targets;

string winName = "pot";
Point pTrackPos;

char first_frame = 1;
int center_x = 0, center_y = 0;
char b_tracking = 0;
int filter_len = FILTER_LEN;
int window_len = 0;
int ksize = 3;
int start = 0;
long c = 0;
bool b_NewFrame;
bool b_FrameFinished;
bool b_Tracking;
bool b_Marking;

ros::Publisher feedback_pub;
ros::Publisher ctrl_pub;
ros::Publisher barrelList_pub;

void dispTargets(vector<Target> &targets)
{
	ROS_INFO("Target number:%d", (int)targets.size());
	for (size_t i = 0; i < targets.size(); i++)
	{
		ROS_INFO("%d %d %d %d",
				 targets[i].location.x, targets[i].location.y, targets[i].location.width, targets[i].location.height);
	}
}

void Targets2Barrels(vector<Target> &targets, imav::BarrelList &barrellist)
{
	for (size_t i = 0; i < targets.size(); i++)
	{
		imav::Barrel barrel;
		barrel.left = targets[i].location.x;
		barrel.top = targets[i].location.y;
		barrel.right = barrel.left + targets[i].location.width;
		barrel.bottom = barrel.top + targets[i].location.height;
		barrel.area = (barrel.right - barrel.left) * (barrel.bottom - barrel.top);
		barrel.color = imav::Barrel::COLOR_BLUE;
		barrellist.barrels.push_back(barrel);
	}
}

void calcFeatLens()
{
	histFeatLen = 256 / HIST_SEG;
	histFeatLen = histFeatLen * histFeatLen * histFeatLen;

	hogFeatLen = ((HOG_SIZE - BLOCK_SIZE) / STRIDE + 1) * ((HOG_SIZE - BLOCK_SIZE) / STRIDE + 1) * 36;
	pixFeatLen = PIX_SIZE * PIX_SIZE * 3;

	bowFeatLen = 256 / BOW_SEG;
	bowFeatLen = bowFeatLen * bowFeatLen * bowFeatLen * BOW_NUM * BOW_NUM;
}

void string2msg(const char *info, std_msgs::String &msg)
{
	stringstream ss;
	ss << info;
	msg.data = ss.str();
}

imav::GameMode current_gamemode;
void get_gamemode(const imav::GameMode::ConstPtr &msg)
{
	current_gamemode = *msg;
}

cv::VideoWriter videowriter;

void getInfoCallback(const std_msgs::String::ConstPtr &msg)
{
	const char *data = msg->data.c_str();
	if (strcmp(data, "quit") == 0)
	{
		b_FrameFinished = true;
		cout << "Real frame count: " << c << endl;
		destroyWindow(winName);
		ros::shutdown();
	}
	else if (strcmp(data, "ok") == 0)
	{
		b_FrameFinished = false;
	}
	else if (strcmp(data, "track") == 0)
	{
		b_Tracking = true;
		//b_Marking=false;
		;
	}
	else if (strcmp(data, "marking") == 0)
	{
		b_Tracking = false;
		b_Marking = true;
	}

	//this "search" command informs this node that the next target is required
	//It can be sent by the uav controlling node
	else if (strcmp(data, "search") == 0)
	{
		b_Tracking = false;
		b_Marking = false;
	}
}

cv::Mat current_image;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

  public:
	ImageConverter()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("imav/video_capture", 1,
								   &ImageConverter::imageCb, this);
		//image_pub_ = it_.advertise("/image_converter/output_video", 1);
	}

	~ImageConverter()
	{
		//cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv_ptr->image.copyTo(current_image);
	}
};

int main(int argc, char **argv)
{
	calcFeatLens();
	get_logistic(log_hist, "hist");
	get_logistic(log_hog, "hog");

	ROS_INFO("classifier loaded");

	ros::init(argc, argv, "PotDetect");
	ros::NodeHandle n_handle;

	ImageConverter image_converter;

	ros::Subscriber inform_sub = n_handle.subscribe("inform", 500, getInfoCallback);
	feedback_pub = n_handle.advertise<std_msgs::String>("feedback_info", 500);
	ctrl_pub = n_handle.advertise<std_msgs::String>("ctrl_info", 100);

	barrelList_pub = n_handle.advertise<imav::BarrelList>("imav/PotDetect_Barrel", 1);
	ros::Subscriber gamemode_sub = n_handle.subscribe<imav::GameMode>("imav/gamemode", 1, get_gamemode);

	std::string my_home_path = expand_user("~") + "/";
	videowriter.open(my_home_path + "workspace/video/1000.avi", CV_FOURCC('M', 'P', '4', '2'), 60, Size(640, 480));

	ros::Rate loop_rate(1000);
	ros::Rate loop_rate1(1);
	for (size_t i = 0; i < 3000; i++)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Time last_time = ros::Time::now();

	while (ros::ok())
	{
		//last_time = ros::Time::now();

		if (!current_image.data or current_gamemode.gamemode != imav::GameMode::GAMEMODE_CHECK_BARREL)
		{
			ros::spinOnce();
			loop_rate1.sleep();
			ROS_INFO("CurGameMode: %d, PotDetect wait gamede GAMEMODE_CHECK_BARREL ...", current_gamemode.gamemode);
			continue;
		}

		b_NewFrame = true;

		Mat dummy;
		current_image.copyTo(dummy);

		std_msgs::String feedback_msg;
		if (first_frame)
		{
			first_frame = 0;
			center_x = current_image.cols / 2;
			center_y = current_image.rows / 2;
			filtered_frame.create(dummy.size(), CV_8UC1);
		}
		if (start)
		{
			if (filter_len > 0)
			{
				detect_bbox(lastFrame, targets, false);
				//if(valid_targets.size()>0){	draw_bbox(dummy, valid_targets, 1);	}
			}
			else
			{
				filter_len = FILTER_LEN + 1;
				filtered_frame.setTo(0);
				detect_bbox(current_image, targets, true);
				valid_targets = targets;
				draw_bbox(dummy, targets, 2);
			}
		}
		if (start and filter_len == 0)
		{
			filter_len = FILTER_LEN + 1;
			int max_dist = 0;
			int min_dist = 1e4;
			stringstream ctrl_ss;

			if (b_Tracking)
			{
				std_msgs::String ctrl_msg;
				if (targets.size() == 0)
				{
					string2msg("no-target", ctrl_msg);
					ctrl_pub.publish(ctrl_msg);
				}
				else
				{
					int dx, dy, dist;
					for (size_t i = 0; i < targets.size(); ++i)
					{
						Target &t = targets[i];
						int cur_cx = t.location.x + (t.location.width >> 1);
						int cur_cy = t.location.y + (t.location.height >> 1);
						dist = abs(cur_cx - pTrackPos.x) + abs(cur_cy - pTrackPos.y);
						if (dist < min_dist)
						{
							dx = center_x - cur_cx;
							dy = center_y - cur_cy;
							pTrackPos.x = cur_cx;
							pTrackPos.y = cur_cy;
						}
					}
					cout << dx << "," << dy << endl;
					ctrl_ss << dx << "," << dy << "," << dist;
					string2msg(ctrl_ss.str().c_str(), ctrl_msg);
					ctrl_pub.publish(ctrl_msg);
				}
			}
			else if (not b_Marking)
			{
				for (size_t i = 0; i < targets.size(); ++i)
				{
					Target &t = targets[i];
					int dist = abs(t.location.x - center_x) + abs(t.location.y - center_y);
					if (dist > max_dist)
					{
						max_dist = dist;
						pTrackPos.x = t.location.x + (t.location.width >> 1);
						pTrackPos.y = t.location.y + (t.location.height >> 1);
					}
				}
				//new target found
				string2msg("found", feedback_msg);
				feedback_pub.publish(feedback_msg);
			}
		}

		string2msg("ok", feedback_msg);
		feedback_pub.publish(feedback_msg);

		//imshow(winName, dummy);

		imav::BarrelList barrelList;
		Targets2Barrels(targets, barrelList);
		barrelList_pub.publish(barrelList);
		videowriter << dummy;

		current_image.copyTo(lastFrame);
		if (start)
		{
			--filter_len;
		}
		if (start == 0)
		{
			++start;
		}
		++c;

		//ROS_INFO("Time: %f ms", (ros::Time::now() - last_time).toSec() * 1000);

		ros::spinOnce();
		//loop_rate.sleep();
	}

	ros::spin();

	return 0;
}
