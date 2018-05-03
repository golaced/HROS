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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <imav/Barrel.h>
#include <imav/BarrelList.h>
#include <imav/GameMode.h>
#include <imav/imavFunctions.h>

#define FILTER_LEN 2 * LIFE
using namespace std;
using namespace cv;

Mat lastFrame;
Mat filtered_frame;
MySVM mSVM;
vector<Target> targets;
vector<Target> valid_targets;
string winName = "PotDetect";
Point pTrackPos;

double max_area;
double min_area;

char first_frame = 1;
int center_x = 0, center_y = 0;
int filter_len = FILTER_LEN;
int ksize = 3;
int start = 0;
long c = 0;

bool bTracking;

cv::VideoWriter videowriter;

void string2msg(const char *info, std_msgs::String &msg)
{
	stringstream ss;
	ss << info;
	msg.data = ss.str();
}

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

imav::GameMode current_gamemode;

void get_gamemode(const imav::GameMode::ConstPtr &msg)
{
	current_gamemode = *msg;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "PotDetect");

	ros::NodeHandle nh;

	ImageConverter image_converter;

	ros::Publisher barrelList_pub = nh.advertise<imav::BarrelList>("imav/PotDetect_Barrel", 1);
	ros::Subscriber gamemode_sub = nh.subscribe<imav::GameMode>("imav/gamemode", 1, get_gamemode);

	bTracking = false;
	pTrackPos.x = center_x;
	pTrackPos.y = center_y;
	mSVM = get_svm();
	std::string my_home_path = expand_user("~") + "/";
	videowriter.open(my_home_path + "workspace/video/1000.avi", CV_FOURCC('M', 'P', '4', '2'), 30, Size(640, 480));
	ros::Rate loop_rate(100);
	ros::Rate loop_rate1(1);
	while (ros::ok())
	{
		if (!current_image.data or current_gamemode.gamemode != imav::GameMode::GAMEMODE_CHECK_BARREL)
		{
			ros::spinOnce();
			loop_rate1.sleep();
			ROS_INFO("CurGameMode: %d, PotDetect wait gamede GAMEMODE_CHECK_BARREL ...", current_gamemode.gamemode);
			continue;
		}

		blur(current_image, current_image, Size(ksize, ksize));
		Mat dummy;
		current_image.copyTo(dummy);

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
				detect_bbox(lastFrame, targets, mSVM, false);
				if (valid_targets.size() > 0)
				{
					draw_bbox(dummy, valid_targets, 1);
				}
			}
			else
			{
				filtered_frame.setTo(0);
				detect_bbox(current_image, targets, mSVM, true);
				valid_targets = targets;
				draw_bbox(dummy, targets, 1);
			}
		}

		///////////////////////////////////////////
		//dispTargets(targets);
		imav::BarrelList barrelList;
		Targets2Barrels(targets, barrelList);
		barrelList_pub.publish(barrelList);
		videowriter << dummy;
		//////////////////////////////////////////

		//imshow(winName, dummy);
		//waitKey(1);

		if (start and filter_len == 0)
		{
			filter_len = FILTER_LEN + 1;
			int max_dist = 0;
			int min_dist = 1e4;
			stringstream ctrl_ss;
			if (bTracking)
			{
				int dx, dy, dist;
				for (unsigned int i = 0; i < targets.size(); ++i)
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
			}
			else
			{
				for (unsigned int i = 0; i < targets.size(); ++i)
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
			}
		}
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

		ros::spinOnce();
		//loop_rate.sleep();
	}

	return 0;
}