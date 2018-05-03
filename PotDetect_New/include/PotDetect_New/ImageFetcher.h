#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Mat read_frame(VideoCapture& vcap);

void read_config(string cfg_name, vector<string>& names);

void string2msg(const char* info, std_msgs::String& msg);
