/*
 * imageTrans.cpp
 *
 *  Created on: Sep 14, 2016
 *      Author: odroid
 */

#include "imav/imageTrans.h"
#include <string>

#include <iomanip>

using namespace std;
using namespace cv;

void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle)
{
	CV_Assert(!inputIm.empty());
	Mat inputImg;
	inputIm.copyTo(inputImg);
	float radian = (float)(angle / 180.0 * CV_PI);
	int uniSize = (int)(max(inputImg.cols, inputImg.rows) * 1.414);
	int dx = (int)(uniSize - inputImg.cols) / 2;
	int dy = (int)(uniSize - inputImg.rows) / 2;
	copyMakeBorder(inputImg, tempImg, dy, dy, dx, dx, BORDER_CONSTANT);
	Point2f center((float)(tempImg.cols / 2), (float)(tempImg.rows / 2));
	Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);
	warpAffine(tempImg, tempImg, affine_matrix, tempImg.size());
	float sinVal = fabs(sin(radian));
	float cosVal = fabs(cos(radian));
	Size targetSize((int)(inputImg.cols * cosVal + inputImg.rows * sinVal),
					(int)(inputImg.cols * sinVal + inputImg.rows * cosVal));
	int x = (tempImg.cols - targetSize.width) / 2;
	int y = (tempImg.rows - targetSize.height) / 2;
	Rect rect(x, y, targetSize.width, targetSize.height);
	tempImg = Mat(tempImg, rect);
}

int video_num = 0;
//std::ifstream video_num_read;
std::ofstream video_num_write;
#ifdef WIN32
std::string video_num_path(
	"D:/IMAV/VideoLocation/video_num.txt");
std::string writer_path("D:/IMAV/VideoLocation/");
#else
std::string video_num_path;
std::string writer_path;

#endif // WIN32

std::string home_path("~");

std::string expand_user(std::string path)
{
	if (not path.empty() and path[0] == '~')
	{
		assert(path.size() == 1 or path[1] == '/'); // or other error handling
		char const *home = getenv("HOME");
		if (home or ((home = getenv("USERPROFILE"))))
		{
			path.replace(0, 1, home);
		}
		else
		{
			char const *hdrive = getenv("HOMEDRIVE"),
					   *hpath = getenv("HOMEPATH");
			assert(hdrive); // or other error handling
			assert(hpath);
			path.replace(0, 1, std::string(hdrive) + hpath);
		}
	}
	return path;
}

void initPath()
{
#ifdef WIN32

#else
	home_path = expand_user(home_path);
	home_path += "/";
	cout << "home_path:" << home_path << endl;

	video_num_path = home_path;
	writer_path = home_path;
	video_num_path += "workspace/video/video_num.txt";
	writer_path += "workspace/video/";
#endif // WIN32

	cout << "video_num_path:" << video_num_path << endl;
	cout << "video writer_path:" << writer_path << endl;
}

void startWriteVideo(std::ifstream &video_num_read,
					 cv::VideoWriter &video_writer)
{
	video_num_read.open(video_num_path.c_str());
	video_num_read >> video_num;
	video_num_read.close();

	cout << video_num << endl;

	video_num_write.open(video_num_path.c_str());
	video_num_write << (video_num + 1);
	video_num_write.close();

	if (video_writer.isOpened())
	{
		video_writer.release();
	}

	std::stringstream ss;
	string video_name;

	ss << video_num;
	ss >> video_name;
	video_name += ".avi";

	video_writer.open(
		writer_path + video_name,
		CV_FOURCC('M', 'P', '4', '2'), 30, Size(640, 480));
}

void startWriteFile(std::ofstream &gps_writer)
{
	std::stringstream ss;
	string file_name;

	ss << video_num;
	ss >> file_name;
	file_name += ".txt";
	gps_writer.open((writer_path + file_name).c_str());
	gps_writer << setprecision(15);
	//gps_writer << "latitude    longitude    height" << endl;
}
