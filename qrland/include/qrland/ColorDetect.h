#ifndef __COLORDETECT_H__
#define __COLORDETECT_H__

#include <opencv2/opencv.hpp>
#include <iostream>

class ColorDetect
{
public:
	enum COLOR_TYPE
	{
		RED,
		GREEN,
		BLUE
	};

	enum COOR_LOCATE
	{
		TOPRIGHT = 1,
		TOPLEFT,
		BOTTOMLEFT,
		BOTTOMRIGHT
	};
	
	ColorDetect();
	ColorDetect(const cv::Mat img);
	~ColorDetect();

	void setImage(const cv::Mat img);
	void getColorRegion(COLOR_TYPE colortype);
	std::vector<int> getPointLocate(std::vector<cv::Point> points);
	std::vector<int> getColorRegionLocate(COLOR_TYPE colortype);
    std::vector<cv::Rect> getRects();
    std::vector<cv::Point> getRectCenters();

private:

	void setHSVScope();

	cv::Mat srcImage;
	cv::Mat dstImage;
	cv::Mat imgHSV;
	cv::Mat imgThresholded;
	cv::Point img_center;

	std::vector<cv::Rect> objectRects;
	std::vector<cv::Point> objectCenters;

	int iLowH;
	int iHighH;
	int iLowH1;
	int iHighH1;

	int iLowS;
	int iHighS;

	int iLowV;
	int iHighV;

	int maxSize;
	int minSize;

};

#endif // __COLORDETECT_H__
