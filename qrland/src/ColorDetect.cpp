#include "ColorDetect.h"

using namespace std;
using namespace cv;

ColorDetect::~ColorDetect()
{

}

void ColorDetect::setImage(const cv::Mat img)
{
	img.copyTo(srcImage);

	img_center = Point(srcImage.cols / 2, srcImage.rows / 2);

	maxSize = 0.9 * (std::max)(srcImage.cols, srcImage.rows) * 4;
	minSize = (std::min)(5.0, 0.1 * (std::max)(srcImage.cols, srcImage.rows) * 4);
}

void ColorDetect::getColorRegion(COLOR_TYPE colortype)
{

	//TickMeter tm1;
	
	//tm1.reset();
	//tm1.start();

	vector<Mat> hsvSplit;
	cvtColor(srcImage, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);

	switch (colortype)
	{
		case ColorDetect::RED:
		{
			iLowH = 0;
			iHighH = 10;
			inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
			Mat imgThresholded1;
			iLowH1 = 165;
			iHighH1 = 181;
			inRange(imgHSV, Scalar(iLowH1, iLowS, iLowV), Scalar(iHighH1, iHighS, iHighV), imgThresholded1); //Threshold the image
			cv::bitwise_or(imgThresholded, imgThresholded1, imgThresholded);
			break;
		}
			
		case ColorDetect::GREEN:
		{
			iLowH = 105;
			iHighH = 135;
			inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
			break;
		}
		case ColorDetect::BLUE:
		{
			iLowH = 45;
			iHighH = 75;
			inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
			break;
		}
		default:
			break;
	}
	
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Mat threshold_output;
	imgThresholded.copyTo(threshold_output);

	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE,
				 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<vector<Point> > contours_poly(contours.size());
	objectRects.clear();
	objectCenters.clear();

	for (unsigned int i = 0; i < contours.size(); i++)
	{
		// check it is a possible element by first checking is has enough points
		if (minSize < contours[i].size() && contours[i].size() < maxSize)
		{
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
			cv::Rect rect_temp = boundingRect(Mat(contours_poly[i]));
			objectRects.push_back(rect_temp);
			objectCenters.push_back(rect_temp.tl() + Point(rect_temp.width / 2, rect_temp.height / 2));
		}
	}


	/*std::sort(objectRects.begin(), objectRects.end(), [&](Rect r1, Rect r2)
	{	return r1.area() > r2.area(); });*/


//	for (size_t i = 0; i < objectRects.size(); i++)
//	{
//		cv::rectangle(srcImage, objectRects[i], Scalar(255, 0, 0), 3);
//		cv::circle(srcImage, objectCenters[i], 3, Scalar(255, 0, 0), -1);
//	}

    //tm1.stop();
    //cout << "detect:" << tm1.getTimeMilli() << "ms" << endl;
    //imshow("Thresholded Image", imgThresholded); //show the thresholded image
    //cv::circle(srcImage,img_center, 3, Scalar(255, 255, 0), -1);
    //imshow("Original", srcImage); //show the original image

//	static Scalar imgThresholded_sum;

//	imgThresholded_sum = sum(imgThresholded);

//	if (imgThresholded_sum.val[0] > 255 * 10000)
//	{
//		cout << endl;
//		cout << "*************" << endl;
//		cout << "object occer;" << endl;
//		cout << "*************" << endl;
//		cout << endl;
//	}
	
}

std::vector<int> ColorDetect::getPointLocate(std::vector<cv::Point> points)
{
	std::vector<int> locate;
	Point p(0, 0);
	for (size_t i = 0; i < points.size(); i++)
	{
		p = points[i] - img_center;
		if (p.x>0)
		{
			if (p.y>0)
			{
				locate.push_back(COOR_LOCATE::BOTTOMRIGHT);
			}
			else
			{
				locate.push_back(COOR_LOCATE::TOPRIGHT);
			}
		}
		else
		{
			if (p.y > 0)
			{
				locate.push_back(COOR_LOCATE::BOTTOMLEFT);
			}
			else
			{
				locate.push_back(COOR_LOCATE::TOPLEFT);
			}
		}
	}

	return locate;
}

std::vector<int> ColorDetect::getColorRegionLocate(COLOR_TYPE colortype)
{
	getColorRegion(colortype);
    return getPointLocate(objectCenters);
}

std::vector<Rect> ColorDetect::getRects()
{
    return this->objectRects;
}

std::vector<Point> ColorDetect::getRectCenters()
{
    return this->objectCenters;
}

void ColorDetect::setHSVScope()
{
	iLowH = 0;
	iHighH = 180;
	iLowH1 = 0;
	iHighH1 = 180;

	iLowS = 50;
	iHighS = 255;
	iLowV = 70;
	iHighV = 255;
}

ColorDetect::ColorDetect()
{
	setHSVScope();
}

ColorDetect::ColorDetect(const cv::Mat img)
{
	setImage(img);

	setHSVScope();
}
