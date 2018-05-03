#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera");
    cv::VideoCapture capture(0);
    cv::Mat image_src;

    if (!capture.isOpened())
    {
        std::cout<<"Capture open failed!"<<std::endl;
        return -1;
    }
    cv::VideoWriter videowriter;
    videowriter.open("/home/odroid/workspace/video/3000.avi", CV_FOURCC('M', 'P', '4', '2'), 60, cv::Size(640, 480));
    
    capture>>image_src;

    while (true)
    {
        capture>>image_src;
        cv::imshow("capture",image_src);
		videowriter<<image_src;
        if (cv::waitKey(15)==27)
        {
            break;
        }
    }
    return 0;
}