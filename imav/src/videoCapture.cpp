#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <imav/imageTrans.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "video_capture");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("imav/video_capture", 1);

    cv::VideoCapture cap(0);

    if (!cap.isOpened())
    {
        ROS_INFO("open image device failed\n");
        return 1;
    }

    ROS_INFO("Video Capture Open Success ...");

    cv::Mat src_image, frame;
    sensor_msgs::ImagePtr msg;

    cap >> src_image;
    if (src_image.data)
    {
        ROS_INFO("Video Capture Read Success ...");
    }
    else
    {
        ROS_ERROR("Video Capture Read Failed ...");
    }

    ros::Rate loop_rate(60);
    ros::Time last_request = ros::Time::now();

    ROS_INFO("Video Capture Working ...");
    while (ros::ok())
    {
        cap >> src_image;
        if (!src_image.data)
        {
            ROS_INFO("src_image empty ...");
        }
        warpFfine(src_image, frame, 180);

        if (!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            image_pub.publish(msg);
        }
        else
        {
            ROS_INFO("frame empty ...");
        }

        if (ros::Time::now() - last_request > ros::Duration(5.0))
        {
            ROS_INFO("Video Capture Working ...");
            last_request = ros::Time::now();
        }

        //ros::spinOnce();
        //loop_rate.sleep();
    }

    return 0;
}
