#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <imav/GameMode.h>
#include "imav/imageTrans.h"
#include "imav/GPSDataStruct.h"

using namespace std;
using namespace cv;

std::ifstream video_num_read;

sensor_msgs::NavSatFix current_gps;
GPSDataStruct gps_g;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    current_gps = *msg;
    gps_g.longitude = current_gps.longitude;
    gps_g.latitude = current_gps.latitude;
    //ROS_INFO("%f %f %f",current_gps.longitude,current_gps.latitude,current_gps.altitude);
}
geometry_msgs::PoseStamped current_pose;
void get_local_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    gps_g.altitude = current_pose.pose.position.z;
}

imav::GameMode current_gamemode;

void get_gamemode(const imav::GameMode::ConstPtr &msg)
{
    current_gamemode = *msg;
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

int main(int argc, char *argv[])
{
    initPath();
    ros::init(argc, argv, "video_location");

    cv::VideoWriter video_writer;
    std::ofstream file_writer;
    startWriteVideo(video_num_read, video_writer);
    startWriteFile(file_writer);

    ros::NodeHandle nh;
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, get_gps_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, get_local_pose);
    ros::Subscriber gamemode_sub = nh.subscribe<imav::GameMode>("imav/gamemode", 1, get_gamemode);

    ImageConverter image_converter;

    ros::Rate loop_rate(20);
    ros::Rate loop_rate1(1);

    for (size_t i = 0; i < 30; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok())
    {
        if (current_gamemode.gamemode != imav::GameMode::GAMEMODE_SURVEY)
        {
            ros::spinOnce();
            loop_rate1.sleep();
            ROS_INFO("CurGameMode: %d, landtargetposition wait gamemode GAMEMODE_SURVEY ...", current_gamemode.gamemode);
            continue;
        }
        ros::spinOnce();
        loop_rate.sleep();
        //cv::imshow("capture",image_src);
        video_writer << current_image;
        file_writer << gps_g << std::endl;
        cv::waitKey(1);
    }

    video_writer.release();
    ROS_INFO("%s", "video_writer.release();");
    file_writer.close();
    ROS_INFO("%s", "file_writer.close();");

    return 0;
}