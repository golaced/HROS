#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <sensor_msgs/image_encodings.h>
#include <imav/GameMode.h>
#include "landing.h"
#include <imav/imavFunctions.h>

using namespace std;
using namespace cv;

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
    ros::init(argc, argv, "land_target_position_H");
    ros::NodeHandle nh;
    ros::Publisher target_position_pub = nh.advertise<geometry_msgs::PoseStamped>("imav/target_position", 1);

    ros::Subscriber gamemode_sub = nh.subscribe<imav::GameMode>("imav/gamemode", 1, get_gamemode);

    ImageConverter image_converter;

    ros::Rate loop_rate(1000);
    ros::Rate loop_rate1(1);
    for (size_t i = 0; i < 3000; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::string my_home_path = expand_user("~");
    my_home_path += "/";
    int ksize = 1;
    std::string tmpl_path(my_home_path + "catkin_ws/src/land/config/templates.txt");
    Mat _tmpl = loadTemplate(tmpl_path);
    cv::VideoWriter videowriter;
    videowriter.open(my_home_path + "workspace/video/2000.avi", CV_FOURCC('M', 'P', '4', '2'), 30, Size(640, 480));

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        geometry_msgs::PoseStamped current_target_position;
        
        if (current_gamemode.gamemode != imav::GameMode::GAMEMODE_CHECK_H)
        {
            //cv::destroyAllWindows();
            ros::spinOnce();
            loop_rate1.sleep();
            ROS_INFO("CurGameMode: %d, land_target_position_H wait gamemode GAMEMODE_CHECK_H ...", current_gamemode.gamemode);
            continue;
        }

        //imshow("image", current_image);

        blur(current_image, current_image, Size(ksize, ksize));
        Mat rgbImage = alignImage(current_image, MAX_SIZE);
        Mat _s = getHSVImage(rgbImage, "s");
        Mat _v = getHSVImage(rgbImage, "v");
        Mat sv[2];
        Mat bi[2];
        sv[0] = _s;
        sv[1] = _v;
        // imshow("hsv", _s);
        for (int i = 0; i < 2; ++i)
        {
            Mat bg(sv[i].size(), CV_8UC1);
            bg.setTo(255);
            threshold(sv[i], bi[i], 250, 255, THRESH_OTSU);
            subtract(bg, bi[i], bi[i]);

            vector<Rect> contours;

            getContours(bi[i], contours);

            int H_index = -1;
            cv::Rect target_H;

            if (contours.size() > 0)
            {
                H_index = KNN(bi[i], _tmpl, contours);

                if (H_index >= 0)
                {
                    target_H = contours[H_index];
                    //rectangle(rgbImage, contours[H_index], Scalar(255,0,0),2);
                    if (target_H.area() > 400 && (float)target_H.width / target_H.height > 0.7 && (float)target_H.width / target_H.height < 1.45)
                    {
                        current_target_position.pose.position.x = target_H.x + target_H.width / 2;
                        current_target_position.pose.position.y = target_H.y + target_H.height / 2;
                        rectangle(rgbImage, contours[H_index], Scalar(0, 0, 255), 2);
                    }
                }
            }
        }

        target_position_pub.publish(current_target_position);

        videowriter << rgbImage;

        if (ros::Time::now() - last_request > ros::Duration(5.0))
        {
            ROS_INFO("land_target_position_H Working ...");
            last_request = ros::Time::now();
        }

        ros::spinOnce();
        //loop_rate.sleep();
    }

    return 0;
}
