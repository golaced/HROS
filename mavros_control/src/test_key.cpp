#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist g_tw;
void get_key(const geometry_msgs::Twist::ConstPtr &msg)
{
    g_tw = *msg;
    ROS_INFO("%f %f %f",g_tw.linear.x,g_tw.linear.y,g_tw.linear.z);
    ROS_INFO("%f %f %f",g_tw.angular.x,g_tw.angular.y,g_tw.angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_key");
    ros::NodeHandle nh;
    ros::Subscriber key_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel",30,get_key);

    ros::spin();

    return 0;

}
