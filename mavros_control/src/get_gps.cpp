#include <ros/ros.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>

#include <iostream>

using namespace std;

sensor_msgs::NavSatFix current_gps;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_gps = *msg;
    ROS_INFO("%f %f %f",current_gps.longitude,current_gps.latitude,current_gps.altitude);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "get_gps_node");
    
    ros::NodeHandle nh;
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 1, get_gps_cb);
    
    ros::Rate loop_rate(5);
    
    while (ros::ok)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}
