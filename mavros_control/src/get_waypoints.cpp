#include <ros/ros.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <iostream>

using namespace std;

void print(const mavros_msgs::Waypoint& wp)
{
    ROS_INFO("gps:%f %f %f", wp.x_lat,wp.y_long,wp.z_alt);
    ROS_INFO("%d %d %d %d",wp.command,wp.frame,wp.autocontinue,wp.is_current);
    ROS_INFO("param:%f %f %f %f", wp.param1,wp.param2,wp.param3,wp.param4);
}

mavros_msgs::WaypointList current_waypoints;
void get_waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    current_waypoints = *msg;

    for (size_t i = 0; i < current_waypoints.waypoints.size(); i++)
    {
        ROS_INFO("WP %d",int(i));
        print(current_waypoints.waypoints[i]);
    }
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "get_waypoints_node");
    
    ros::NodeHandle nh;
    ros::Subscriber get_waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 5, get_waypoints_cb);
    
    ros::Rate loop_rate(10);
    ros::spin();

    return 0;
}
