/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/CommandCode.h>
#include <iostream>
#include "imav/transformWaypoints.h"

using namespace std;

#define CHANNEL_INIT -1
#define LOW 0
#define MID 1
#define HIGH 2

mavros_msgs::WaypointList record_points;

double wp_height = 3.0;
double mission_speed = 1.0;

sensor_msgs::NavSatFix current_gps;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    current_gps = *msg;
    //ROS_INFO("%f %f %f",current_gps.longitude,current_gps.latitude,current_gps.altitude);
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

mavros_msgs::RCIn rcIn;
mavros_msgs::RCOut rcOut;

void get_rc_out(const mavros_msgs::RCOut::ConstPtr &msg)
{
    rcOut = *msg;
}
bool recorded = true;
int channel6_laststatus = CHANNEL_INIT;
int channel6_currentstatus = CHANNEL_INIT;
int channel7_laststatus = CHANNEL_INIT;
int channel7_currentstatus = CHANNEL_INIT;
mavros_msgs::Waypoint point;
void checkChannel7()
{

    if (rcIn.channels[6] > 1500)
    {
        channel7_currentstatus = HIGH;
    }
    else
    {
        channel7_currentstatus = LOW;
    }
    if (channel7_currentstatus == HIGH)
    {
        point.y_long = current_gps.longitude;
        point.x_lat = current_gps.latitude;
    }
    if (channel7_laststatus == HIGH && channel7_currentstatus == LOW)
    {
        record_points.waypoints.push_back(point);
        ROS_INFO("Record Point:[%f %f]", point.y_long, point.x_lat);
    }
    channel7_laststatus = channel7_currentstatus;
}

void checkChannel6()
{
    if (rcIn.channels[5] > 1350 && rcIn.channels[5] < 1650)
    {
        channel6_currentstatus = MID;
    }
    if (rcIn.channels[5] > 1650)
    {
        channel6_currentstatus = HIGH;
    }
    if (rcIn.channels[5] < 1350)
    {
        channel6_currentstatus = LOW;
    }
    if (channel6_laststatus == MID && channel6_currentstatus == HIGH)
    {
        mavros_msgs::WaypointList wp_list = transformWaypoints(record_points, mission_speed, wp_height);
        pushWaypointList(wp_list);
    }
    else if (channel6_laststatus == MID && channel6_currentstatus == LOW)
    {
        record_points.waypoints.clear();
        clearWaypointList();
    }
    channel6_laststatus = channel6_currentstatus;
}

void get_rc_in(const mavros_msgs::RCIn::ConstPtr &msg)
{
    rcIn = *msg;

    checkChannel7(); // record waypoints

    checkChannel6(); // push/clear waypoints
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recordpoints_node");
    ros::NodeHandle nh;

    ros::param::get("~wp_height", wp_height);
    ros::param::get("~mission_speed", mission_speed);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber rc_out_sub = nh.subscribe<mavros_msgs::RCOut>("mavros/rc/out", 1, get_rc_out);
    ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 1, get_rc_in);
    ros::ServiceClient waypoint_setcurrent_client = nh.serviceClient<mavros_msgs::WaypointSetCurrent>("mavros/mission/set_current");
    ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>("mavros/mission/pull");
    ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    ros::ServiceClient waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, get_gps_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}