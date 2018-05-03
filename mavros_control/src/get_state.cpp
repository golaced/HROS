/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandHome.h>
#include <iostream>

#if ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, 1, 12, 7)
#include <sensor_msgs/BatteryState.h>
#else
#include <mavros_msgs/BatteryStatus.h>
#endif

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void get_local_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    // ROS_INFO("ori:%f %f %f %f", current_pose.pose.orientation.w, current_pose.pose.orientation.x,
    //         current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    //ROS_INFO("pos: %f %f %f", current_pose.pose.position.x, current_pose.pose.position.y,
    //        current_pose.pose.position.z);
}

double current_battery_percentage = 1.0;

#if ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, 1, 12, 7)
sensor_msgs::BatteryState current_battery;
void get_battery(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    current_battery = *msg;
    current_battery_percentage = current_battery.percentage;
    ROS_INFO("Battery: %f", current_battery_percentage);
}
#else
mavros_msgs::BatteryStatus current_battery;
void get_battery(const mavros_msgs::BatteryStatus::ConstPtr &msg)
{
    current_battery = *msg;
    current_battery_percentage = current_battery.remaining;
    ROS_INFO("Battery: %f", current_battery_percentage);
}
#endif

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_state_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");

    ros::Subscriber local_pos_sub = nh.subscribe("mavros/local_position/pose", 10, get_local_pose);
    ros::Subscriber battery_sub = nh.subscribe("mavros/battery", 1, get_battery);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for (int i = 20; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        ROS_INFO("PX4 Mode: %s", current_state.mode.c_str());

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
