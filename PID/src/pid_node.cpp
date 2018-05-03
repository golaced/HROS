#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandHome.h>
#include <ros/param.h>
#include <iostream>

#include "PID/PID.h"

using namespace std;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void get_local_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
    // ROS_INFO("ori:%f %f %f %f", current_pose.pose.orientation.w, current_pose.pose.orientation.x, 
    //         current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    //ROS_INFO("pos: %f %f %f", current_pose.pose.position.x, current_pose.pose.position.y, 
    //        current_pose.pose.position.z);        
}

geometry_msgs::PoseStamped currentTarget_pose;
void get_target_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    currentTarget_pose = *msg;
    //ROS_INFO("target:[%.1f %.1f]", currentTarget_pose.pose.position.x, currentTarget_pose.pose.position.y);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_node");
    ros::NodeHandle nh;

    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("imav/pid_velocity", 1);

    ros::Subscriber local_pos_sub = nh.subscribe("mavros/local_position/pose", 10, get_local_pose);
    ros::Subscriber target_pos_sub = nh.subscribe("imav/target_position", 10, get_target_pose);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    geometry_msgs::TwistStamped velocity_tw;

    double Setpoint_x, Input_x, Output_x;
    double Setpoint_y, Input_y, Output_y;
    double Setpoint_z, Input_z, Output_z;
    Setpoint_x = 320;
    Setpoint_y = 240;
    Setpoint_z = 2;

    PIDParam pidparam_x,pidparam_y,pidparam_z;
    pidparam_x.Kp = 0.002;
    pidparam_y.Kp = 0.002;
    pidparam_z.Kp = 0.8;

    PID pid_x(&Input_x, &Output_x, &Setpoint_x, pidparam_x, REVERSE);
    PID pid_y(&Input_y, &Output_y, &Setpoint_y, pidparam_y);
    PID pid_z(&Input_z, &Output_z, &Setpoint_z, pidparam_z);

    double velocity_z_notarget = 0.0;
    double velocity_z_hastarget = 0.0;
    ros::param::get("~velocity_z_hastarget", velocity_z_hastarget);
    ros::param::get("~velocity_z_notarget", velocity_z_notarget);
	ros::param::get("~pidparam_x_Kp", pidparam_x.Kp);
	ros::param::get("~pidparam_y_Kp", pidparam_y.Kp);
    
    ROS_INFO("velocity_z_hastarget: %fm/s", velocity_z_hastarget);
    ROS_INFO("velocity_z_notarget: %fm/s", velocity_z_notarget);
    
    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (currentTarget_pose.pose.position.x>0&&currentTarget_pose.pose.position.y>0)
        {
            Input_z = current_pose.pose.position.z;
            pid_z.Compute();
            Input_x = currentTarget_pose.pose.position.x;
            pid_x.Compute();
            Input_y = currentTarget_pose.pose.position.y;
            pid_y.Compute();
            velocity_tw.twist.linear.z=velocity_z_hastarget;
            velocity_tw.twist.linear.x=Output_x;
            velocity_tw.twist.linear.y=Output_y;
        }
        else
        {
            velocity_tw.twist.linear.z=velocity_z_notarget;
            velocity_tw.twist.linear.x=0.0;
            velocity_tw.twist.linear.y=0.0;
        }
        //ROS_INFO("target:[%f %f]", velocity_tw.twist.linear.x, velocity_tw.twist.linear.y);

        velocity_pub.publish(velocity_tw);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
