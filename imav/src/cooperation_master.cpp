#include <ros/ros.h>
#include <ros/param.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <sstream>
#include <stdlib.h>
#include <cmath>
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
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <imav/imavFunctions.h>
using namespace std;

std_msgs::UInt32 current_distance;
void get_uwb_disrance_cb(const std_msgs::UInt32::ConstPtr &msg) //????
{
    current_distance = *msg;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void get_local_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

sensor_msgs::Imu current_imu;
double current_yaw;
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    current_imu = *msg;
    double q0,q1,q2,q3;
    q0 = current_imu.orientation.w;
    q1 = current_imu.orientation.x;
    q2 = current_imu.orientation.y;
    q3 = current_imu.orientation.z;

    double Rol = atan2(2 * (q0 * q1 + q2 * q3), -1 + 2 * (q1 * q1 + q2 * q2)) * 57.3f;
    double Pit = asin(2 * (-q1 * q3 + q0 * q2)) * 57.3f;
    double Yaw = atan2(2 * (-q1 * q2 - q0 * q3), 1 - 2 * (q0 * q0 + q1 * q1)) * 57.3f;
    current_yaw = 90-atan2(2. * (q0 * q3 + q1 * q2), 1. - 2. * (q2 * q2 + q3 * q3)) * 57.3f;
    if (current_yaw<0)
    {
        current_yaw += 360;
    }

    //ROS_INFO("%.2f %.2f %.2f %.2f", Pit, Rol, Yaw, current_yaw);
}

sensor_msgs::NavSatFix current_gps;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_gps = *msg;
    //ROS_INFO("%f %f %f",current_gps.longitude,current_gps.latitude,current_gps.altitude);
}

void print_wp(const mavros_msgs::Waypoint &wp)
{
    ROS_INFO("gps:%f %f %f", wp.x_lat, wp.y_long, wp.z_alt);
    ROS_INFO("%d %d %d %d", wp.command, wp.frame, wp.autocontinue, wp.is_current);
    ROS_INFO("param:%f %f %f %f", wp.param1, wp.param2, wp.param3, wp.param4);
}
mavros_msgs::WaypointList current_waypoints;
void get_waypoints(const mavros_msgs::WaypointList::ConstPtr &msg)
{
    current_waypoints = *msg;

    for (size_t i = 0; i < current_waypoints.waypoints.size(); i++)
    {
        ROS_INFO("WP %d",int(i));
        print_wp(current_waypoints.waypoints[i]);
    }
}

int med_filter_tmp_0[11];
int med_fil_cnt;
int Moving_Median(int width_num, int in)
{
    int tmp[11];
    int i, j;
    int t;

    if (width_num >= 10)
        return 0;
    else
    {
        if (++med_fil_cnt >= width_num)
            med_fil_cnt = 0;
        med_filter_tmp_0[med_fil_cnt] = in;
        for (i = 0; i < width_num; i++)
            tmp[i] = med_filter_tmp_0[i];
        // ROS_INFO_STREAM("tmp[0]= "<<tmp[0]);
        // ROS_INFO_STREAM("tmp[1]= "<<tmp[1]);
        // ROS_INFO_STREAM("tmp[2]= "<<tmp[2]);
        // ROS_INFO_STREAM("tmp[3]= "<<tmp[3]);
        //??????
        for (i = 0; i < width_num - 1; i++)
        {
            for (j = 0; j < (width_num - 1 - i); j++)
            {
                if (tmp[j] > tmp[j + 1])
                {
                    t = tmp[j];
                    tmp[j] = tmp[j + 1];
                    tmp[j + 1] = t;
                }
            }
        }
        return (tmp[(int)width_num / 2]); //??tmp[]???
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cooperation_master");
    ros::NodeHandle nh;
    ros::Subscriber get_uwb_distance_sub = nh.subscribe("uwb_distance", 1, get_uwb_disrance_cb);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10,get_local_pose);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, get_gps_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",2,imu_cb);
    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>("mavros/mission/waypoints", 10, get_waypoints);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    for (int i = 20; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        ROS_INFO("PX4 Mode: %s", current_state.mode.c_str());
        geometry_msgs::TwistStamped velocity_tw;
        velocity_pub.publish(velocity_tw);

        if (current_state.mode=="OFFBOARD")
        {
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    double lat_takeoff = current_gps.latitude;
    double long_takeoff = current_gps.longitude;
    double fly_dis = 0.0;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.LAND";

    float exp_high = 3; //????
    ros::param::get("~exp_high", exp_high);
    double land_dis = 50.0;
    ros::param::get("~land_dis", land_dis);
    double cooperation_master_speed = 0.0;
    ros::param::get("~cooperation_master_speed", cooperation_master_speed);    
    float high_err = 0;

    float kp_high = 0.2;

    float pid_out_high = 0;


    ros::Time last_request = ros::Time::now();
    bool breakmission_over = false;

    while (ros::ok())
    {
        if (!breakmission_over)
        {
            if (current_state.mode == "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                offb_set_mode.request.custom_mode = "AUTO.MISSION";
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
                {
                    ROS_INFO("MISSION enabled");
                    breakmission_over = true;
                }
            }
        }
        geometry_msgs::TwistStamped velocity_tw;
        velocity_pub.publish(velocity_tw);

        // high_err = exp_high - current_pose.pose.position.z;
        // pid_out_high = kp_high * high_err;

        // if (pid_out_high > 0.3 || pid_out_high < -0.3)
        // {
        //     pid_out_high = 0.3 * pid_out_high / abs(pid_out_high);
        // }

        // geometry_msgs::TwistStamped velocity_tw;
        // velocity_tw.twist.linear.z = pid_out_high;
        // velocity_tw.twist.linear.x = sin(current_yaw/57.3f) * cooperation_master_speed;
        // velocity_tw.twist.linear.y = cos(current_yaw/57.3f) * cooperation_master_speed;

        // ROS_INFO("velocity_tw:%f %f", velocity_tw.twist.linear.x, velocity_tw.twist.linear.y);

        // velocity_pub.publish(velocity_tw);

        fly_dis = GetDirectDistance(lat_takeoff, long_takeoff, current_gps.latitude, current_gps.longitude);
        ROS_INFO("fly_dis:%f", fly_dis);
        if (fly_dis>land_dis)
        {
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
    {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }

    ros::Rate rate1(1);
    while (ros::ok())
    {
	    ROS_INFO("------Game Over------");
        ROS_INFO("PX4 Mode: %s", current_state.mode.c_str());
        
        ros::spinOnce();
        rate1.sleep();
    }

    return 0;
}