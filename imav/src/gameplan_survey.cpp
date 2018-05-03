//手动起飞 手动记录航点
//切换offboard模式
//5秒后自动执行任务
//任务执行完成之后自动切换回offboard模式

#include <ros/ros.h>
#include <ros/param.h>
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
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <imav/GameMode.h>
#include <iostream>
using namespace std;
using namespace mavros_msgs;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void print_wp(const mavros_msgs::Waypoint& wp)
{
    ROS_INFO("gps:%f %f %f", wp.x_lat,wp.y_long,wp.z_alt);
    ROS_INFO("%d %d %d %d",wp.command,wp.frame,wp.autocontinue,wp.is_current);
    ROS_INFO("param:%f %f %f %f", wp.param1,wp.param2,wp.param3,wp.param4);
}
mavros_msgs::WaypointList current_waypoints;
void get_waypoints(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    current_waypoints = *msg;
    
    for (size_t i = 0; i < current_waypoints.waypoints.size(); i++)
    {
        ROS_INFO("WP %d",int(i));
        print_wp(current_waypoints.waypoints[i]);
    }
}

sensor_msgs::NavSatFix current_gps;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_gps = *msg;
    //ROS_INFO("%f %f %f",current_gps.longitude,current_gps.latitude,current_gps.altitude);
}

geometry_msgs::TwistStamped current_pid_velocity;
void get_current_pid_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_pid_velocity = *msg;
}

geometry_msgs::PoseStamped current_local_pose;
void get_local_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_local_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gameplan_surver");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, get_waypoints);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/mavros/global_position/global", 1, get_gps_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 1, get_local_pose);
    ros::Subscriber pid_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("imav/pid_velocity", 1, get_current_pid_velocity);
    ros::Publisher gamemode_pub = nh.advertise<imav::GameMode>("imav/gamemode", 1);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    double height_land = 1.0;
    ros::param::get("~height_land", height_land);
    if (height_land<0.5)
    {
		height_land = 0.5;
    }

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    imav::GameMode current_gamemode;
    current_gamemode.gamemode = imav::GameMode::GAMEMODE_OTHER;

    bool breakmission_over = false;
    ros::Time last_request = ros::Time::now();
    ros::Time last_show_time = ros::Time::now();

    while (ros::ok())
    {
        current_gamemode.gamemode = imav::GameMode::GAMEMODE_SURVEY;
        gamemode_pub.publish(current_gamemode);
		
		if (ros::Time::now()-last_show_time>ros::Duration(1.0))
		{
			ROS_INFO("PX4 Mode: %s", current_state.mode.c_str());
			
			last_show_time = ros::Time::now();
		}
        
        if (!breakmission_over)
        {
            if (current_state.mode == "OFFBOARD" &&ros::Time::now()-last_request>ros::Duration(1.0))
            {
                offb_set_mode.request.custom_mode = "AUTO.MISSION";
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
                {
                    ROS_INFO("MISSION enabled");
                    breakmission_over = true;
                }
            }
        }
        geometry_msgs::TwistStamped velocity_tw;
        velocity_pub.publish(velocity_tw);

        ros::spinOnce();
        rate.sleep();
        
        if (current_waypoints.waypoints.size()==0)
        {
            continue;
        }
        if (current_waypoints.waypoints[current_waypoints.waypoints.size()-1].is_current)
        {
            break;
        }
    }

    offb_set_mode.request.custom_mode = "OFFBOARD";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
    {
        ROS_INFO("Offboard enabled");
        last_request = ros::Time::now();
    }

    current_gamemode.gamemode = imav::GameMode::GAMEMODE_CHECK_H;
    
    while (ros::ok())
    {
        gamemode_pub.publish(current_gamemode);

        if (current_local_pose.pose.position.z<height_land)
        {
            break;
        }
        
        geometry_msgs::TwistStamped velocity_tw;

        velocity_tw = current_pid_velocity;
        //velocity_tw.twist.linear.z = -0.2;

        velocity_pub.publish(velocity_tw);
        
        ros::spinOnce();
        rate.sleep();
    }

    current_gamemode.gamemode = imav::GameMode::GAMEMODE_AUTOLAND;
    gamemode_pub.publish(current_gamemode);

    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
    {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }

    ros::Rate rate1(1);
    while (ros::ok())
    {
        gamemode_pub.publish(current_gamemode);
        ROS_INFO("PX4 Mode: %s", current_state.mode.c_str());
        
        ros::spinOnce();
        rate1.sleep();
    }

    ros::spin();

    return 0;
}
