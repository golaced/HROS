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
#include <math.h>
using namespace std;

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))
#define M_M_PI			3.14159265f
#define M_M_PI_2			(M_M_PI / 2.0f)
#define NAV_HF_HOME_DIST_D_MIN	2.0f						// do not compute dynamic bearing when closer than this to home position (zero to never compute)
#define NAV_HF_HOME_DIST_FREQ	4						// update distance to home at this Hz, should be > 0 and <= 400
#define NAV_HF_HOME_BRG_D_MAX	1.0f * DEG_TO_RAD				// re-compute headfree reference angles when bearing to home changes by this many degrees (zero to always re-compute)
#define NAV_HF_DYNAMIC_DELAY	((int)3e6f)					// delay micros before entering dynamic mode after switch it toggled high
#define RAD_TO_DEG		(180.0f / M_M_PI)
#define DEG_TO_RAD		(M_M_PI / 180.0f)

double r1, r2,local_lat,local_lon;

void CalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

 void CalcGlobalDistance(double lat, double lon,float local_Lat,float local_Lon,float *posNorth,float *posEast ) {
    *posNorth = (lat - local_Lat) * r1;
    *posEast =  (lon - local_Lon) * r2;
}


 void CalcGlobalLocation(float posNorth,float posEast,float local_Lat,float local_Lon,double *GPS_W_F,double* GPS_J_F){ 
    *GPS_W_F=(double)posNorth/(double)(r1+0.1)+local_Lat;
    *GPS_J_F=(double)posEast/(double)(r2+0.1)+local_Lon;
}

// input lat/lon in degrees, returns distance in meters
float navCalcDistance(double lat1, double lon1, double lat2, double lon2) {
    float n = (lat1 - lat2) * r1;
    float e = (lon1 - lon2) * r2;
    return sqrtf(n*n + e*e);
}

// input lat/lon in degrees, returns bearing in radians
float navCalcBearing(double lat1, double lon1, double lat2, double lon2) {
    float n = (float)((lat2 - lat1) * (double)DEG_TO_RAD * r1);
    float e = (float)((lon2 - lon1) * (double)DEG_TO_RAD * r2);
    float ret = atan2f(e, n);

    if (!isfinite(ret))
        ret = 0.0f;

    return ret*57.3;
}

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
double current_yaw_rad,current_yaw_x;
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
    current_yaw_x=atan2(2. * (q0 * q3 + q1 * q2), 1. - 2. * (q2 * q2 + q3 * q3)) * 57.3f;
    current_yaw = 90 - current_yaw_x;
    if (current_yaw < 0)
    {
        current_yaw += 360;
    }
    current_yaw_rad = current_yaw / 57.3f;

    //ROS_INFO("%.2f %.2f %.2f %.2f", Pit, Rol, Yaw, current_yaw);
}

sensor_msgs::NavSatFix current_gps;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_gps = *msg;
    //ROS_INFO("%f %f %f",current_gps.longitude,current_gps.latitude,current_gps.altitude);
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

void print_wp(const mavros_msgs::Waypoint &wp)
{
    ROS_INFO("gps:%f %f %f", wp.x_lat, wp.y_long, wp.z_alt);
    ROS_INFO("%d %d %d %d", wp.command, wp.frame, wp.autocontinue, wp.is_current);
    ROS_INFO("param:%f %f %f %f", wp.param1, wp.param2, wp.param3, wp.param4);
}

mavros_msgs::Waypoint target_waypoint;
double target_dis = 0.0;
double wp_p = 0.2;
float wp_i = 0.001;
double y_speed_g = 0.0;
double x_speed_g = 0.0;

double y_speed_bady = 0.0;
double x_speed_bady = 0.0;
mavros_msgs::WaypointList current_waypoints;
void get_waypoints(const mavros_msgs::WaypointList::ConstPtr &msg)
{
    current_waypoints = *msg;

    //target_waypoint = current_waypoints.waypoints[current_waypoints.waypoints.size() - 2];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cooperation_node");
    ros::NodeHandle nh;
    ros::Subscriber get_uwb_distance_sub = nh.subscribe("uwb_distance", 1, get_uwb_disrance_cb);
    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
    ("mavros/mission/waypoints", 1, get_waypoints);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, get_local_pose);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
    ("/mavros/global_position/global", 1, get_gps_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
    ("/mavros/imu/data",2,imu_cb);

    ros::Publisher pid_pub = nh.advertise<std_msgs::Float64>("pid_out", 1);

    int en_spd0 = 0;
    int en_spd1 = 0;

    ros::param::get("~en_spd0", en_spd0);
    ros::param::get("~en_spd1", en_spd1);

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

    CalcEarthRadius(current_gps.latitude);
    
    double lat_takeoff = current_gps.latitude;
    double long_takeoff = current_gps.longitude;
    double fly_dis = 0.0;
    local_lat = lat_takeoff;
    local_lon = long_takeoff;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.LAND";

    string distance_str;

    int distance_int = 0; //???????
    int distance_filter = 0;

    int exp_distance = 1900;
    ros::param::get("~exp_distance", exp_distance);
    int dis_err = 0;
    float exp_high = 3.5; //????
    ros::param::get("~exp_high", exp_high);
    double land_dis = 50.0;
    ros::param::get("~land_dis", land_dis);
    float high_err = 0;
    int dis_err_last = 0;
    int dis_integral = 0;
    float kp = 0.0035;
    ros::param::get("~kp", kp);
    float kp_high = 0.2;
    float ki = 0;
    float kd = 0.007;
    ros::param::get("~kd", kd);
    float pid_out = 0;
    float pid_out_high = 0;

    ros::Time last_request = ros::Time::now();

    std_msgs::Float64 pid_msg;
    ros::Time target_time = ros::Time::now();
    int target_wp_index = 1;
    double target_change_time = 10.0;
    ros::param::get("~target_change_time", target_change_time);
    while (ros::ok())
    {
        if (ros::Time::now()-target_time>ros::Duration(target_change_time))
        {
            target_time = ros::Time::now();
            if (target_wp_index > current_waypoints.waypoints.size()-2)
            {
                target_wp_index = current_waypoints.waypoints.size() - 2;
            }
            target_waypoint = current_waypoints.waypoints[target_wp_index];
            target_wp_index++;
        }
        float ex, ey;
        static float int_x, int_y;
        float pos_x, pos_y, tar_x, tar_y ,tar_x_9,tar_y_9;
        float yaw_ep;
        CalcGlobalDistance(current_gps.latitude, current_gps.longitude, local_lat, local_lon, &pos_y, &pos_x);
        CalcGlobalDistance(target_waypoint.x_lat, target_waypoint.y_long, local_lat, local_lon, &tar_y, &tar_x);
        yaw_ep = navCalcBearing(target_waypoint.x_lat, target_waypoint.y_long, current_gps.latitude, current_gps.longitude);

        float b = tar_y-tan(current_yaw_x / 57.3f) * tar_x;
        float b2 = pos_y+ pos_x/tan(current_yaw_x / 57.3f) ;
        tar_x_9 = (b2 - b) / (tan(current_yaw_x / 57.3f) + 1 / tan(current_yaw_x / 57.3f));
        tar_y_9 = tan(current_yaw_x  / 57.3f) * tar_x_9 + b;

        ex =LIMIT(tar_x_9 - pos_x, -8,8);
        ey =LIMIT( tar_y_9 - pos_y,-8,8);
        int_x += ex * wp_i;
        int_y += ey * wp_i;
        int_x = LIMIT(int_x, -1, 1);
        int_y = LIMIT(int_y, -1, 1);
        y_speed_g = wp_p * ey+int_y;
        x_speed_g = wp_p * ex+int_x;
      
        y_speed_bady = y_speed_g * cos(current_yaw_rad) + x_speed_g * sin(current_yaw_rad);
        x_speed_bady = -y_speed_g * sin(current_yaw_rad) + x_speed_g * cos(current_yaw_rad);
        x_speed_bady *= 1;
        y_speed_bady *= 1;

        x_speed_bady = LIMIT(x_speed_bady, -3, 3);
        y_speed_bady = LIMIT(y_speed_bady, -3, 3);
        ROS_INFO("yaw_ep:%f, ex:%f,ey:%f x:%f,y:%f", yaw_ep, ex, ey, x_speed_bady, y_speed_bady);

        distance_int = current_distance.data;

        //distance_filter = Moving_Median(8, distance_int);
		distance_filter = distance_int;
        ROS_INFO_STREAM("distance_filter!!!: " << distance_filter);
        //PID

        dis_err = exp_distance - distance_filter;
        high_err = exp_high - current_pose.pose.position.z;
        pid_out = (kp * dis_err + kd*(dis_err-dis_err_last));
        pid_out *= -1;
        dis_err_last = dis_err;
        pid_out_high = kp_high * high_err;
        if (pid_out > 0.8 || pid_out < -0.8)
        {
            pid_out = 0.8 * pid_out / abs(pid_out);
        }

        if (pid_out_high > 0.3 || pid_out_high < -0.3)
        {
            pid_out_high = 0.3 * pid_out_high / abs(pid_out_high);
        }
            
        ROS_INFO_STREAM("PID_OUT= " << pid_out);
        pid_msg.data = pid_out;
        pid_pub.publish(pid_msg);

        geometry_msgs::TwistStamped velocity_tw;
        velocity_tw.twist.linear.z = pid_out_high;
        velocity_tw.twist.linear.x = sin(current_yaw_rad) * pid_out*en_spd0 + cos(current_yaw_rad) * x_speed_bady*en_spd1;
        velocity_tw.twist.linear.y = cos(current_yaw_rad) * pid_out*en_spd0 - sin(current_yaw_rad) * x_speed_bady*en_spd1;

        ROS_INFO("velocity_tw:%f %f", velocity_tw.twist.linear.x, velocity_tw.twist.linear.y);

        velocity_pub.publish(velocity_tw);

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