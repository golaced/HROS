#include "imav/transformWaypoints.h"
#include <ros/ros.h>

mavros_msgs::WaypointList transformWaypoints(mavros_msgs::WaypointList& pointlist, double mission_speed, double wp_height)
{
    mavros_msgs::WaypointList wp_list;

    // set velocity
    mavros_msgs::Waypoint waypoint_setvelocity;
    waypoint_setvelocity.x_lat = 0.0;
    waypoint_setvelocity.y_long = 0.0;
    waypoint_setvelocity.z_alt = 0.0;
    waypoint_setvelocity.command = mavros_msgs::CommandCode::CMD_DO_CHANGE_SPEED;
    waypoint_setvelocity.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    waypoint_setvelocity.autocontinue = true;
    waypoint_setvelocity.is_current = true;
    waypoint_setvelocity.param1 = 1;
    waypoint_setvelocity.param2 = mission_speed;//velocity
    waypoint_setvelocity.param3 = -1;

    wp_list.waypoints.push_back(waypoint_setvelocity);

    if (pointlist.waypoints.size()>0)
    {
        mavros_msgs::Waypoint waypoint0;
        waypoint0.x_lat = pointlist.waypoints[0].x_lat;
        waypoint0.y_long = pointlist.waypoints[0].y_long;
        waypoint0.z_alt = wp_height;
        waypoint0.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
        waypoint0.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint0.autocontinue = true;
        waypoint0.is_current = false;
        wp_list.waypoints.push_back(waypoint0);
    }

    for (size_t i = 1; i < pointlist.waypoints.size(); i++)
    {
        mavros_msgs::Waypoint waypoint1;
        waypoint1.x_lat = pointlist.waypoints[i].x_lat;
        waypoint1.y_long = pointlist.waypoints[i].y_long;
        waypoint1.z_alt = wp_height;
        waypoint1.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint1.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint1.autocontinue = true;
        wp_list.waypoints.push_back(waypoint1);
        //waypoint1.param1 = 5;
    }
    mavros_msgs::Waypoint waypoint1;
    waypoint1.x_lat = pointlist.waypoints[pointlist.waypoints.size()-1].x_lat;
    waypoint1.y_long = pointlist.waypoints[pointlist.waypoints.size()-1].y_long;
    waypoint1.z_alt = wp_height;
    waypoint1.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    waypoint1.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint1.autocontinue = true;
    wp_list.waypoints.push_back(waypoint1);

    return wp_list;
}


bool pushWaypointList(mavros_msgs::WaypointList& pointlist)
{
    ros::NodeHandle nh;
    ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>
            ("mavros/mission/pull");
    ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");
    mavros_msgs::WaypointPush waypoint_push;
    waypoint_push.request.waypoints = pointlist.waypoints;
    if (waypoint_push_client.call(waypoint_push)&&waypoint_push.response.success)
    {
        ROS_INFO("WP_transfered:%d", waypoint_push.response.wp_transfered);
        ROS_INFO("Waypoint Push success");
    }

}
bool clearWaypointList()
{
    ros::NodeHandle nh;
    ros::ServiceClient waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>
            ("mavros/mission/clear");
    mavros_msgs::WaypointClear waypoint_clear;
    if (waypoint_clear_client.call(waypoint_clear)&&waypoint_clear.response.success)
    {
        ROS_INFO("Waypoint Clear success");
    }            
}