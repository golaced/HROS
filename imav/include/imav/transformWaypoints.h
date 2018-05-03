#ifndef __TRANSFORMWAYPOINTS_H__
#define __TRANSFORMWAYPOINTS_H__

#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/CommandCode.h>

mavros_msgs::WaypointList transformWaypoints(mavros_msgs::WaypointList& pointlist, double mission_speed = 1.0, double wp_height = 5.0);
bool pushWaypointList(mavros_msgs::WaypointList& pointlist);
bool clearWaypointList();

#endif //__TRANSFORMWAYPOINTS_H__