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
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <std_msgs/Int32.h>

#include <imav/imavFunctions.h>
#include <imav/Barrel.h>
#include <imav/BarrelList.h>
#include <imav/GameMode.h>

#include <iostream>
using namespace std;
using namespace mavros_msgs;
using namespace imav;

bool haveBarrel = false;
int mission_status = MISSION_IDLE;

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
mavros_msgs::RCIn rcIn;
void get_rc_in(const mavros_msgs::RCIn::ConstPtr& msg)
{
    rcIn = *msg;
/*    ROS_INFO("RC IN:");
    for (size_t i = 0; i < rcIn.channels.size(); i++)
    {
        cout<<rcIn.channels[i]<<" ";
    }
    cout<<endl;
    */
}

geometry_msgs::PoseStamped current_local_pose;
void get_local_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_local_pose = *msg;
}

ros::Publisher barrel_targetPosition_pub;

imav::GameMode current_gamemode;

imav::BarrelList current_barrelList;
void get_BarrelList(const imav::BarrelList::ConstPtr& msg)
{
    current_barrelList = *msg;
    if (current_barrelList.barrels.size()!=0)
    {
        haveBarrel = true;
    }
    else
    {
        haveBarrel = false;
    }

    sortBarrelsByArea(current_barrelList);
    
    geometry_msgs::PoseStamped barrel_pose;
    barrel_pose.pose.position.x = -1;
    barrel_pose.pose.position.y = -1;
    if (haveBarrel)
    {     
        barrel_pose.pose.position.x = (current_barrelList.barrels[0].left+current_barrelList.barrels[0].right)/2;
        barrel_pose.pose.position.y = (current_barrelList.barrels[0].top+current_barrelList.barrels[0].bottom)/2;
        //ROS_INFO("%f, %f", barrel_pose.pose.position.x,barrel_pose.pose.position.y);
    }

    if (current_gamemode.gamemode==imav::GameMode::GAMEMODE_CHECK_BARREL)
    {
        barrel_targetPosition_pub.publish(barrel_pose);
    }
}

imav::BarrelList checkedbarrelList;
imav::Barrel targetBarrel;

int main(int argc, char **argv)
{
    int Num = 0;
    std::string barrel_gps_write_file = initBarrelWritePath();
    std::ofstream barrel_gps_writer;
    ROS_INFO("Barrel GPS File:%s", barrel_gps_write_file.c_str());

    ros::init(argc, argv, "gameplan_barrel");
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
    
    ros::Subscriber barrel_sub = nh.subscribe<imav::BarrelList>("imav/PotDetect_Barrel", 1, get_BarrelList);
    
    barrel_targetPosition_pub = nh.advertise<geometry_msgs::PoseStamped>("imav/target_position", 1);
    ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in",1,get_rc_in);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1000);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    //send a few setpoints before starting
    for(int i = 1000; ros::ok() && i > 0; --i){
        ros::spinOnce();
        rate.sleep();
    }
    
    double height_land = 1.0;
    ros::param::get("~height_land", height_land);
    if (height_land<1.0)
    {
		height_land = 1.0;
    }
    
    int counter_to_savebarrel = 10;
    ros::param::get("~counter_to_savebarrel", counter_to_savebarrel);
    int counter_to_checkbarrel = 5;
    ros::param::get("~counter_to_checkbarrel", counter_to_checkbarrel);

    ros::Time last_request = ros::Time::now();
    last_request = ros::Time::now();
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    bool breakmission_over = false;
    current_gamemode.gamemode = imav::GameMode::GAMEMODE_CHECK_BARREL;
    
    ros::Time lase_loop_time = ros::Time::now();
    double loop_T = ros::Time::now().toSec()-lase_loop_time.toSec();

    last_request = ros::Time::now();
    ros::Time last_show_time = ros::Time::now();
    while (ros::ok())
    {
        gamemode_pub.publish(current_gamemode);
        geometry_msgs::TwistStamped velocity_tw;
        if (ros::Time::now()-last_show_time>ros::Duration(1.0))
        {
            ROS_INFO("PX4 Mode: %s", current_state.mode.c_str());
            ROS_INFO("mission_status: %d", mission_status);
            last_show_time = ros::Time::now();
        }
        
        if (current_waypoints.waypoints.size()==0)
        {
            velocity_pub.publish(velocity_tw);
            ros::spinOnce();
            rate.sleep();
            continue;
        }
	
        if (!breakmission_over)
        {
            if (current_state.mode == "OFFBOARD" &&ros::Time::now()-last_request>ros::Duration(1.0))
            {
                offb_set_mode.request.custom_mode = "AUTO.MISSION";
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
                {
                    ROS_INFO("MISSION enabled 1");
                    breakmission_over = true;
                }
            }
        }
	
        static int cnt[20];
        
        //ROS_INFO("cnt[0]: %d", cnt[0]);

        if (ros::Time::now()-lase_loop_time>ros::Duration(0.05))
        {
            loop_T = ros::Time::now().toSec()-lase_loop_time.toSec();
            lase_loop_time = ros::Time::now();
            switch(mission_status)
            {
                case MISSION_IDLE:
                {
                    if(haveBarrel&&rcIn.channels[4]>1650)
                    {
		    
                        if (outofcheckedarea(current_gps, checkedbarrelList))
                        {
                            cnt[0]++;
                        }
                    }
                    else
                    {
                        cnt[0]=0;
                    }
                    if(cnt[0]>counter_to_checkbarrel)
                    {
                        cnt[0]=0;            
                        mission_status=MISSION_CHECK_BARREL;      
                        targetBarrel = current_barrelList.barrels[0];
                    }
                    break;
                }
                
                case MISSION_CHECK_BARREL:
                {
                    if(!haveBarrel)
                    {
                        cnt[0]++;
                    }
                    else
                    {
                        cnt[0]=0;
                    }
                    if(cnt[0]>2.0/loop_T||cnt[1]++>5.0/loop_T)
                    {
                        mission_status=MISSION_IDLE;
                        cnt[1]=cnt[0]=0;
                    }
                
                    if (haveBarrel && current_barrelList.barrels[0].area>1000
                        &&current_barrelList.barrels[0].color == targetBarrel.color 
                        &&(!current_barrelList.barrels[0].latitude))
                    {
                        cnt[2]++;
                    }

                    if(cnt[2]>counter_to_savebarrel)
                    {
                        mission_status=MISSION_SAVE_GPS;
                        cnt[2]=0;
                    }
                    break;
                }
                case MISSION_SAVE_GPS:
                {
                    ROS_INFO("Barrel gps:[%.8f %.8f]", current_gps.longitude, current_gps.latitude);
                    Barrel cur_barrel;
                    cur_barrel.latitude = current_gps.latitude;
                    cur_barrel.longitude = current_gps.longitude;
                    checkedbarrelList.barrels.push_back(cur_barrel);

                    barrel_gps_writer.open(barrel_gps_write_file.c_str(), ios::app);
                    if (!barrel_gps_writer.is_open())
                    {
                        ROS_WARN("barrel_gps_writer open failed!!!");
                    }
                    else
                    {
                        Num++;
                        barrel_gps_writer << setprecision(15);
                        // barrel_gps_writer<<current_gps.longitude<<" "<<current_gps.latitude<<endl;
                        // string long_dufenmiao = lonlat2degreefenmiao(current_gps.longitude);
                        // string lat_dufenmiao = lonlat2degreefenmiao(current_gps.latitude);
                        // barrel_gps_writer<<long_dufenmiao<<" "<<lat_dufenmiao<<endl;
                        // barrel_gps_writer<<endl;
                        // barrel_gps_writer.close();

                        barrel_gps_writer << "<Placemark>" << endl;
                        barrel_gps_writer << "<name>B" << Num << "</name>" << endl;
                        barrel_gps_writer << "<description>Color is:Red</description>" << endl;
                        barrel_gps_writer << "<Point>" << endl;
                        barrel_gps_writer << "<extrude>1</extrude>" << endl;
                        barrel_gps_writer << "<altitudeMode>relativeToGround</altitudeMode>" << endl;
                
                        barrel_gps_writer << "<coordinates>" << current_gps.longitude << "," << current_gps.latitude << ",50</coordinates>"<<endl;
                        barrel_gps_writer << "</Point>" << endl;
                        barrel_gps_writer << "</Placemark>" << endl;
                
                        barrel_gps_writer.close();
                    }

                    mission_status = MISSION_IDLE;
                    break;
                }
                
                default:
                    break;
            }
        }
	
	    if (rcIn.channels[4]<1650)
        {
            mission_status = MISSION_IDLE;
        }
        
        //ouput
        switch(mission_status)
        {
            case MISSION_IDLE:
            {
                if (rcIn.channels[4]>1650)
                {
                    if (current_state.mode!="AUTO.MISSION")
                    {
                        offb_set_mode.request.custom_mode = "AUTO.MISSION";
                        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
                        {
                            ROS_INFO("MISSION enabled 2");
                        }
                    }
                }
		        break;
            }

            case MISSION_CHECK_BARREL:
            {
                /*if (current_state.mode!="OFFBOARD")
                {
                    offb_set_mode.request.custom_mode = "OFFBOARD";
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
                    {
                        ROS_INFO("OFFBOARD enabled 1");
                    }
                }

                velocity_tw = current_pid_velocity;
		        velocity_tw.twist.linear.z = 0.0;
                if(current_barrelList.barrels[0].color==targetBarrel.color)
                {
                    velocity_tw.twist.linear.z = 0.0;   
                }
                if (current_local_pose.pose.position.z<5.0)
                {
                    velocity_tw.twist.linear.z = 0.0;
                }
		
		        ROS_INFO("velocity_tw:[%.2f,%.2f]",velocity_tw.twist.linear.x,velocity_tw.twist.linear.y);

                velocity_pub.publish(velocity_tw);*/

                break;
            }
        }
	
        if (current_waypoints.waypoints[current_waypoints.waypoints.size()-1].is_current)
        {
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    barrel_gps_writer.open(barrel_gps_write_file.c_str(), ios::app);
	barrel_gps_writer << "</Folder>" << endl;
	barrel_gps_writer << "</kml>" << endl;
	barrel_gps_writer.close();

    offb_set_mode.request.custom_mode = "OFFBOARD";

    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
    {
        ROS_INFO("Offboard enabled 2");
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
        //velocity_tw.twist.linear.z = -0.3;

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
	    ROS_INFO("------Game Over------");
        ROS_INFO("PX4 Mode: %s", current_state.mode.c_str());
        
        ros::spinOnce();
        rate1.sleep();
    }


    ros::spin();

    return 0;
}
