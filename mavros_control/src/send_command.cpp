#include <ros/ros.h>
#include <mavros_msgs/CommandInt.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandTriggerControl.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "send_command_node");
    ros::NodeHandle nh;
    ros::ServiceClient arm_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    ros::ServiceClient sethome_client = nh.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");

    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandTOL takeoff_cmd;
    mavros_msgs::CommandTOL land_cmd;
    mavros_msgs::CommandHome sethome_cmd;

    arm_cmd.request.value = true;
    takeoff_cmd.request.longitude = 8.545796;
    takeoff_cmd.request.latitude = 47.397948;
    takeoff_cmd.request.yaw = 3.1415926/2;
    takeoff_cmd.request.altitude = 500;
    sethome_cmd.request.current_gps = true;

    sethome_client.call(sethome_cmd);
    if (sethome_cmd.response.success)
    {
        ROS_INFO("Set Home success");
    }

    ros::Time last_request = ros::Time::now();

    arm_client.call(arm_cmd);
    if (arm_cmd.response.success)
    {
        ROS_INFO("Arm success");
    }
    while (ros::Time::now() - last_request < ros::Duration(2.0))
    {
        /* code for True */
    }
    
    takeoff_client.call(takeoff_cmd);
    if (takeoff_cmd.response.success)
    {
        ROS_INFO("takeoff command success");
    }

    while (ros::Time::now() - last_request < ros::Duration(10.0))
    {
        /* code for True */
    }
    ROS_INFO("time wake up");

    land_client.call(land_cmd);
    if(land_cmd.response.success)
    {
        ROS_INFO("land command success");
    }


    // ros::Rate loop_rate(10);
    // while (ros::ok())
    // {
    //     /* code for loop body */
    
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    
    

    return 0;
}
