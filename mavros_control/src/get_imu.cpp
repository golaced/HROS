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
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/AttitudeTarget.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

sensor_msgs::Imu current_imu;
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    current_imu = *msg;
    double q0,q1,q2,q3;
    q0 = current_imu.orientation.w;
    q1 = current_imu.orientation.x;
    q2 = current_imu.orientation.y;
    q3 = current_imu.orientation.z;

    double Rol = atan2(2 * (q0 * q1 + q2 * q3), -1 + 2 * (q1 * q1 + q2 * q2))
                * 57.3f;
    double Pit = asin(2 * (-q1 * q3 + q0 * q2)) * 57.3f;
    double Yaw = atan2(2 * (-q1 * q2 - q0 * q3), 1 - 2 * (q0 * q0 + q1 * q1))
                * 57.3f;
    double Yaw1 = atan2(2. * (q0 * q3 + q1 * q2), 1. - 2. * (q2 * q2 + q3 * q3)) * 57.3f;
    ROS_INFO("%.2f %.2f %.2f %.2f", Pit, Rol, Yaw,Yaw1);
}

mavros_msgs::AttitudeTarget current_atti;
void atti_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg){
    current_atti = *msg;
//    ROS_INFO("%.2f %.2f %.2f", current_imu.angular_velocity.x,
//             current_imu.angular_velocity.y, current_imu.angular_velocity.z);
}

geometry_msgs::PoseStamped pose_local;
void pose_local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose_local = *msg;
//    ROS_INFO("or: %.2f %.2f %.2f %.2f", pose_local.pose.orientation.w,pose_local.pose.orientation.x,pose_local.pose.orientation.y,pose_local.pose.orientation.z);
//    ROS_INFO("position: %.2f %.2f %.2f", pose_local.pose.position.x,pose_local.pose.position.y,pose_local.pose.position.z);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_locol_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1,pose_local_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("/mavros/imu/data",2,imu_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


    int n=0;
    while (ros::ok())
    {
//        std::cout << current_state.mode << std::endl;
//        if (current_state.mode != "OFFBOARD" &&
//            (ros::Time::now() - last_request > ros::Duration(2.0)))
//        {
//            if( set_mode_client.call(offb_set_mode) &&
//                offb_set_mode.response.success){
//                ROS_INFO("Offboard enabled");
//            }
//            last_request = ros::Time::now();
//        }
//        else
//        {
//            if( !current_state.armed &&
//                (ros::Time::now() - last_request > ros::Duration(2.0))){
//                if( arming_client.call(arm_cmd) &&
//                    arm_cmd.response.success){
//                    ROS_INFO("Vehicle armed");
//                }
//                last_request = ros::Time::now();
//            }
//        }

//        if (current_state.armed)
//        {

//        }

        n++;
        //if(n>200)
//        {
//            offb_set_mode.request.custom_mode = "MANUAL";
//            if (current_state.mode != "MANUAL" &&
//                (ros::Time::now() - last_request > ros::Duration(1.0)))
//            {
//                if( set_mode_client.call(offb_set_mode) &&
//                    offb_set_mode.response.success){
//                    ROS_INFO("MANUAL enabled");
//                }
//                last_request = ros::Time::now();
//            }
//            else
//            {
//                arm_cmd.request.value = false;
//                if( current_state.armed &&
//                    (ros::Time::now() - last_request > ros::Duration(1.0))){
//                    if( arming_client.call(arm_cmd) &&
//                        arm_cmd.response.success){
//                        ROS_INFO("Vehicle disarmed");
//                        break;
//                    }
//                    last_request = ros::Time::now();
//                }
//            }
//        }
//        local_pos_pub.publish(pose);


        // ROS_INFO("%f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
