#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <sstream>
#include <iostream>
#include <std_msgs/Float64.h>//global position relative alt
#include <geometry_msgs/Vector3.h> //三个float64

//订阅状态回调函数
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
//订阅UWB位置回调函数
geometry_msgs::Vector3 current_pos;
void get_location_cb(const geometry_msgs::Vector3::ConstPtr &msg) 
{
    current_pos = *msg;
}
//位置控制器结构体
typedef struct
{
    float Expect;//期望 
    float FeedBack;//反馈
    float Err;//误差
    float Last_Err;//上一时刻误差 
    float Err_Max;//误差限幅值 
    float Integrate_Separation_Err;//积分分离偏差值     先设100把 
    float Integrate;//积分值 
    float Integrate_Max;//积分限幅值 
    float Kp;
    float Ki;
    float Kd;
    float Control_OutPut;//控制器输出 
    float Last_Control_OutPut;//上一时刻控制器输出 
    float Control_OutPut_Limit;//输出限幅 

    float Last_FeedBack;//上一时刻反馈值 
    float Dis_Err;//微分量 
    float Dis_Error_History[5];//历史微分量 
}PID_Controler;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "holdheight");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber get_lacation_sub = nh.subscribe("current_pos", 1, get_location_cb);
    //ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
           // ("mavros/setpoint_velocity/cmd_vel", 10);
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
    pose.pose.position.z = 5;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    int state = 3;
    ros::Time last_request = ros::Time::now();

        last_request = ros::Time::now();

        while(ros::ok()) {
           if( (ros::Time::now() - last_request > ros::Duration(5.0))) break;
           
           pose.pose.position.x = 5;
           pose.pose.position.y = 5;
           //local_pos_pub.publish(pose);
           //geometry_msgs::TwistStamped velocity_tw;
           //velocity_pub.publish(velocity_tw);
               ROS_INFO_STREAM("location_x: " << current_pos.x);
               ROS_INFO_STREAM("location_y: " << current_pos.y);
               ROS_INFO_STREAM("location_z: " << current_pos.z);
           ros::spinOnce();
           rate.sleep();
       }
       ROS_INFO_STREAM("state="<<state);

    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
    {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }

    return 0;
}