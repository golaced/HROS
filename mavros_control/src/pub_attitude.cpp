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
#include <mavros_msgs/HomePosition.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped position;
void get_position(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    position = *msg;
}

mavros_msgs::HomePosition home_position;
void get_home_position(const mavros_msgs::HomePosition::ConstPtr& msg)
{
    home_position = *msg;
}

geometry_msgs::Twist att_tw;
void get_throttle(const geometry_msgs::Twist::ConstPtr& msg)
{
    att_tw = *msg;
}

void print(std_msgs::Float64& throttle, geometry_msgs::PoseStamped& atti)
{
    ROS_INFO("%.4f %.4f %.4f %.4f",atti.pose.orientation.x,atti.pose.orientation.y,atti.pose.orientation.z,atti.pose.orientation.w);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_attitude");
    ros::NodeHandle n;

    ros::Publisher pub_att = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 100);
    ros::Publisher pub_thr = n.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 100);
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Subscriber position_sub = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,get_position);
    ros::Subscriber home_position_sub = n.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home",10,get_home_position);

    ros::Subscriber key_sub = n.subscribe<geometry_msgs::Twist>("cmd_key",30,get_throttle);

    ros::Rate loop_rate(20);
        // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    ros::spinOnce();
    geometry_msgs::PoseStamped cmd_att;
    std_msgs::Float64 cmd_thr;
    int count = 1;
    double v[3] = {1.0, 1.0, 1.0};
    double lambda = 0.6;

    double v_norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    double theta = 0.0;
    att_tw.linear.x=0.5;

    ros::Time last_request = ros::Time::now();
    while (ros::ok())
    {
        //std::cout << current_state.mode << std::endl;
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //some_object = qp.getStatus();
        // some_object.print();
        //printf("%f\n",some_object.position_x);
        //Create attitude command message
        cmd_att.header.stamp = ros::Time::now();
        cmd_att.header.seq = count;
        cmd_att.header.frame_id = 1;
        cmd_att.pose.position.x = 0.0; //0.001*some_object.position_x;
        cmd_att.pose.position.y = 0.0; //0.001*some_object.position_y;
        cmd_att.pose.position.z = 0.3; //0.001*some_object.position_z;

        // cmd_att.pose.orientation.x = sin(theta / 2.0) * v[0] / v_norm;
        // cmd_att.pose.orientation.y = sin(theta / 2.0) * v[1] / v_norm;
        // cmd_att.pose.orientation.z = sin(theta / 2.0) * v[2] / v_norm;
        cmd_att.pose.orientation.w = 1;
//        cmd_att.pose.orientation.x = att_tw.angular.x;
//        cmd_att.pose.orientation.y = att_tw.angular.y;
//        cmd_att.pose.orientation.z = att_tw.angular.z;
        // cmd_att.pose.orientation.w = att_tw.linear.z;

        //Create throttle command message
        if(position.pose.position.z>5)
        {
            lambda = 0.5;
        }
//        cmd_thr.data = att_tw.linear.x;
        cmd_thr.data = 0.5;


        if(position.pose.position.z>1)
        {
            pub_att.publish(cmd_att);
        }

        //pub_thr.publish(cmd_thr);

        count++;
        theta = 0.5 * sin(count / 300.0);

//        ROS_INFO("theta:%f x:%f y:%f z:%f", theta,cmd_att.pose.orientation.x,cmd_att.pose.orientation.y,cmd_att.pose.orientation.z);
//        ROS_INFO("%f %f %f",home_position.latitude, home_position.longitude,home_position.altitude);
        print(cmd_thr,cmd_att);
        ros::spinOnce();
        loop_rate.sleep();
        /*
    if(count>1000){
        count =0;
    }
    */
    }

    return 0;
}
