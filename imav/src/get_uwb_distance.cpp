#include <ros/ros.h>
#include <ros/param.h>
#include <serial/serial.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>
#include <sstream>
#include <iostream>
using namespace std;


serial::Serial uwb; //声明串口对象

int main(int argc,char** argv)
{
    ros::init(argc,argv,"get_uwb_distance_node");//初始化节点
    ros::NodeHandle nh;//声明节点句柄
    ros::Publisher dis_pub = nh.advertise<std_msgs::UInt32>("uwb_distance", 1);

    std::string uwb_port_name("/dev/ttyACM0");//gaichuankou
    ros::param::get("~uwb_port_name", uwb_port_name);

    try
    {
        uwb.setPort(uwb_port_name);/* code for Try */
        uwb.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        uwb.setTimeout(to);
        uwb.open();
    }
    catch (serial::IOException& e)
    {
       ROS_ERROR_STREAM("Unable to open port "); 
       return -1;  /* code for Catch */
    }
    //指定循环的频率 
    ros::Rate loop_rate(20); 

    int data_length = 0;
    std_msgs::UInt32 current_dis;

    while (ros::ok())
    { 
        unsigned int distance = 0;
        data_length = uwb.available();
        if(data_length)
        { 
            std::string data_buf = uwb.read(data_length);

            for (size_t i = 0; i < data_length; i++)
            {
                if (data_buf[i]>'9')
                {
                    distance = distance *16+ (unsigned int)(data_buf[i]-'a'+10);
                }
                else
                {
                    distance = distance *16+ (unsigned int)(data_buf[i]-'0');
                }
            }

            current_dis.data = distance;

            dis_pub.publish(current_dis);

            ROS_INFO_STREAM("Read: " << current_dis.data);
        } 

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
  
    } 
    
}