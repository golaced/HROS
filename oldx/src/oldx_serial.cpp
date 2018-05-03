#include <ros/ros.h>
#include <serial/serial.h>
#include <serial/v8stdint.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ByteMultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h> 
#include "oldx.h"
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <fstream>
#include<ctime>  
using std::cout;
using std::endl;
using namespace std;
serial::Serial m_serial;

float x = 0.0, y = 0.0, z = 0.0, yaw = 0.0, yaw_rad = 0.0;
float x_min=0.0,x_max=0.0,y_min=0.0,y_max=0.0,z_min=0.0,z_max=0.0,yaw_min=0.0,yaw_max=0.0;
float mode_tx;
geometry_msgs::Vector3 mode_rx;
std_msgs::ByteMultiArray data_to_send;
void oldx_send_cb(const std_msgs::ByteMultiArray::ConstPtr &msg)
{
	data_to_send.data.clear();
	data_to_send = *msg;
#ifdef DEBUG_COUT
	cout<<data_to_send<<endl;
#endif
}

void SEND_PX4(void)
{
	// cout<<"*****"<<endl;
	m_serial.write((unsigned char *)data_to_send.data.data(), data_to_send.data.size());
}

bool checkcommand(const unsigned char *data_buf, int data_length)
{
	if (!(*(data_buf) == 0xFA && *(data_buf + 1) == 0xFB && *(data_buf + 2) == 0x04 && *(data_buf + 3) == 0x01))
	{
		return false; //判断帧头
	}
	if (*(data_buf + data_length - 1) != 0xFE)
	{
		return false;
	}
	return true;
}

int data_save[20];
void decode(const unsigned char *data_buf, int data_length)
{
	int16_t temp = 0,i,j; //temp==>mm, x==>m
	unsigned char mode = data_buf[4];
	temp = data_buf[5];
	temp <<= 8;
	temp |= data_buf[6];
	x = (float)temp / 1000;
	temp = data_buf[7];
	temp <<= 8;
	temp |= data_buf[8];
	y = (float)temp / 1000;
	temp = data_buf[9];
	temp <<= 8;
	temp |= data_buf[10];
	z = (float)temp / 1000;
	temp = data_buf[11];
	temp <<= 8;
	temp |= data_buf[12];
	yaw = (float)temp / 100; //degree
        mode_rx.y=data_buf[13];//data
	mode_rx.z=data_buf[14];//video
	
	for(i=0;i<11;i++){
	temp = data_buf[15+i*2];
	temp <<= 8;
	temp |= data_buf[15+i*2+1];
	data_save[i]=temp;
	}
	x = LIMIT(x, x_min, x_max);
	y = LIMIT(y, y_min, y_max);
	z = LIMIT(z, z_min, z_max);
	yaw = LIMIT(yaw, yaw_min, yaw_max);
	yaw_rad = -1 * yaw / 57.3;
	mode_rx.x=mode_tx=mode;
	
	
	
	static int cnt;
	if(cnt++>40){cnt=0;
	//cout << x << " " << y << " " << z << " " << yaw <<" "<<mode_rx<<"end"<<endl;
	} 
#ifdef DEBUG_COUT
	cout << x << " " << y << " " << z << " " << yaw << endl;
#endif
}

int video_num = 0;
int write_flag;
int newfile_flag;
std::ifstream video_num_read;
std::ofstream video_num_write;
std::string video_num_path("/home/odroid/DATA_SAVE/file_num.txt");
std::string writer_path("/home/odroid/DATA_SAVE/");


void startWriteFile(std::ofstream &writer_txt,float dt)
{   static int init_for_map_est=0;
    int i;
    std::stringstream ss;
    static string file_name;
	
    if(newfile_flag==1){
    video_num_read.open(video_num_path.c_str());
    video_num_read >> video_num;
    video_num_read.close();

    newfile_flag=2;
    video_num=video_num+1;
    video_num_write.open(video_num_path.c_str());
    video_num_write << (video_num );
    video_num_write.close();
    ss << video_num;
    ss >> file_name;
    file_name += ".txt";
    writer_txt.open((writer_path + file_name).c_str());
    cout<<"data write path:"<<writer_path+file_name<<endl;
   }
 
   
 if(newfile_flag==2)
    writer_txt<<(int)(dt*1000)<<"\t"<<data_save[0]<<"\t"<<data_save[1]<<"\t"<<
    data_save[2]<<"\t"<<data_save[3]<<"\t"<<data_save[4]<<"\t"<<
    data_save[5]<<"\t"<<data_save[6]<<"\t"<<data_save[7]<<"\t"<<
    data_save[8]<<"\t"<<data_save[9]<<"\t"<<data_save[10]<<"\t"
    <<endl;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "oldx_serial");
	ros::NodeHandle nh;
	ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("oldx/velocity", 1);
	ros::Publisher mode_pub = nh.advertise<geometry_msgs::Vector3>("mode_fc", 1);
	ros::Subscriber sub = nh.subscribe("oldx/oldx_send", 1, oldx_send_cb);
       
	std::string port_name;
	int baudrate = 230400;
	serial::Timeout timeout(serial::Timeout::simpleTimeout(1000));
	ros::param::get("~port_name", port_name);
	ros::param::get("~baudrate", baudrate);
	ros::param::get("~x_min", x_min);
	ros::param::get("~x_max", x_max);
	ros::param::get("~y_min", y_min);
	ros::param::get("~y_max", y_max);
	ros::param::get("~z_min", z_min);
	ros::param::get("~z_max", z_max);
	ros::param::get("~yaw_min", yaw_min);
	ros::param::get("~yaw_max", yaw_max);

	ROS_INFO("serial port name:%s", port_name.c_str());
	ROS_INFO("serial baudrate:%d", baudrate);

	m_serial.setPort(port_name);
	m_serial.setBaudrate(baudrate);
	m_serial.setTimeout(timeout);
	m_serial.open();
	if (!m_serial.isOpen())
	{
		cout << "serial open failed!" << endl;
		return -1;
	}

	size_t data_length = 0;
	unsigned char sum = 0;
	unsigned char data_buf[BUFF_SIZE] = {0};
	int16_t temp = 0;

	geometry_msgs::TwistStamped velocity;
		
	ros::Rate loop_rate(200);
	std::ofstream file_writer;
	ros::Time last_request = ros::Time::now();
   
	while (ros::ok())
	{clock_t start,finish;  
	start=clock();  
		data_length = m_serial.available();
		if (data_length)
		{
			m_serial.read(data_buf, data_length);
			if (checkcommand(data_buf, data_length))
			{
				decode(data_buf, data_length);
			}
		}

		velocity.twist.linear.x = x;
		velocity.twist.linear.y = y;
		velocity.twist.linear.z = z;

		velocity.twist.angular.z = yaw_rad;
		velocity.twist.angular.x = mode_tx;
		//cout<<mode_tx<<endl;
		velocity_pub.publish(velocity);
		mode_pub.publish(mode_rx);
		SEND_PX4();



		//saving
		static int save_state;
		//ros::Time dt=ros::Time::now() - last_request;
		finish=clock();  
		float dt=(float)( finish-start )/CLOCKS_PER_SEC;
		static float dt_s=0;
		int save_trig;
		dt_s+=dt;
		//if(dt_s>0.000)
		{save_trig=1;}
		switch(save_state)
		{
		 case 0:if(mode_rx.y==1)
		        {newfile_flag=1,save_state=1;}
			break;
		 case 1:
		       dt_s+=dt;
		       if(save_trig)
		       {
		       startWriteFile(file_writer,dt_s);
		        save_trig=0;
		       }
		       if(mode_rx.y==0)
		       {file_writer.close();newfile_flag=save_state=0;}
		       break;
		}
		
	       // if(dt_s>0.005)dt_s=0;

		//last_request = ros::Time::now();
		//ROS_INFO_STREAM("dT: " << dt);
		ros::spinOnce();

		loop_rate.sleep();
	}

	cout << "serial quit" << endl;

	return 0;
}