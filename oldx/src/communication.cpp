#include <ros/ros.h>
#include <serial/serial.h>
#include <serial/v8stdint.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

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

geometry_msgs::TwistStamped velocity;
void velocityCallback(const geometry_msgs::TwistStampedPtr& msg)
{
	float x = (*msg).twist.linear.x;
	float y = (*msg).twist.linear.y;
	velocity.twist.linear.x = sin(current_yaw_rad) * y + cos(current_yaw_rad) * x;
    velocity.twist.linear.y = cos(current_yaw_rad) * y - sin(current_yaw_rad) * x;
	velocity.twist.linear.z = (*msg).twist.linear.z;

	velocity.twist.angular.z = (*msg).twist.angular.z;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "communication");

	ros::NodeHandle nh;
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 1);
    ros::Subscriber velocity_sub = nh.subscribe("oldx/velocity", 1, velocityCallback);
	ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",2,imu_cb);
	
	ros::Rate loop_rate(100);

	while (ros::ok())
	{
        velocity_pub.publish(velocity);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}