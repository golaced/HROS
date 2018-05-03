#include "PotDetect/ImageFetcher.h"
#include <pthread.h>

bool next_frame;
bool bFirstArrived;

ros::Publisher inform_pub;

struct arg_struct{
	int argc;
	char** argv;
	ros::NodeHandle* pHandle;
};

Mat read_frame(VideoCapture& vcap){
	Mat frame;
	bool ok=vcap.read(frame);
	if(not ok){	return Mat();	}
	return frame;
}

void read_config(string cfg_name, vector<string>& names){

}

void string2msg(const char* info, std_msgs::String& msg){
	stringstream ss;
	ss<<info;
	msg.data=ss.str();
}

void getFeedbackCallback(const std_msgs::String::ConstPtr& msg){
	const char* data=msg->data.c_str();
	if(strcmp(data, "ok")==0){
		//cout<<"received data: "<<data<<endl;
		next_frame=true;
		bFirstArrived=true;
	}
	else if(strcmp(data, "next")==0){
		std_msgs::String inform_msg;
		string2msg("track", inform_msg);
		inform_pub.publish(inform_msg);	
	}
	else if(strcmp(data, "quit")==0){
		next_frame=false;
	}
	
}

void* feedback_thread(void* arg){
	arg_struct* args=(arg_struct*)arg;
	ros::init(args->argc, args->argv, "ImageFetcher");
	ros::NodeHandle* pHandle=args->pHandle;
	ros::Subscriber feedback_sub=pHandle->subscribe("feedback_info", 500, getFeedbackCallback);
	ros::spin();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ImageFetcher");
	ros::NodeHandle n_handle;
	image_transport::ImageTransport it(n_handle);
	image_transport::Publisher image_pub=it.advertise("frame", 1);
	inform_pub=n_handle.advertise<std_msgs::String>("inform",500);
	
	sensor_msgs::ImagePtr image_msg;
	std_msgs::String info_msg;

	Mat frame;
	char key=0;
	bool go_on;
	bool first_frame;
	arg_struct args;
	pthread_t id_feedback;
	string video_path="/home/odroid/catkin_ws_new/src/PotDetect/pot.avi";

	VideoCapture vcap(0);
	int fps=vcap.get(CV_CAP_PROP_FPS);
	cout<<"fps: "<<fps<<endl;
	if(!vcap.isOpened()){
		ROS_INFO("open image device failed\n");  
        return 1; 
	}
	
	ros::Rate loop_rate(fps);
	first_frame=true;
	next_frame=false;
	go_on=true;
	bFirstArrived=false;
	args.argc=argc;
	args.argv=argv;
	args.pHandle=&n_handle;
	int res=pthread_create(&id_feedback, NULL, feedback_thread, (void*)(&args));
	if (res != 0){
        perror("Thread creation failed!");
        exit(0);
    }
	
	int up=6;
	cout<<"start"<<endl;
	while(ros::ok()){	
		if(first_frame or next_frame or not bFirstArrived){
			if(first_frame){	first_frame=false;	}
			if(not bFirstArrived){	cout<<"continue to send"<<endl; 	}
			go_on=vcap.read(frame);	
			next_frame=false;
			if(not go_on){	
				break;	
			}
			string2msg("ok",info_msg);
			inform_pub.publish(info_msg);
			image_msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
			image_pub.publish(image_msg);
		}
		loop_rate.sleep();	
		ros::spinOnce();		
	}
	/*
	if(go_on){
		ros::init(argc, argv, "ImageFetcher");
		n_handle=ros::NodeHandle();
		inform_pub=n_handle.advertise<std_msgs::String>("frame_info",1000);
	}
	*/
	string2msg("quit",info_msg);
	inform_pub.publish(info_msg);
	cout<<"All frames processed"<<endl;
	loop_rate=ros::Rate(10);
	loop_rate.sleep();
	ros::shutdown();
	return 0;
}

