/*****************************
Copyright 2011 Rafael Mu��oz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list
of conditions and the following disclaimer in the documentation and/or other materials
provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Mu��oz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Mu��oz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Mu��oz Salinas.
********************************/

#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "AttitudePosition.h"
#include "MarkerWorldCoornidate.h"
#include "ColorDetect.h"
#include <geometry_msgs/Vector3.h> //Èý¸öfloat64
using namespace std;
using namespace cv;
using namespace aruco;

#define TRANS_WORLD 1
#define WRITE_VIDEO  1
#define SIZE_METER 0
#define USE_PIXLE_LAND 1

extern std::vector<MarkerWorld> CoordinateTable;
cv::Point3f coordinate_camera(0, 0, 0);
Attitude atti_camera;
std::vector< aruco::Marker > Markers;
std::map<int, MarkerConfig> markermap;
std::vector<cv::Rect> objectRects;
std::vector<cv::Point> objectCenters;
std::vector<int> objectLocate;
cv::Scalar markerorigin_img(0,0);
cv::Size m_size(0,0);
cv::Point image_center(0,0);

int video_num = 0;
std::ifstream video_num_read;
std::ofstream video_num_write;
std::string video_num_path(
        "/home/odroid/Videos/video_num.txt");

void startWriteVideo(std::ifstream &video_num_read,
        cv::VideoWriter &video_writer, cv::Size m_size, int fps)
{
    video_num_read.open(video_num_path.c_str());
    video_num_read >> video_num;
    video_num_read.close();

    cout << "video_num:"<<video_num << endl;

    video_num_write.open(video_num_path.c_str());
    video_num_write << (video_num + 1);
    video_num_write.close();

    if (video_writer.isOpened())
    {
        video_writer.release();
    }

    std::stringstream ss;
    string video_name;

    ss << video_num;
    ss >> video_name;
    video_name += ".avi";

    video_writer.open(
            "/home/odroid/Videos/" + video_name,
            CV_FOURCC('D', 'I', 'V', 'X'), fps, m_size);

}


void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle)
{
    CV_Assert(!inputIm.empty());
    Mat inputImg;
    inputIm.copyTo(inputImg);
    float radian = (float) (angle / 180.0 * CV_PI);
    int uniSize = (int) (max(inputImg.cols, inputImg.rows) * 1.414);
    int dx = (int) (uniSize - inputImg.cols) / 2;
    int dy = (int) (uniSize - inputImg.rows) / 2;
    copyMakeBorder(inputImg, tempImg, dy, dy, dx, dx, BORDER_CONSTANT);
    Point2f center((float) (tempImg.cols / 2), (float) (tempImg.rows / 2));
    Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);
    warpAffine(tempImg, tempImg, affine_matrix, tempImg.size());
    float sinVal = fabs(sin(radian));
    float cosVal = fabs(cos(radian));
    Size targetSize((int) (inputImg.cols * cosVal + inputImg.rows * sinVal),
            (int) (inputImg.cols * sinVal + inputImg.rows * cosVal));
    int x = (tempImg.cols - targetSize.width) / 2;
    int y = (tempImg.rows - targetSize.height) / 2;
    Rect rect(x, y, targetSize.width, targetSize.height);
    tempImg = Mat(tempImg, rect);
}

bool loadConfig(std::string configfile_path, std::map<int, MarkerConfig> &markermap)
{
	std::ifstream configFileIn;
	configFileIn.open(configfile_path);

	if (!configFileIn.is_open())
	{
		return false;
	}

	char temp[255];
	int id = -1;
    float boardsize = -1;
    float x = -1;
    float y = -1;
    float z = -1;
    float factor = 0.234;
configFileIn.getline(temp, 255);
configFileIn.getline(temp, 255);
sscanf(temp, "%f", &factor);
	while (!configFileIn.eof())
	{
		configFileIn.getline(temp, 255);
        sscanf(temp, "%d %f %f %f %f", &id, &boardsize, &x, &y, &z);
        
		if (id>0)
		{
            cout<<"id"<<id<<" factor:"<<factor<<" boardsize"<<boardsize<<endl;
#if SIZE_METER
            markermap.insert(make_pair(id, MarkerConfig(id, factor*boardsize/100., factor*x/100., factor*y/100., z/100.)));
#else
            markermap.insert(make_pair(id, MarkerConfig(id, factor*boardsize, factor*x, factor*y, z)));
#endif

		}
	}

	return true;
}


std_msgs::ByteMultiArray data_to_publish;

void format_data_to_send()
{
    unsigned char data_to_send[50];
    int Length = 0;
	int _cnt = 0, i = 0, sum = 0;
    int Locate[5] = {0,0,0,0,0};

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAF;
	data_to_send[_cnt++] = 0x21;
	data_to_send[_cnt++] = 0;

	data_to_send[_cnt++] = Markers.size();
	data_to_send[_cnt++] = int(coordinate_camera.x) >> 8;
	data_to_send[_cnt++] = int(coordinate_camera.x) % 256;
	data_to_send[_cnt++] = int(coordinate_camera.y) >> 8;
	data_to_send[_cnt++] = int(coordinate_camera.y) % 256;
	data_to_send[_cnt++] = int(coordinate_camera.z) >> 8;
	data_to_send[_cnt++] = int(coordinate_camera.z) % 256;
	data_to_send[_cnt++] = int(atti_camera.Pit) >> 8;
	data_to_send[_cnt++] = int(atti_camera.Pit) % 256;
	data_to_send[_cnt++] = int(atti_camera.Rol) >> 8;
	data_to_send[_cnt++] = int(atti_camera.Rol) % 256;
	data_to_send[_cnt++] = int(atti_camera.Yaw) >> 8;
	data_to_send[_cnt++] = int(atti_camera.Yaw) % 256;


    if(Markers.size() == 0)
    {
        data_to_send[_cnt++] = 0;
        data_to_send[_cnt++] = 0;
        data_to_send[_cnt++] = 0;
        data_to_send[_cnt++] = 0;
	    data_to_send[_cnt++] = 0;
        data_to_send[_cnt++] = 0;
    }
    else
    {
        data_to_send[_cnt++] = int(markerorigin_img.val[0]) >> 8;
        data_to_send[_cnt++] = int(markerorigin_img.val[0]) % 256;
        data_to_send[_cnt++] = int(markerorigin_img.val[1]) >> 8;
        data_to_send[_cnt++] = int(markerorigin_img.val[1]) % 256;
    }

    for(size_t i=0;i<objectLocate.size();i++)
    {
        Locate[objectLocate[i]] = bool(objectLocate[i]);
    }

    data_to_send[_cnt++] = Locate[1];
    data_to_send[_cnt++] = Locate[2];
    data_to_send[_cnt++] = Locate[3];
    data_to_send[_cnt++] = Locate[4];
    
    data_to_send[_cnt++] = int(markerorigin_img.val[2]) >> 8;
    data_to_send[_cnt++] = int(markerorigin_img.val[2]) % 256;
    data_to_send[_cnt++] = int(markerorigin_img.val[3]) >> 8;
    data_to_send[_cnt++] = int(markerorigin_img.val[3]) % 256;

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Length = _cnt;

	data_to_publish.data.clear();
	for (int i = 0; i < _cnt; i++)
	{
		data_to_publish.data.push_back(data_to_send[i]);
	}
}


Mat Filter_aruco(Mat in,int user_threshold)
{
  Mat I_filtered,I;
  if(user_threshold > 0)
  {


  cv::cvtColor(in, I, COLOR_BGR2GRAY);
  //cv::GaussianBlur(I, I_filtered, cv::Size(0,0),2);

  // Weights
  //cv::addWeighted(I, 2.5, I_filtered, -1.5, 0, I_filtered);

  // Equalize histogram
  cv::equalizeHist(I,I_filtered);

  // Treshold
  cv::threshold(I_filtered,I_filtered,user_threshold,0,3);
 }else{
      in.copyTo(I_filtered);
       cv::cvtColor(I_filtered, I_filtered, COLOR_BGR2GRAY);
   }
  return I_filtered;
}

//¶©ÔÄUWBÎ»ÖÃ»Øµ÷º¯Êý
geometry_msgs::Vector3 mode;
void get_mode_cb(const geometry_msgs::Vector3::ConstPtr &msg) 
{
    mode = *msg;
    cout<<"mode_rx"<<mode<<endl;
}


int main(int argc, char** argv)
{    static int cnt_test;
    ros::init(argc, argv, "qrland");
    ros::NodeHandle nh;
    ros::Publisher qrland_info_pub = nh.advertise<std_msgs::ByteMultiArray>("oldx/oldx_send", 1);
    ros::Subscriber get_mode_sub = nh.subscribe("mode_fc", 1, get_mode_cb);//UWB
	try
	{
#define CAP_WIDTH_320 1

#if CAP_WIDTH_320
        string cameraParamFileName("/home/odroid/catkin_ws/src/qrland/CAM_320.xml");
        m_size = cv::Size(320, 240);
        image_center = Point(160,120);
#else
        string cameraParamFileName("/home/odroid/catkin_ws/src/qrland/PS3_640.yml");
        m_size = cv::Size(640, 480);
        image_center = Point(320,240);
#endif
		string imagename("SizeQR.png");
		//string imagename("3.bmp");

        string configpath("/home/odroid/catkin_ws/src/qrland/config.ini");
		bool bloadOK = loadConfig(configpath, markermap);
		if (!bloadOK)
		{
			cout << "markers config load failed!" << endl;
			return -1;
		}

		initCoordinateTable(CoordinateTable);

		aruco::CameraParameters CamParam;
		MarkerDetector MDetector;
        ColorDetect colorDetector;

		// read the input image
		cv::Mat InImage;
        cv::Mat SrcImage;
        cv::Mat src_temp;
		// try opening first as video
		VideoCapture cap(0);
#if CAP_WIDTH_320
  cap.set(CV_CAP_PROP_FRAME_WIDTH , 320);
cap.set(CV_CAP_PROP_FRAME_HEIGHT , 240);
#else
  cap.set(CV_CAP_PROP_FRAME_WIDTH , 640);
cap.set(CV_CAP_PROP_FRAME_HEIGHT , 480);

#endif
		cap >> InImage;
		//resize(InImage, InImage, m_size);
                resize(Filter_aruco(InImage,0), InImage, m_size);

		//read camera parameters if specifed
		CamParam.readFromXMLFile(cameraParamFileName);

		cout << CamParam.CameraMatrix << endl;
		cout << CamParam.Distorsion << endl;
		cout << CamParam.CamSize << endl;

		cv::String windowTitle("thes");
		cv::namedWindow(windowTitle, 1);

		int p1 = 7;
		int p2 = 7;
		int t_p_range = 2;
		int perimeter_th = 120;
		createTrackbar("p1", "thes", &p1, 101);
		createTrackbar("p2", "thes", &p2, 50);
		createTrackbar("range", "thes", &t_p_range, 31);
		createTrackbar("perimeter_th", "thes", &perimeter_th, 200);


	        //MDetector.setThresholdParamRange(t_p_range);
	        //MDetector.setThresholdParams(p1, p2);
	        MDetector.setCornerRefinementMethod(MDetector.SUBPIX ,1);//=1;//.minCornerDistance()
		MDetector.setThresholdMethod(MDetector.ADPT_THRES);
		MDetector.setMinMaxSize(0.03,0.8);
		ostringstream ostr_pos;
		ostringstream ostr_angle;

#if WRITE_VIDEO
        int fps = 30;
        cv::VideoWriter video_writer;
        cv::VideoWriter videowriter_process;
#endif

	ros::Rate loop_rate(500);

	while (ros::ok())
	{
			p1 = p1 / 2 * 2 + 1;
			p2 = p2 / 2 * 2 + 1;
			//MDetector.setThresholdParamRange(t_p_range);
			//MDetector.setThresholdParams(p1, p2);
         
            cout<<"test:"<<cnt_test++<<endl;
            cap >> src_temp;
            warpFfine(src_temp, InImage, 359);

	    resize(InImage, InImage, m_size);
            InImage.copyTo(SrcImage);

	    // Ok, let's detect
	    MDetector.detect(InImage, Markers, CamParam,((markermap.find(7))->second).boardsize);
           
            float markersize = 0;
            for (auto it = Markers.begin(); it != Markers.end(); )
            {
                //cout<<it->getPerimeter()<<endl;
   	         
                if ((it->id>0)&&(it->getPerimeter())<perimeter_th)
                {
                    Markers.erase(it);
                    continue;
                }

                markersize = ((markermap.find(it->id))->second).boardsize;
		
                if(markersize<=0)
                {
                    Markers.erase(it);
                }
                else
                {
                    it->draw(InImage, Scalar(0, 0, 255), 2);
                    //it->calculateExtrinsics(markersize, CamParam, false);
                    it++;
                }

            }
            //cout<< Markers.size()<<endl;
	    getCameraPosWithMarkers(Markers, coordinate_camera,atti_camera, 0);
#if USE_PIXLE_LAND
            markerorigin_img = getLandCenter(Markers);

            Point center_pt(int(markerorigin_img.val[0]), int(markerorigin_img.val[1]));
            cv::circle(InImage,center_pt, 5, Scalar(255, 0, 255), -1);
	    ostr_pos.clear();
	    ostr_pos.str("");
	    ostr_pos << center_pt;

	    putText(InImage, ostr_pos.str(), center_pt+Point(10,10), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

	    ostr_pos.clear();
	    ostr_pos.str("");
	    ostr_pos << "dis="<<Point(int(markerorigin_img.val[2]),int(markerorigin_img.val[3]));

	    putText(InImage, ostr_pos.str(), Point(180, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);
#endif
            //colorDetector.setImage(SrcImage);
           // objectLocate = colorDetector.getColorRegionLocate(ColorDetect::RED);

			format_data_to_send();
             qrland_info_pub.publish(data_to_publish);

			ostr_pos.clear();
			ostr_pos.str("");
#if SIZE_METER
            coordinate_camera.x*=100;
            coordinate_camera.y*=100;
            coordinate_camera.z*=100;
#endif
            ostr_pos << Point3i((int)coordinate_camera.x,(int)coordinate_camera.y ,(int)coordinate_camera.z);

	    putText(InImage, ostr_pos.str(), Point(15, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

            objectRects = colorDetector.getRects();
            objectCenters = colorDetector.getRectCenters();

            for(size_t i = 0;i<objectRects.size();i++)
            {
                cv::rectangle(InImage,objectRects[i],Scalar(255,0,0),2);
                cv::circle(InImage,objectCenters[i], 3, Scalar(255, 0, 0), -1);
            }

			// show input with augmented information
	   cv::imshow("in", InImage);
	   
	   static char state_video;
	   cout<<"mode:"<<mode<<endl;
	   switch(state_video)
	   {
	    case 0:
	      if(mode.z==1)
	       {
	        state_video=1;
	        startWriteVideo(video_num_read, video_writer, m_size, fps);
                startWriteVideo(video_num_read, videowriter_process, m_size, fps);
	       }
	    break;
	    case 1:
	    #if WRITE_VIDEO
            video_writer << SrcImage;
            videowriter_process<<InImage;
	    #endif
	    if(mode.z==0)
	       {
	        //video_writer.release();
                //videowriter_process.release();
	        state_video=0;
	       }
	    break;
	   }

			// show also the internal image resulting from the threshold operation
          //  cv::imshow("thes", MDetector.getThresholdedImage());

            char c_key = cv::waitKey(5);
	    if (c_key == 27) // wait for key to be pressed
	    {
		break;
	    }
	    ros::spinOnce();
	    loop_rate.sleep();
	}

#if WRITE_VIDEO
            video_writer.release();
            videowriter_process.release();
#endif
		return 0;
}
	catch (std::exception &ex)
	{
		cout << "Exception :" << ex.what() << endl;
	}
}
