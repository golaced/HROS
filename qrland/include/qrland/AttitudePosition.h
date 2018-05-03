#ifndef __AttitudePosition_H__
#define __AttitudePosition_H__

#include <iostream>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <sstream>
#include <opencv2/opencv.hpp>


class Attitude
{
public:
    Attitude(){ Pit = 0, Yaw = 0, Rol = 0, Pit_rad = 0, Yaw_rad = 0, Rol_rad = 0; }
    ~Attitude(){};

    float Pit;
    float Yaw;
    float Rol;
    float Pit_rad;
    float Yaw_rad;
    float Rol_rad;

private:

};

std::vector<cv::Mat> getR(float alpha_X, float alpha_Y, float alpha_Z);
void getCameraPos(cv::Mat Rvec, cv::Mat Tvec, cv::Point3f &pos);
void getCameraPosWithMarkers(std::vector< aruco::Marker > Markers, cv::Point3f &pos_camera, Attitude &atti_camera, int flag = 0);
void getAttitude(aruco::Marker marker, Attitude &attitude);
cv::Scalar getLandCenter(std::vector< aruco::Marker > Markers);

#endif // __AttitudePosition_H__
