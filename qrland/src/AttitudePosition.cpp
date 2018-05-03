#include "AttitudePosition.h"
#include "MarkerWorldCoornidate.h"
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;
using namespace aruco;

extern std::vector<MarkerWorld> CoordinateTable;
extern std::map<int, MarkerConfig> markermap;

//��������������
bool SortByDown(const float &p1, const float &p2) //��������������������������������vector����������������
{
    return p1 > p2; //��������
}

std::vector<cv::Mat> getR(float alpha_X, float alpha_Y, float alpha_Z)
{
    Mat R_X = Mat::eye(3, 3, CV_32FC1);
    Mat R_Y = Mat::eye(3, 3, CV_32FC1);
    Mat R_Z = Mat::eye(3, 3, CV_32FC1);

    alpha_X /= 57.3;
    alpha_Y /= 57.3;
    alpha_Z /= 57.3;

    R_X.at<float>(1, 1) = cos(alpha_X);
    R_X.at<float>(1, 2) = sin(alpha_X);
    R_X.at<float>(2, 1) = -sin(alpha_X);
    R_X.at<float>(2, 2) = cos(alpha_X);

    R_Y.at<float>(0, 0) = cos(alpha_Y);
    R_Y.at<float>(0, 2) = -sin(alpha_Y);
    R_Y.at<float>(2, 0) = sin(alpha_Y);
    R_Y.at<float>(2, 2) = cos(alpha_Y);

    R_Z.at<float>(0, 0) = cos(alpha_Z);
    R_Z.at<float>(0, 1) = sin(alpha_Z);
    R_Z.at<float>(1, 0) = -sin(alpha_Z);
    R_Z.at<float>(1, 1) = cos(alpha_Z);

    vector<Mat> dst;
    dst.push_back(R_X);
    dst.push_back(R_Y);
    dst.push_back(R_Z);

    return dst;

}

void getCameraPos(cv::Mat Rvec, cv::Mat Tvec, cv::Point3f &pos)
{
    Mat Rot(3, 3, CV_32FC1);
    Rodrigues(Rvec, Rot);

    Rot = Rot.t();  // rotation of inverse
    Mat pos_camera = -Rot * Tvec; // translation of inverse

    pos.x = pos_camera.at<float>(0, 0);
    pos.y = pos_camera.at<float>(1, 0);
    pos.z = pos_camera.at<float>(2, 0);
}


#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
void getCameraPos1(cv::Mat Rvec, cv::Mat Tvec, cv::Point3f &pos)
{
    vector<Eigen::Vector3d> landmarks_pointXYZ;
    Mat Rot(3, 3, CV_32FC1);
    Rodrigues(Rvec, Rot);
    Eigen::Matrix3d eigen_r;
//    Eigen::Vector3d eigen_t;
    cv2eigen(Rot,eigen_r);
//    cv2eigen(Tvec,eigen_t);
    Rot = Rot.t();  // rotation of inverse
    Mat pos_camera = -Rot * Tvec*100; // translation of inverse
    //landmarks_pointXYZ.push_back(eigen_r +eigen_t);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(eigen_r);
    T = angle.inverse();
    Eigen::Matrix<double,3,1> t;
    cv::cv2eigen(Tvec, t);
    t = -1 * angle.inverse().matrix() *t;
    T(0, 3) = t(0);
    T(1, 3) = t(1);
    T(2, 3) = t(2);
    pos.x = t(0);
    pos.y = -t(1);
    pos.z = t(2);
//cout<<t<<endl;
//    pos.x = pos_camera.at<float>(0, 0);
//    pos.y = -pos_camera.at<float>(1, 0);
//    pos.z = pos_camera.at<float>(2, 0);

}

void getCameraPosWithMarkers(std::vector<aruco::Marker> Markers,
                             cv::Point3f &pos_camera, Attitude &atti_camera, int flag /*= 0*/)
{
    Point3f pos_world(0, 0, 0);
    Attitude atti_t;
    float s1=((markermap.find(7))->second).boardsize;
 
    pos_camera.x=pos_camera.y=pos_camera.z=0;
    atti_camera.Pit = 0;
    atti_camera.Rol = 0;
    atti_camera.Yaw = 0;
    
    
    switch (flag)
    {
    case 0:
    {
        //cout<< Markers.size()<<endl;
        for (unsigned int i = 0; i < Markers.size(); i++)
        {   float s2=((markermap.find(Markers[i].id))->second).boardsize;
            getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);
            pos_camera += ((markermap.find(Markers[i].id))->second).coordinate*1 + pos_world*s2/s1;
            getAttitude(Markers[i], atti_t);
            atti_camera.Pit+=atti_t.Pit;
            atti_camera.Rol+=atti_t.Rol;
            atti_camera.Yaw+=atti_t.Yaw;
        }
        if (Markers.size() > 0)
        {
            pos_camera.x = pos_camera.x / Markers.size();
            pos_camera.y = pos_camera.y / Markers.size();
            pos_camera.z = pos_camera.z / Markers.size();

            atti_camera.Pit/= Markers.size();
            atti_camera.Rol/= Markers.size();
            atti_camera.Yaw/= Markers.size();
        }

        break;
    }
    case 1:
    {
        float dis = 0;
        float dismin = 100000;

        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);

            dis = sqrt(pos_world.x * pos_world.x + pos_world.y * pos_world.y);

            if (dis < dismin)
            {
                dismin = dis;
                pos_camera = CoordinateTable[Markers[i].id - 1].coordinate
                        + pos_world;
            }
        }
        break;
    }
    case 2:
    {
        std::vector<float> vec_x, vec_y, vec_yaw;
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);
            getAttitude(Markers[i], atti_t);
            pos_camera = CoordinateTable[Markers[i].id - 1].coordinate
                    + pos_world;
            vec_x.push_back(pos_camera.x);
            vec_y.push_back(pos_camera.y);
            vec_yaw.push_back(atti_t.Yaw);
        }
        if (vec_x.size() > 0)
        {
            sort(vec_x.begin(), vec_x.end(), SortByDown);
            sort(vec_y.begin(), vec_y.end(), SortByDown);
            sort(vec_yaw.begin(), vec_yaw.end(), SortByDown);

            pos_camera.x = vec_x[vec_x.size() / 2];
            pos_camera.y = vec_y[vec_y.size() / 2];
            //atti_camera.Yaw = vec_yaw[vec_yaw.size() / 2];
        }

        break;
    }
    case 3:
    {
        std::vector<float> vec_x, vec_y, vec_z, vec_pit, vec_rol, vec_yaw;
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world); //camera to local marker o dis
            getAttitude(Markers[i], atti_t);
            //pos_camera = CoordinateTable[Markers[i].id - 1].coordinate + pos_world;//camera to global dis
            pos_camera.x = pos_world.x
                    + CoordinateTable[Markers[i].id].coordinate.x;
            pos_camera.y = pos_world.y
                    + CoordinateTable[Markers[i].id].coordinate.y;
            pos_camera.z = pos_world.z;
            vec_x.push_back(pos_camera.x);
            vec_y.push_back(pos_camera.y);
            vec_z.push_back(pos_camera.z);
            vec_pit.push_back(atti_t.Pit);
            vec_rol.push_back(atti_t.Rol);
            vec_yaw.push_back(atti_t.Yaw);
        }
        if (vec_x.size() > 3)
        {
            sort(vec_x.begin(), vec_x.end(), SortByDown);
            sort(vec_y.begin(), vec_y.end(), SortByDown);
            sort(vec_z.begin(), vec_z.end(), SortByDown);
            sort(vec_pit.begin(), vec_pit.end(), SortByDown);
            sort(vec_rol.begin(), vec_rol.end(), SortByDown);
            sort(vec_yaw.begin(), vec_yaw.end(), SortByDown);
            float temp[6] = { 0 };
            for (unsigned int i = 1; i < vec_x.size() - 1; i++)
                temp[0] += vec_x[i];
            for (unsigned int i = 1; i < vec_y.size() - 1; i++)
                temp[1] += vec_y[i];
            for (unsigned int i = 1; i < vec_z.size() - 1; i++)
                temp[2] += vec_z[i];
            for (unsigned int i = 1; i < vec_pit.size() - 1; i++)
                temp[3] += vec_pit[i];
            for (unsigned int i = 1; i < vec_rol.size() - 1; i++)
                temp[4] += vec_rol[i];
            for (unsigned int i = 1; i < vec_yaw.size() - 1; i++)
                temp[5] += vec_yaw[i];
            pos_camera.x = temp[0] / (vec_x.size() - 2);
            pos_camera.y = temp[1] / (vec_y.size() - 2);
            pos_camera.z = temp[2] / (vec_z.size() - 2);
            atti_camera.Pit = temp[3] / (vec_pit.size() - 2);
            atti_camera.Rol = temp[4] / (vec_rol.size() - 2);
            atti_camera.Yaw = temp[5] / (vec_yaw.size() - 2);
        }
        else if (vec_x.size() > 0)
        {
            float temp[6] = { 0 };
            for (unsigned int i = 0; i < vec_x.size(); i++)
                temp[0] += vec_x[i];
            for (unsigned int i = 0; i < vec_y.size(); i++)
                temp[1] += vec_y[i];
            for (unsigned int i = 0; i < vec_z.size(); i++)
                temp[2] += vec_z[i];
            for (unsigned int i = 0; i < vec_pit.size(); i++)
                temp[3] += vec_pit[i];
            for (unsigned int i = 0; i < vec_rol.size(); i++)
                temp[4] += vec_rol[i];
            for (unsigned int i = 0; i < vec_yaw.size(); i++)
                temp[5] += vec_yaw[i];
            pos_camera.x = temp[0] / vec_x.size();
            pos_camera.y = temp[1] / vec_y.size();
            pos_camera.z = temp[2] / vec_z.size();
            atti_camera.Pit = temp[3] / vec_pit.size();
            atti_camera.Rol = temp[4] / vec_rol.size();
            atti_camera.Yaw = temp[5] / vec_yaw.size();
        }

        break;
    }

    default:
        break;
    }

}

void getAttitude(aruco::Marker marker, Attitude &attitude)
{
    double pos[3] = { 0 };
    double ori[4] = { 0 };

    double q0, q1, q2, q3;

    marker.OgreGetPoseParameters(pos, ori);
    pos[0] = -pos[0];
    pos[1] = -pos[1];

    q0 = ori[0];
    q1 = ori[1];
    q2 = ori[2];
    q3 = ori[3];

    attitude.Pit_rad = atan2(2 * (q0 * q1 + q2 * q3), -1 + 2 * (q1 * q1 + q2 * q2));
    attitude.Rol_rad = asin(2 * (q1 * q3 - q0 * q2));
    attitude.Yaw_rad = -atan2(2 * (-q1 * q2 - q0 * q3), 1 - 2 * (q0 * q0 + q1 * q1));

    attitude.Pit = attitude.Pit_rad * 57.3f;
    attitude.Rol = attitude.Rol_rad * 57.3f;
    attitude.Yaw = attitude.Yaw_rad * 57.3f;
}

cv::Scalar getLandCenter(std::vector< aruco::Marker > Markers)
{
    float x = 0;
    float y = 0;
    float z = 0;

    float x_origin_img = 0;
    float y_origin_img = 0;
    float z_origin_img = 0;

    float x_origin_phy = ((markermap.find(7))->second).coordinate.x;
    float y_origin_phy = ((markermap.find(7))->second).coordinate.y;
    float z_origin_phy = ((markermap.find(7))->second).coordinate.z;

    float size_phy;

    Point center(0, 0);
    Point2f center_diff_meter(0, 0);
    Attitude atti_t;
    float x_temp = 0;
    float y_temp = 0;

    for (size_t i = 0; i < Markers.size(); i++)
    {
        getAttitude(Markers[i], atti_t);
        x = ((markermap.find(Markers[i].id))->second).coordinate.x;
        y = ((markermap.find(Markers[i].id))->second).coordinate.y;
        z = ((markermap.find(Markers[i].id))->second).coordinate.z;
        size_phy = ((markermap.find(Markers[i].id))->second).boardsize;

        x_temp = (x_origin_phy - x) / size_phy*Markers[i].getPerimeter() / 4;
        y_temp = (-y_origin_phy + y) / size_phy*Markers[i].getPerimeter() / 4;

        x_origin_img = x_temp*cos(atti_t.Yaw_rad) + y_temp*sin(atti_t.Yaw_rad) + Markers[i].getCenter().x;
        y_origin_img = -x_temp*sin(atti_t.Yaw_rad) + y_temp*cos(atti_t.Yaw_rad) + Markers[i].getCenter().y;

        center += Point(x_origin_img, y_origin_img);
        center_diff_meter += Point2f((x_origin_img - 160)*size_phy / (Markers[i].getPerimeter() / 4),
       (-y_origin_img + 120)*size_phy / (Markers[i].getPerimeter() / 4));
    }
    if (Markers.size()>0)
    {
        center.x /= Markers.size();
        center.y /= Markers.size();
        center_diff_meter.x /= Markers.size();
        center_diff_meter.y /= Markers.size();
    }

    return Scalar(center.x, center.y, center_diff_meter.x, center_diff_meter.y);
}
