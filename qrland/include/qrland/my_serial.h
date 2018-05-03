#ifndef MY_SERIAL_H_
#define MY_SERIAL_H_
#include "serial/serial.h"
#include <aruco/aruco.h>
#include "AttitudePosition.h"
#include "MarkerWorldCoornidate.h"

extern cv::Point3f coordinate_camera;
extern Attitude atti_camera;
extern std::vector< aruco::Marker > Markers;
extern std::vector<int> objectLocate;
extern cv::Scalar markerorigin_img;
extern cv::Point image_center;

void serialSent();

#endif //MY_SERIAL_H_
