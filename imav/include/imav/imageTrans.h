/*
 * imageTrans.h
 *
 *  Created on: Sep 14, 2016
 *      Author: odroid
 */

#ifndef IMAGETRANS_H_
#define IMAGETRANS_H_

#ifdef WIN32
#include <windows.h>
#include <time.h>
#endif // WIN32


#include <opencv2/opencv.hpp>
#include <fstream>

void initPath();
void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle);
std::string get_time();

//void drawImage(cv::Mat &image);
std::string expand_user(std::string path);
void startWriteVideo(std::ifstream &video_num_read,
		cv::VideoWriter &video_writer);
void startWriteFile(std::ofstream &gps_writer);
#endif /* IMAGETRANS_H_ */
