/*
 * landing.h
 *
 *  Created on: 2017??8??14??
 *      Author: JYF
 */

#ifndef SRC_LANDING_H_
#define SRC_LANDING_H_

#define MAX_SIZE	640
#define MIN_AREA	25
#define MAX_PAD	10
#define TEMP_W	32
#define	TEMP_H	37

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
using namespace std;
using namespace cv;

typedef unsigned int uint;

void createSamples(string& seed, string& dir, int N=20);

Mat getHSVImage(const Mat& src, const char* chn="s");

Mat alignImage(const Mat& src, int size);

void getContours(Mat& biImg, vector<Rect>& contours);

void createTemplate(const string& file_list, const string& temp_file);

Mat loadTemplate(const string& temp_file);

Mat ToTemplate(Mat& src);

int KNN(Mat& img, Mat& _tmpl, vector<Rect>& contours);

#endif /* SRC_LANDING_H_ */
