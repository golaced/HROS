#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void transform(Mat& src, Mat& dst);

void detectSaliency(Mat& src, Mat& dst);

void Normalize(Mat& src, Mat& dst);

float gamma(float x);

void RGBToLab(unsigned char * rgbImg, float * labImg);

void saliencyDetectFT(Mat& src, Mat& dst);
