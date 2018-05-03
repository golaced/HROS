/*
 * extract_feat.h
 *
 *  Created on: 2017Äê7ÔÂ23ÈÕ
 *      Author: JYF
 */
/**
 * Canny
 * Hough circle
 * ORB feature match
 * HOG feature match
 * saliency detect
 */

#ifndef SRC_EXTRACT_FEAT_H_
#define SRC_EXTRACT_FEAT_H_
#define SA_FREQ	0
#define SA_FT	1
#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include "fhog.hpp"

using namespace std;
using namespace cv;

Mat feat_canny(Mat& src);

Mat feat_houghCircle(Mat& src);

void feat_hog_cv2(Mat& src, Size blockSize, Size cellSize, Size stride, vector<float>& hogFeat);

Mat feat_hog(Mat& src);

Mat feat_saliency(Mat& src, int mode=SA_FREQ);


#endif /* SRC_EXTRACT_FEAT_H_ */
