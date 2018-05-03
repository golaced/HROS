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
#include <iostream>
#include <vector>
#include "util.h"
using namespace std;
using namespace cv;


#define HOG_SIZE	96
#define CELL_SIZE	8
#define BLOCK_SIZE	2*CELL_SIZE
#define STRIDE	16

#define TRAIN_SVM	0
#define TRAIN_LOG	1

#define BOW_NUM	4
#define HIST_SEG	16
#define BOW_SEG	32
#define HIST_SIZE	80
#define PIX_SIZE	40

void feat_hog_cv2(Mat& src, Size blockSize, Size cellSize, Size stride, vector<float>& hogFeat);

Mat feat_saliency(Mat& src, int mode=SA_FREQ);

void getHogFeature(Mat& image, HogParam& param, vector<float>& hogFeat, bool cvt);

void getMSHogFeature(Mat& image, vector<HogParam>& params, vector<float>& msHogFeat);

void getHistogramFeature(Mat& image, float*& feat, int featLen, int size, int seg, bool bNorm=true);

void getHistFeatureBoW(Mat& image, float*& feat, int featLen, int size, int seg, int bowNum);

#endif /* SRC_EXTRACT_FEAT_H_ */
