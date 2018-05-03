/*
 * features.h
 *
 *  Created on: 2017Äê7ÔÂ23ÈÕ
 *      Author: JYF
 */

#ifndef SRC_FEATURES_H_
#define SRC_FEATURES_H_

#define SCALED_SIZE	128
#define CELL_SIZE	16
#define BLOCK_SIZE	2*CELL_SIZE
#define STRIDE	32

#define TRAIN_SVM	0
#define TRAIN_LOG	1
/**
 * Hough circle detect
 * saliency detect
 * ORB match
 * SVM using HOG
 * Adaboost using HOG
 * KCF
 */
#include "extract_feat.h"
#include "Logistic.h"
#include <stdlib.h>

typedef Ptr<cv::ml::SVM> MySVM;

void feat_max_blur(Mat& src, Size ksize);

void feat_detect_canny(Mat& src);

void feat_detect_houghCircle(Mat& src);

Mat feat_detect_threshold(Mat& src);

void feat_detect_saliency(Mat& src, int mode=SA_FREQ);

MySVM get_svm();

void get_logistic(Logistic& logi);

int feat_train(int iter=1e2, int mode=TRAIN_LOG);

int feat_detect_svm(Mat& src, MySVM& svm);

float feat_detect_logistic(Mat& src, Logistic& logi);

void feat_train_svm(int iter=1e2);

void feat_train_logistic(int iter=1e2);




#endif /* SRC_FEATURES_H_ */
