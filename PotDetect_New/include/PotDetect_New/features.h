/*
 * features.h
 *
 *      Author: JYF
 */

#ifndef SRC_FEATURES_H_
#define SRC_FEATURES_H_

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
#include "util.h"

void feat_max_blur(Mat& src, Size ksize);

void feat_detect_saliency(Mat& src, int mode=SA_FREQ);

void get_logistic(Logistic& logi, const char* feat="hog");

float detect_logistic_hog(Mat& src, Logistic& classifier, int featLen);

float detect_logistic_hist(Mat& src, Logistic& classifier, int featLen);

float detect_logistic_pix(Mat& src, Logistic& classifier, int featLen);

float detect_logistic_bow(Mat& src, Logistic& classifier, int featLen);

void train_logistic_hog(int iter=1e2);

void train_logistic_hist(int iter=1e2);

void train_logistic_pix(int iter=1e3);

void train_logistic_bow(int iter=1e3);

#endif /* SRC_FEATURES_H_ */
