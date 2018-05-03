/*
 * Logistic.h
 *
 *  Created on: 2017Äê7ÔÂ27ÈÕ
 *      Author: JYF
 */

#ifndef LOGISTIC_H_
#define LOGISTIC_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <fstream>
using namespace std;
using namespace cv;

class Logistic {
public:
	Logistic();
	Logistic(int featLen);
	virtual ~Logistic();
public:
	float lr;
	float* params;
	int featLen;
	int minibatch;
	int step;
	float gamma;

private:
	vector<int> ind;
	inline void init_param(){
		for(int i=0;i<featLen;++i){	params[i]=2.0*rand()/RAND_MAX-1;	}
	}
	void shuffle();

public:
	void train(Mat& sampleMat, Mat& labelMat, int max_iter=1e2, float min_err=1e-6);
	float predict(Mat& featMat);
	int classify(Mat& featMat);
  inline	void setFeatLen(int len){
  	featLen=len;
  	if(params==NULL){	params=new float[featLen];	}
  }
	void save(const char* file_name);
	void load(const char* file_name);
};

#endif /* LOGISTIC_H_ */
