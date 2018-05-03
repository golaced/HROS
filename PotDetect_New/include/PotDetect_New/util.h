/*
 * util.h
 *
 *  Created on: 2017Äê8ÔÂ7ÈÕ
 *      Author: JYF
 */

#ifndef SRC_UTIL_H_
#define SRC_UTIL_H_
#include <iostream>
#include <fstream>
#include <vector>
#include <assert.h>
#include "HogParam.h"
using namespace std;

void readLabelFile(string label_file, vector<string>& file_names, vector<int>& labels);

void readParamsFromFile(string param_file, vector<HogParam>& params);

int calcFeatLen(vector<HogParam>& params);

void printParams(vector<HogParam>& params);

template<typename T>
void Normalize(T* data, int len, T min, T max){
	T data_min=(T)1e6;
	T data_max=-data_min;
	for(int i=0;i<len;++i){
		if(data[i]<data_min){	data_min=data[i];	}
		if(data[i]>data_max){	data_max=data[i];	}
	}
	data_max-=data_min;
	for(int i=0;i<len;++i){
		data[i]=(data[i]-data_min)/data_max;
	}
}

#endif /* SRC_UTIL_H_ */
