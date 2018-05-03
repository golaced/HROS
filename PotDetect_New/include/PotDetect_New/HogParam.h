/*
 * HogParam.h
 *
 */

#ifndef SRC_HOGPARAM_H_
#define SRC_HOGPARAM_H_
#include <opencv2/opencv.hpp>

#define ID	2
struct HogParam{
	int blockSize;
	int cellSize;
	int stride;
	cv::Size winSize;
};


#endif /* SRC_HOGPARAM_H_ */
