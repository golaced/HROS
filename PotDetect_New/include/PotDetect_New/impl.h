/*
 * impl.h
 *
 *      Author: JYF
 */

#ifndef SRC_IMPL_H_
#define SRC_IMPL_H_

#define LIFE	1

#include "features.h"
#include <iostream>
#include <opencv2/opencv.hpp>

struct Target{
	int index;
	int life;
	Rect location;
};

struct queue_compare{
	bool operator()(const Target& t1, const Target& t2) const{
		int t1_area=t1.location.width*t1.location.height;
		int t2_area=t2.location.width*t2.location.height;
		return t1_area>t2_area;
	}
};

void draw_bbox(Mat& frame, vector<Target>& targets, int width);

void detect_bbox(Mat& src, vector<Target>& targets,  bool bCurFrame);

void detect_bbox_kcf(Mat& src, vector<Target>& targets);

bool is_new_target(Target& t, vector<Target>& targets, int& index);

bool target_detectable(Target& t, const Size& win);

void track_by_detect(vector<Target>& targets);

bool target_compare(Target& t1, Target& t2);

#endif /* SRC_IMPL_H_ */
