/*
 * impl.cpp
 *
 *      @Author: JYF
 */

#include "PotDetect_New/impl.h"
#include <math.h>

#define PAD_LT 1
#define PAD_RB 5

#define NEAR_METRIC 100

void draw_bbox(Mat &frame, vector<Target> &targets, int width)
{
	if (targets.size() == 0)
	{
		//cout<<"no target detected"<<endl;
		return;
	}
	Scalar s = Scalar(0, 255, 255);
	if (frame.channels() == 1)
	{
		s = 255;
	}
	for (size_t i = 0; i < targets.size(); ++i)
		rectangle(frame, targets[i].location, s, width);
}

void detect_bbox(Mat &src, vector<Target> &targets, bool bCurFrame)
{
	//one frame
	Mat scaled;
	int scale = 4;
	resize(src, scaled, Size(src.cols / scale, src.rows / scale));
	feat_detect_saliency(scaled, SA_FT);
	Mat bg(scaled.size(), CV_8UC1);
	bg.setTo(255);
	//subtract(bg, scaled, bg);
	//imshow("sa", scaled);
	//imshow("bg", bg);
	IplImage iplFg = scaled;
	//IplImage iplBg=bg;
	CvMemStorage *pStorageFg = cvCreateMemStorage(0);
	//CvMemStorage* pStorageBg=cvCreateMemStorage(0);
	CvSeq *pContours[1] = {NULL};
	extern double max_area;
	extern double min_area;

	extern Logistic log_hog;
	extern Logistic log_hist;
	extern int histFeatLen;
	extern int hogFeatLen;
	extern int pixFeatLen;
	extern int bowFeatLen;

	max_area = (double)(src.cols * src.rows / 2);
	min_area = 100.0;
	if (targets.size() > 0)
	{
		Rect &max_loc = targets[0].location;
		Rect &min_loc = targets[targets.size() - 1].location;
		max_area = (double)min(max_area, max_loc.height * max_loc.width * 3.0);
		min_area = (double)max(min_area, min_loc.height * min_loc.width * 0.2);
	}
	double ratio[2] = {0.3, 2}; //w/h
	cvFindContours(&iplFg, pStorageFg, &(pContours[0]), sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//cvFindContours(&iplBg, pStorageBg, &(pContours[1]), sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	vector<Target>::iterator iter = targets.begin();
	for (; iter != targets.end(); ++iter)
	{
		(*iter).life -= 1;
	}

	int pad = 1;
	for (int i = 0; i < 1; ++i)
	{
		CvSeq *pContour = pContours[i];

		for (; pContour; pContour = pContour->h_next)
		{
			double area = fabs(cvContourArea(pContour)) * scale * scale;
			if (area > max_area or area < min_area)
			{
				cvSeqRemove(pContour, 0);
				continue;
			}
			CvRect bbox = cvBoundingRect(pContour, 0);
			Rect raw_bbox = Rect(max(0, bbox.x * scale - pad), max(0, bbox.y * scale - pad), bbox.width * scale + 2 * pad, bbox.height * scale + 2 * pad);
			raw_bbox.width = min(src.cols - raw_bbox.x, raw_bbox.width);
			raw_bbox.height = min(src.rows - raw_bbox.y, raw_bbox.height);

			double bw = 1.0 * raw_bbox.width;
			double bh = 1.0 * raw_bbox.height;
			if (bw / bh < ratio[0] or bw / bh > ratio[1])
			{
				cvSeqRemove(pContour, 0);
				continue;
			}

			Mat roi = src(raw_bbox);

			float res_hist = detect_logistic_hist(roi, log_hist, histFeatLen);
			float res_hog = detect_logistic_hog(roi, log_hog, hogFeatLen);

			if (res_hist < 0.6 or res_hog < 0.6)
			{
				cvSeqRemove(pContour, 0);
				continue;
			}
			Target t;
			t.location = raw_bbox;
			int closest_index = 0;
			bool is_new = is_new_target(t, targets, closest_index);

			if (is_new)
			{
				if (not bCurFrame)
				{
					if (target_detectable(t, src.size()))
					{
						if (targets.size() < 20)
						{
							t.life = LIFE;
							t.index = targets.size();
							//cout << "target " << t.index << " added" << endl;
							targets.push_back(t);
						}
						else
						{
							//cout << "too many pots" << endl;
						}
					}
				}
			}
			else if (target_detectable(t, src.size()))
			{ //update
				//cout << "target " << targets[closest_index].index << " updated" << endl;
				targets[closest_index].location = raw_bbox;
				targets[closest_index].life = LIFE;
			}
			else
			{
				//cout << "target " << targets[closest_index].index << " disappeared" << endl;
				vector<Target>::iterator iter_erase = targets.erase(targets.begin() + closest_index);
				for (; iter_erase != targets.end(); ++iter_erase)
				{
					(*iter_erase).index--;
				}
			}
			cvSeqRemove(pContour, 0);
		}
		if (targets.size() > 0)
		{
			vector<Target>::iterator iter = targets.begin();
			for (; iter != targets.end();)
			{
				if ((*iter).life == 0)
				{
					//cout << "target " << (*iter).index << " died" << endl;
					iter = targets.erase(iter);
				}
				else
				{
					++iter;
				}
			}
		}
	}
	vector<Target> temp = targets;
	vector<Target>().swap(targets);
	targets = temp;
	cvReleaseMemStorage(&pStorageFg);
	//	cvReleaseMemStorage(&pStorageBg);
}

void detect_bbox_kcf(Mat &src, vector<Target> &targets)
{
	//use kcf to track the target
}

bool is_new_target(Target &t, vector<Target> &targets, int &index)
{
	if (targets.size() == 0)
		return true;
	bool is_new = true;
	int max_dist = 10000;
	Rect &cur_loc = t.location;
	int metric = NEAR_METRIC;

	vector<Target>::iterator iter = targets.begin();
	for (; iter != targets.end(); ++iter)
	{
		Rect &loc = (*iter).location;
		if (abs(cur_loc.x - loc.x) > metric or abs(cur_loc.y - loc.y) > metric)
		{
			continue; //not near to this target
		}
		int dist = abs(cur_loc.x - loc.x) + abs(cur_loc.y - loc.y);
		if (dist < max_dist)
		{
			max_dist = dist;
			index = iter - targets.begin();
		}
		is_new = false;
	}
	//find the nearest visible target
	return is_new;
}

bool target_detectable(Target &t, const Size &win)
{
	Rect &loc = t.location;
	if (loc.x > PAD_LT and loc.y > PAD_LT and loc.x + loc.width < win.width - PAD_RB and loc.y + loc.height < win.height - PAD_RB)
		return true;
	return false;
}

void track_by_detect()
{
}

bool target_compare(Target &t1, Target &t2)
{
	int t1_area = t1.location.width * t1.location.height;
	int t2_area = t2.location.width * t2.location.height;
	return t1_area > t2_area;
}
