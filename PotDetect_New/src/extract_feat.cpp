/*
 * extract_feat.cpp
 *
 *  Created on: 2017��7��23��
 *      Author: JYF
 */

#include "PotDetect_New/extract_feat.h"
#include "PotDetect_New/saliency.h"

void feat_hog_cv2(Mat &src, Size blockSize, Size cellSize, Size stride, vector<float> &hogFeat)
{
	int ch = src.channels();
	Mat gray = src;
	if (ch == 3)
	{
		cvtColor(src, gray, CV_RGB2GRAY);
	}
	HOGDescriptor hog;
	hog.blockSize = blockSize;
	hog.cellSize = cellSize;
	hog.blockStride = stride;
	hog.winSize = Size(src.cols, src.rows);
	hog.compute(src, hogFeat);
}

Mat feat_saliency(Mat &src, int mode)
{
	Mat sa;
	if (mode == SA_FREQ)
	{
		int ch = src.channels();
		Mat gray = src;
		if (ch == 3)
		{
			cvtColor(src, gray, CV_RGB2GRAY);
		}
		detectSaliency(gray, sa);
	}

	else if (mode == SA_FT)
	{
		saliencyDetectFT(src, sa);
	}
	else
		assert(0);
	threshold(sa, sa, 250, 255, THRESH_OTSU);
	return sa;
}

void getHogFeature(Mat &image, HogParam &param, vector<float> &hogFeat, bool cvt)
{
	Mat gray = image;
	if (cvt)
	{
		int ch = image.channels();
		if (ch == 3)
		{
			cvtColor(image, gray, CV_BGR2GRAY);
		}
		resize(gray, gray, param.winSize);
		normalize(gray, gray, 0, 255, NORM_MINMAX);
	}
	HOGDescriptor hog;
	hog.blockSize = Size(param.blockSize, param.blockSize);
	hog.cellSize = Size(param.cellSize, param.cellSize);
	hog.blockStride = Size(param.stride, param.stride);
	hog.winSize = param.winSize;
	hog.compute(image, hogFeat);
}

void getMSHogFeature(Mat &image, vector<HogParam> &params, vector<float> &msHogFeat)
{
	Mat gray = image;
	int ch = image.channels();
	if (ch == 3)
	{
		cvtColor(image, gray, CV_BGR2GRAY);
	}
	resize(gray, gray, params[0].winSize);
	//normalize(gray, gray, 0, 255, NORM_MINMAX);
	equalizeHist(gray, gray);
	for (unsigned int i = 0; i < params.size(); ++i)
	{
		HogParam &param = params[i];
		vector<float> ssHogFeat;
		getHogFeature(gray, param, ssHogFeat, false);
		for (unsigned int j = 0; j < ssHogFeat.size(); ++j)
			msHogFeat.push_back(ssHogFeat[j]);
	}
}

void getHistogramFeature(Mat &image, float *&feat, int featLen, int size, int seg, bool bNorm)
{
	resize(image, image, Size(size, size));
	int h = image.rows;
	int w = image.cols;
	int stride = w * 3;

	int base = 256 / seg;
	int coeff[3] = {base * base, base, 1};
	uchar *data = image.data;
	if (feat == NULL)
	{
		feat = new float[featLen];
	}

	for (int i = 0; i < featLen; ++i)
	{
		feat[i] = 0.0;
	}

	for (int i = 0; i < h * stride; i += 3)
	{
		int b = (int)data[i];
		int g = (int)data[i + 1];
		int r = (int)data[i + 2];
		b /= seg;
		g /= seg;
		r /= seg;

		feat[b * coeff[0] + g * coeff[1] + r * coeff[2]] += 1.0;
	}
	if (bNorm)
	{
		Normalize<float>(feat, featLen, 0.0f, 1.0f);
	}
}

void getHistFeatureBoW(Mat &image, float *&feat, int featLen, int size, int seg, int bowNum)
{
	resize(image, image, Size(size, size));
	int i, j, k;
	int grid_sz = size / bowNum;
	int crop_w = grid_sz * bowNum;
	int crop_h = grid_sz * bowNum;
	int start_x = (size - crop_w) / 2;
	int start_y = (size - crop_h) / 2;
	int total = 256 / seg;
	total = total * total * total;
	assert(total * bowNum * bowNum == featLen);
	float *bag_feat = new float[total];
	//for(i=0;i<featLen;++i){	feat[i]=0.0;	}
	for (i = 0; i < bowNum; ++i)
	{
		for (j = 0; j < bowNum; ++j)
		{
			Rect r(j * grid_sz + start_x, i * grid_sz + start_y, min(grid_sz, crop_w - j * grid_sz), min(grid_sz, crop_h - i * grid_sz));
			Mat roi = image(r);
			getHistogramFeature(roi, bag_feat, total, grid_sz, seg, false);
			int offset = (i * bowNum + j) * total;
			for (k = 0; k < total; ++k)
			{
				feat[k + offset] = bag_feat[k];
			}
		}
	}
	Normalize<float>(feat, featLen, 0.0f, 1.0f);
	delete bag_feat;
}
