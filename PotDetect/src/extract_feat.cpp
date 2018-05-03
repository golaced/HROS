/*
 * extract_feat.cpp
 *
 *  Created on: 2017��7��23��
 *      Author: JYF
 */

#include "PotDetect/extract_feat.h"
#include "PotDetect/saliency.h"

Mat feat_canny(Mat& src){
	int ch=src.channels();
	Mat gray=src;
	if(ch==3){
		cvtColor(src, gray, CV_RGB2GRAY);
	}
	Mat canny;
	Canny(src, canny, 100,300);
	return canny;
}

Mat feat_houghCircle(Mat& src){
	int ch=src.channels();
	Mat gray=src;
	if(ch==3){
		cvtColor(src, gray, CV_RGB2GRAY);
	}
	GaussianBlur(gray, gray, Size(3,3),0.1);
	int radiusThresh=0;
	Mat canvas=Mat::zeros(src.size(), CV_8UC1);
    vector<Vec3f> circles;
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1.5 ,10, 80, 110, 35 ,65);
    vector<Vec3f>::iterator iter=circles.begin();
    for(;iter!=circles.end();++iter){
    	Vec3f point=*iter;
    	int centerX=point[0];
    	int centerY=point[1];
    	int radius=point[2];
    	if(radius>radiusThresh){
    		circle(canvas, Point(centerX,centerY),radius,255);
    	}
    }
    return canvas;
}


void feat_hog_cv2(Mat& src, Size blockSize, Size cellSize, Size stride, vector<float>& hogFeat){
	int ch=src.channels();
	Mat gray=src;
	if(ch==3){
		cvtColor(src, gray, CV_RGB2GRAY);
	}
	HOGDescriptor hog;
	hog.blockSize=blockSize;
	hog.cellSize=cellSize;
	hog.blockStride=stride;
	hog.winSize=Size(src.cols,src.rows);
	hog.compute(src, hogFeat);
}

Mat feat_hog(Mat& src){
	IplImage _ipl=src;
	CvLSVMFeatureMapCaskade * map;
	int cellSize=8;
	getFeatureMaps(&_ipl,cellSize,&map);
	normalizeAndTruncate(map,0.2f);
	PCAFeatureMaps(map);
	Mat FeaturesMap = cv::Mat(cv::Size(map->numFeatures,map->sizeX*map->sizeY), CV_32F, map->map);  // Procedure do deal with cv::Mat multichannel bug
	FeaturesMap = FeaturesMap.t();
	freeFeatureMapObject(&map);
	return FeaturesMap;
}

Mat feat_saliency(Mat& src, int mode){
	Mat sa;
	if(mode==SA_FREQ){
		int ch=src.channels();
		Mat gray=src;
		if(ch==3){
			cvtColor(src, gray, CV_RGB2GRAY);
		}
		detectSaliency(gray, sa);
	}

	else if(mode==SA_FT){
		saliencyDetectFT(src, sa);
	}
	else
		assert(0);
	threshold(sa, sa, 0, 255, THRESH_OTSU);
	return sa;
}
