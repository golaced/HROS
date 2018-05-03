/*
 * landing.cpp
 *
 *  Created on: 2017??8??14??
 *      Author: JYF
 */


#include "landing.h"

void createSamples(string& seed, string& dir, int N){
	Mat seedImg=imread(seed);
	double angle=180;
	double res=angle/N;
	cv::Point2f center(seedImg.cols / 2, seedImg.rows / 2);

	stringstream ss;
	int cnt=1;
	for(double ang=res;ang<angle;ang+=res){
		Mat rotated;
		Mat rot = cv::getRotationMatrix2D(center, ang, 1);
		Rect bbox = cv::RotatedRect(center, seedImg.size(), angle).boundingRect();

		rot.at<double>(0, 2) += bbox.width / 2.0 - center.x;
		rot.at<double>(1, 2) += bbox.height / 2.0 - center.y;

		cv::warpAffine(seedImg, rotated, rot, seedImg.size());
		ss<<"h_"<<cnt<<".jpg";
		imwrite(dir+ss.str(), rotated);
		ss.str("");
		++cnt;
	}
}

Mat getHSVImage(const Mat& src, const char* chn){
	Mat hsv;
	cvtColor(src, hsv, CV_BGR2HSV);
	vector<Mat> hsvPlane;
	split(hsv,hsvPlane);
	Mat ret;
	if(strcmp(chn,"s")==0){
		ret=hsvPlane[1];
	}
	else if(strcmp(chn,"v")==0){
		ret=hsvPlane[2];
	}
	return ret;
}

Mat alignImage(const Mat& src, int size){
	float fw=1.0*src.cols;
	float fh=1.0*src.rows;
	float ratio=1.0*size/max(fw,fh);

	Mat aligned;
	if(fw>fh){
		resize(src, aligned, Size(size, (int)(fh*ratio)));
	}
	else{
		resize(src, aligned, Size((int)(fw*ratio), size));
	}
	return aligned;
}

void getContours(Mat& biImg, vector<Rect>& contours){
	int min_area=MIN_AREA;
	int max_pad=MAX_PAD;

	Mat dummy;
	biImg.copyTo(dummy);

	std::vector<cv::Vec4i> hierarchy2;
	std::vector<std::vector<cv::Point> > contours2;
	cv::findContours(dummy, contours2, hierarchy2, CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);

	for (auto it = contours2.begin(); it != contours2.end();it++)
	{
		double area = fabs(cv::contourArea(*it));
		cv::Rect bbox = cv::boundingRect(*it);
		if(area>min_area and bbox.width<biImg.cols-max_pad and bbox.height<biImg.rows-max_pad)
		{
			contours.push_back(bbox);
		}
	}
}

Mat ToTemplate(Mat& src){
	assert(src.channels()==1);
	Mat _tmpl=src.reshape(0,1);
	return _tmpl;
}

void createTemplate(const string& file_list, const string& temp_file){
	int temp_w=TEMP_W;
	int temp_h=TEMP_H;
	int n_Samples=0;
	ifstream in;
	ofstream out;
	string line;
	bool bFirstLine=true;

/*
	string ins_file="./ins.txt";
	ofstream ins_out;

	in.open(file_list.c_str(), ios::in);
	getline(in,line);
	Mat sample=imread(line,0);
	threshold(sample,sample,250,255,THRESH_BINARY);
	imshow("roi",sample);	waitKey();
	vector<Rect> _c;

	getContours(sample, _c);

	Mat h_roi=sample(_c[0]);

	resize(h_roi,h_roi,Size(temp_w,temp_h));
	threshold(h_roi,h_roi,250,255,cv::THRESH_BINARY);
	imshow("roi",h_roi);	waitKey();

	int val;
	for(int i=0;i<h_roi.rows;++i){
		uchar* ptr=h_roi.ptr(i);
		for(int j=0;j<h_roi.cols;++j){
			val=(int)ptr[j];
			if(ptr[j]==255){	val=1;	}
			ins_out<<val<<' ';
		}
		ins_out<<'\n';
	}
	ins_out.close();
*/

	in.open(file_list.c_str(), ios::in);
	out.open(temp_file.c_str(), ios::out);

	while(not in.eof()){
		getline(in, line);
		if(line.length()<=1){	continue;	}
		++n_Samples;
	}
	in.close();
	in.open(file_list.c_str(), ios::in);
	while(not in.eof()){
		getline(in, line);
		if(line.length()<=1){	continue;	}
		Mat image=imread(line,0);

		vector<Rect> c;
		threshold(image,image,250,255,THRESH_BINARY);

		getContours(image, c);

		Mat h_roi=image(c[0]);

		resize(h_roi,h_roi,Size(temp_w,temp_h));

		threshold(h_roi,h_roi,250,255,cv::THRESH_BINARY);

		Mat _tmpl=h_roi.reshape(0,1);
		uchar* ptr=_tmpl.ptr(0);

		if(bFirstLine){
			bFirstLine=false;
			out<<n_Samples<<' '<<_tmpl.cols<<'\n';
		}
		int val=0;
		for(int j=0;j<_tmpl.cols;++j){
			val=(int)ptr[j];
			if(ptr[j]==255){	val=1;	}
			out<<val;
			if(j<_tmpl.cols-1){	out<<' ';	}
			else{	out<<'\n';	}
		}
	}
	in.close();
	out.close();
}

Mat loadTemplate(const string& temp_file){
	ifstream in;
	in.open(temp_file.c_str(), ios::in);

	int w,h;
	in>>h>>w;
	//cout<<w<<","<<h<<endl;

	Mat _tmpl(Size(w,h), CV_8UC1);
	int row=0;
	string line;

	int val;
	while(not in.eof()){
		getline(in, line);
		if(line.length()<=1){	continue;	}
		stringstream ss(line);
		uchar* ptr=_tmpl.ptr(row);
		for(int j=0;j<w;++j){
			ss>>val;
			if(val==1){	val=255;	}
			ptr[j]=(uchar)val;
		}
		++row;
	}
	in.close();
	/*
	Mat sample=_tempMat.row(10);
	sample=sample.reshape(0,TEMP_H);
	imshow("", sample);waitKey();
	*/
	return _tmpl;
}

int KNN(Mat& img, Mat& _tmpl, vector<Rect>& contours){
	int _tmpl_h=TEMP_H;
	int _tmpl_w=TEMP_W;

	int min_diff_=1e7;
	int min_index=0;

	for(uint i=0;i<contours.size();++i){
		Rect& c=contours[i];
		Mat roi=img(c);
		resize(roi,roi,Size(_tmpl_w,_tmpl_h));

		threshold(roi,roi,250,255,THRESH_BINARY);

		int min_diff=1e7;
		for(int j=0;j<_tmpl.rows;++j){
			Mat _t=_tmpl.row(j);
			int diff=0;
			for(int k=0;k<_tmpl.cols;++k){
				if(roi.data[k]!=_t.data[k]){	++diff;	}
			}
			if(diff<min_diff){	min_diff=diff;	}
			//cout<<diff<<endl;
		}
		if(min_diff<min_diff_){
			min_index=i;
			min_diff_=min_diff;
		}
	}
	if(min_diff_>250){
		min_index=-1;
	}
	return min_index;
}
