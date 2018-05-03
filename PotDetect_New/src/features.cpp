/*
 * features.cpp
 *
 */

#include "PotDetect_New/features.h"
#include <math.h>
#include <fstream>
#include <imav/imavFunctions.h>

std::string my_home_path = expand_user("~") + "/";

string log_file_hog = my_home_path + "catkin_ws/src/PotDetect_New/src/log_params/log_hog.txt";
string log_file_hist = my_home_path + "catkin_ws/src/PotDetect_New/src/log_params/log_hist.txt";
string log_file_pix = my_home_path + "catkin_ws/src/PotDetect_New/src/log_params/log_pix.txt";
string log_file_bow = my_home_path + "catkin_ws/src/PotDetect_New/src/log_params/log_bow.txt";

void feat_max_blur(Mat &src, Size ksize)
{
	int h = src.rows;
	int w = src.cols;
	int pooled_x = w / ksize.width;
	int pooled_y = h / ksize.height;
	uchar *data = src.data;
	for (int y = 0; y < pooled_y; ++y)
	{
		int ystart = y * ksize.height;
		for (int x = 0; x < pooled_x; ++x)
		{
			int xstart = x * ksize.width;
			int max = -10000;
			for (int yy = ystart; yy < ystart + min(ksize.height, h - ystart); ++yy)
			{
				for (int xx = xstart; xx < xstart + min(ksize.width, w - xstart); ++xx)
				{
					int index = yy * w + xx;
					if (data[index] > max)
					{
						max = data[index];
					}
					else
					{
						data[index] = max;
					}
				}
			}
		}
	}
}

void feat_detect_saliency(Mat &src, int mode)
{
	src = feat_saliency(src, mode);
}

void get_logistic(Logistic &logi, const char *feat)
{
	string log_file_path = "";
	if (strcmp(feat, "hog") == 0)
	{
		log_file_path = log_file_hog;
	}
	else if (strcmp(feat, "hist") == 0)
	{
		log_file_path = log_file_hist;
	}
	else if (strcmp(feat, "pix") == 0)
	{
		log_file_path = log_file_pix;
	}
	else if (strcmp(feat, "bow") == 0)
	{
		log_file_path = log_file_bow;
	}
	logi.load(log_file_path.c_str());
}

float detect_logistic_hist(Mat &src, Logistic &classifier, int featLen)
{
	float *feat = new float[featLen];
	getHistogramFeature(src, feat, featLen, HIST_SIZE, HIST_SEG);
	return classifier.predict(feat, featLen);
	delete feat;
}

float detect_logistic_hog(Mat &src, Logistic &classifier, int featLen)
{
	Mat gray = src;
	if (src.channels() == 3)
	{
		cvtColor(src, gray, CV_BGR2GRAY);
	}
	int fixed_size = HOG_SIZE;
	int stride = STRIDE;
	int cellSize = CELL_SIZE;
	int blockSize = BLOCK_SIZE;

	Mat sample(1, featLen, CV_32F);
	vector<float> hogFeat;
	resize(gray, gray, Size(fixed_size, fixed_size));
	normalize(gray, gray, 0, 255, NORM_MINMAX);
	feat_hog_cv2(gray, Size(blockSize, blockSize), Size(cellSize, cellSize),
				 Size(stride, stride), hogFeat);
	float *ptr = sample.ptr<float>(0);
	for (int k = 0; k < featLen; ++k)
		ptr[k] = hogFeat[k];
	normalize(sample, sample, 0, 1, NORM_MINMAX);
	float res = classifier.predict(sample);
	return res;
}

float detect_logistic_pix(Mat &src, Logistic &classifier, int featLen)
{
	Mat scale;
	resize(src, scale, Size(PIX_SIZE, PIX_SIZE));
	uchar *data = scale.data;
	float *feat = new float[featLen];
	for (int i = 0; i < featLen; ++i)
	{
		feat[i] = (float)data[i];
	}
	Normalize<float>(feat, featLen, 0.0f, 1.0f);
	float res = classifier.predict(feat, featLen);
	delete feat;
	return res;
}

float detect_logistic_bow(Mat &src, Logistic &classifier, int featLen)
{
	int bowNum = BOW_NUM;

	float *feat = new float[featLen];
	getHistFeatureBoW(src, feat, featLen, HIST_SIZE, BOW_SEG, bowNum);

	float res = classifier.predict(feat, featLen);
	delete feat;
	return res;
}

void train_logistic_hist(int iter)
{
	int featLen = 256 / HIST_SEG;
	featLen = featLen * featLen * featLen;
	string label_file_path = "I:/TestOpenCV/Videos/pot_train/label_hist.txt";
	string log_file_path = log_file_hist;
	ifstream in;
	in.open(label_file_path.c_str(), ios::in);

	float *feat = new float[featLen];
	int pos_cnt = 0, neg_cnt = 0;
	cout << "preparing data..." << endl;
	while (not in.eof())
	{
		string line;
		int label;
		in >> line;
		in >> label;
		if (line.length() <= 1)
			continue;
		if (label == 1)
			++pos_cnt;
		else if (label == -1)
			++neg_cnt;
		else
			assert(label == 1 or label == -1);
	}
	cout << pos_cnt << ", " << neg_cnt << endl;
	Mat sampleMat(pos_cnt + neg_cnt, featLen, CV_32F);
	Mat labelMat(1, pos_cnt + neg_cnt, CV_32F);
	int r = 0;
	in.close();
	in.open(label_file_path.c_str(), ios::in);

	float *labelPtr = labelMat.ptr<float>(0);
	while (not in.eof())
	{
		string line;
		int label;
		in >> line;
		in >> label;
		if (line.length() <= 1)
			continue;
		if (label == -1)
		{
			label = 0;
		}
		Mat image = imread(line);

		float *featPtr = sampleMat.ptr<float>(r);
		getHistogramFeature(image, featPtr, featLen, HIST_SIZE, HIST_SEG);
		/*
		for(int k=0;k<featLen;++k)
			featPtr[k]=feat[k];
		*/
		labelPtr[r] = (float)label;
		++r;
	}
	in.close();
	cout << "data prepared, start training..." << endl;
	Logistic logi(featLen);
	logi.lr = 0.01;
	logi.minibatch = 20;
	logi.step = 4;
	logi.gamma = 0.2;
	logi.train(sampleMat, labelMat, iter);
	logi.save(log_file_path.c_str());
	delete feat;
}

void train_logistic_hog(int iter)
{
	int fixed_size = HOG_SIZE;
	int stride = STRIDE;
	int cellSize = CELL_SIZE;
	int blockSize = BLOCK_SIZE;
	int featLen = ((fixed_size - blockSize) / stride + 1) * ((fixed_size - blockSize) / stride + 1) * 36;
	string label_file_path = "I:/TestOpenCV/Videos/pot_train/label_hog.txt";
	string log_file_path = log_file_hog;
	ifstream in;
	in.open(label_file_path.c_str(), ios::in);
	vector<float> hogFeat;
	int pos_cnt = 0, neg_cnt = 0;
	cout << "preparing data..." << endl;
	while (not in.eof())
	{
		string line;
		int label;
		in >> line;
		in >> label;
		if (line.length() <= 1)
			continue;
		if (label == 1)
			++pos_cnt;
		else if (label == -1)
			++neg_cnt;
		else
			assert(label == 1 or label == -1);
	}
	cout << pos_cnt << ", " << neg_cnt << endl;
	Mat sampleMat(pos_cnt + neg_cnt, featLen, CV_32F);
	Mat labelMat(1, pos_cnt + neg_cnt, CV_32F);
	int r = 0;
	in.close();
	in.open(label_file_path.c_str(), ios::in);
	Mat gray;
	float *labelPtr = labelMat.ptr<float>(0);
	while (not in.eof())
	{
		string line;
		int label;
		in >> line;
		in >> label;
		Mat image = imread(line);
		if (line.length() <= 1)
			continue;
		resize(image, image, Size(fixed_size, fixed_size));
		if (image.channels() == 3)
			cvtColor(image, gray, CV_BGR2GRAY);
		else
			gray = image;
		normalize(gray, gray, 0, 255, NORM_MINMAX);
		feat_hog_cv2(image, Size(blockSize, blockSize), Size(cellSize, cellSize),
					 Size(stride, stride), hogFeat);
		assert((int)hogFeat.size() == featLen);
		float *featPtr = sampleMat.ptr<float>(r);

		for (int k = 0; k < featLen; ++k)
			featPtr[k] = hogFeat[k];
		Mat row = sampleMat.row(r);
		normalize(row, row, 0, 1, NORM_MINMAX);
		if (label == -1)
		{
			label = 0;
		}
		labelPtr[r] = (float)label;
		++r;
	}
	in.close();
	cout << "data prepared, start training..." << endl;
	Logistic logi(featLen);
	logi.lr = 0.01;
	logi.minibatch = 20;
	logi.step = 4;
	logi.gamma = 0.2;
	logi.train(sampleMat, labelMat, iter);
	logi.save(log_file_path.c_str());
}

void train_logistic_pix(int iter)
{
	int featLen = PIX_SIZE * PIX_SIZE * 3;
	string label_file_path = "I:/TestOpenCV/Videos/pot_train/label_hist.txt";
	string log_file_path = log_file_pix;
	ifstream in;
	in.open(label_file_path.c_str(), ios::in);

	int pos_cnt = 0, neg_cnt = 0;
	cout << "preparing data..." << endl;
	while (not in.eof())
	{
		string line;
		int label;
		in >> line;
		in >> label;
		if (line.length() <= 1)
			continue;
		if (label == 1)
			++pos_cnt;
		else if (label == -1)
			++neg_cnt;
		else
			assert(label == 1 or label == -1);
	}
	cout << pos_cnt << ", " << neg_cnt << endl;
	Mat sampleMat(pos_cnt + neg_cnt, featLen, CV_32F);
	Mat labelMat(1, pos_cnt + neg_cnt, CV_32F);
	int r = 0;
	in.close();
	in.open(label_file_path.c_str(), ios::in);

	float *labelPtr = labelMat.ptr<float>(0);
	while (not in.eof())
	{
		string line;
		int label;
		in >> line;
		in >> label;
		if (line.length() <= 1)
			continue;
		if (label == -1)
		{
			label = 0;
		}
		Mat image = imread(line);
		resize(image, image, Size(PIX_SIZE, PIX_SIZE));
		uchar *data = image.data;

		float *featPtr = sampleMat.ptr<float>(r);
		for (int i = 0; i < featLen; ++i)
		{
			featPtr[i] = (float)data[i];
		}
		Normalize<float>(featPtr, featLen, 0.0f, 1.0f);
		labelPtr[r] = (float)label;
		++r;
	}
	in.close();
	cout << "data prepared, start training..." << endl;
	Logistic logi(featLen);
	logi.lr = 0.01;
	logi.minibatch = 20;
	logi.step = 4;
	logi.gamma = 0.2;
	logi.train(sampleMat, labelMat, iter);
	logi.save(log_file_path.c_str());
}

void train_logistic_bow(int iter)
{
	int featLen = 256 / BOW_SEG;
	featLen = featLen * featLen * featLen * BOW_NUM * BOW_NUM;
	string label_file_path = "I:/TestOpenCV/Videos/pot_train/label_hist.txt";
	string log_file_path = log_file_bow;
	ifstream in;
	in.open(label_file_path.c_str(), ios::in);

	float *feat = new float[featLen];
	int pos_cnt = 0, neg_cnt = 0;
	cout << "preparing data..." << endl;
	while (not in.eof())
	{
		string line;
		int label;
		in >> line;
		in >> label;
		if (line.length() <= 1)
			continue;
		if (label == 1)
			++pos_cnt;
		else if (label == -1)
			++neg_cnt;
		else
			assert(label == 1 or label == -1);
	}
	cout << pos_cnt << ", " << neg_cnt << endl;
	Mat sampleMat(pos_cnt + neg_cnt, featLen, CV_32F);
	Mat labelMat(1, pos_cnt + neg_cnt, CV_32F);
	int r = 0;
	in.close();
	in.open(label_file_path.c_str(), ios::in);

	float *labelPtr = labelMat.ptr<float>(0);
	while (not in.eof())
	{
		string line;
		int label;
		in >> line;
		in >> label;
		if (line.length() <= 1)
			continue;
		if (label == -1)
		{
			label = 0;
		}
		Mat image = imread(line);
		float *featPtr = sampleMat.ptr<float>(r);
		getHistFeatureBoW(image, featPtr, featLen, HIST_SIZE, BOW_SEG, BOW_NUM);
		/*
		for(int k=0;k<featLen;++k)
			featPtr[k]=feat[k];
		*/
		labelPtr[r] = (float)label;
		++r;
	}
	in.close();
	cout << "data prepared, start training..." << endl;
	Logistic logi(featLen);
	logi.lr = 0.01;
	logi.minibatch = 20;
	logi.step = 4;
	logi.gamma = 0.2;
	logi.train(sampleMat, labelMat, iter);
	logi.save(log_file_path.c_str());
	delete feat;
}
