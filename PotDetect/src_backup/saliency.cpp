#include "PotDetect/saliency.h"

void transform(Mat& src, Mat& dst) {
	Mat imag = Mat::zeros(Size(src.cols, src.rows), CV_64FC1);
	vector<Mat> plane(2);
	plane[0] = src; plane[1] = imag;
	Mat tmp;
	merge(plane, tmp);
	dft(tmp, dst, CV_DXT_FORWARD);
}

void Normalize(Mat& src, Mat& dst) {
	//double min, max;
	//minMaxLoc(src, &min, &max, NULL, NULL);
	//double scale = 255.0 / (max - min);
	//double shift = -min * scale;
	normalize(src, src, 0, 1, NORM_MINMAX);
	src.convertTo(dst, CV_8UC1, 255.0,1);
}
/*
void Normalize(Mat& src, Mat& dst) {
	double min, max;
	minMaxLoc(src, &min, &max, 0, 0);
	double scale = 255.0 / (max - min);
	double shift = -min * scale;
	src.convertTo(dst, CV_64FC1, scale, shift);
}*/

void detectSaliency(Mat& src, Mat& dst) {
	Mat mSource, mFourier, mInverse, mSine, mCosine;
	Mat mAmplitude, mPhase, logAmplitude;
	Mat mResidual, mSaliency;
	Mat temp;
	src.convertTo(mSource, CV_64F, 1.0 / 255.0, 0);
	transform(mSource, mFourier);
	//cout << "Fourier channel: " << mFourier.channels() << endl;
	vector<Mat> fourierPlane(2);
	split(mFourier, fourierPlane);
	Mat mReal = fourierPlane[0];
	Mat mImag = fourierPlane[1];
	
	Mat realTmp, imagTmp;
	pow(fourierPlane[0], 2.0, realTmp);
	pow(fourierPlane[1], 2.0, imagTmp);
	add(realTmp, imagTmp, mAmplitude);
	pow(mAmplitude, 0.5, mAmplitude);
	log(mAmplitude, logAmplitude);

	divide(fourierPlane[0], mAmplitude, mCosine);
	divide(fourierPlane[1], mAmplitude, mSine);

	blur(logAmplitude, temp, Size(3, 3));
	subtract(logAmplitude, temp, mResidual);

	exp(mResidual, mResidual);
	multiply(mResidual, mCosine, fourierPlane[0]);
	multiply(mResidual, mSine, fourierPlane[1]);

	merge(fourierPlane, mFourier);
	dft(mFourier, mInverse, CV_DXT_INV_SCALE);
	split(mInverse, fourierPlane);
	pow(fourierPlane[0], 2.0, realTmp);
	pow(fourierPlane[1], 2.0, imagTmp);
	add(realTmp, imagTmp, mSource);
	//pow(mSource, 0.5, mSource);
	GaussianBlur(mSource, mSaliency, Size(7, 7), 0, 0);
	Normalize(mSaliency, dst);
}

float gamma(float x){
	return x>0.04045 ? pow((x + 0.055f) / 1.055f, 2.4f) : x / 12.92;
}

void RGBToLab(unsigned char * rgbImg, float * labImg)
{
	float B = gamma(rgbImg[0] / 255.0f);
	float G = gamma(rgbImg[1] / 255.0f);
	float R = gamma(rgbImg[2] / 255.0f);
	float X = 0.412453*R + 0.357580*G + 0.180423*B;
	float Y = 0.212671*R + 0.715160*G + 0.072169*B;
	float Z = 0.019334*R + 0.119193*G + 0.950227*B;

	X /= 0.95047;
	Y /= 1.0;
	Z /= 1.08883;

	float FX = X > 0.008856f ? pow(X, 1.0f / 3.0f) : (7.787f * X + 0.137931f);
	float FY = Y > 0.008856f ? pow(Y, 1.0f / 3.0f) : (7.787f * Y + 0.137931f);
	float FZ = Z > 0.008856f ? pow(Z, 1.0f / 3.0f) : (7.787f * Z + 0.137931f);
	labImg[0] = Y > 0.008856f ? (116.0f * FY - 16.0f) : (903.3f * Y);
	labImg[1] = 500.f * (FX - FY);
	labImg[2] = 200.f * (FY - FZ);
}

void saliencyDetectFT(Mat& src, Mat& dst) {
	assert(src.channels() == 3);
	if (dst.empty()) {
		dst.create(src.size(), CV_32FC1);
	}
	Mat lab, labf;
	int h = src.rows, w = src.cols;
	labf.create(Size(w, h), CV_32FC3);
	uchar* fSrc = src.data;
	float* fLab = (float*)labf.data;
	float* fDst = (float*)dst.data;

	int stride = w * 3;
	for (int i = 0; i < h; ++i) {
		for (int j = 0; j < stride; j += 3) {
			RGBToLab(fSrc + i*stride + j, fLab + i*stride + j);
		}
	}
	float MeanL = 0, MeanA = 0, MeanB = 0;
	for (int i = 0; i < h; ++i) {
		int index = i*stride;
		for (int x = 0; x < w; ++x) {
			MeanL += fLab[index];
			MeanA += fLab[index + 1];
			MeanB += fLab[index + 2];
			index += 3;
		}
	}
	MeanL /= (w * h);
	MeanA /= (w * h);
	MeanB /= (w * h);
	GaussianBlur(labf, labf, Size(5, 5), 1);
	for (int Y = 0; Y < h; Y++)
	{
		int Index = Y * stride;
		int CurIndex = Y * w;
		for (int X = 0; X < w; X++)
		{
			fDst[CurIndex++] = (MeanL - fLab[Index]) *  \
				(MeanL - fLab[Index]) + (MeanA - fLab[Index + 1]) *  \
				(MeanA - fLab[Index + 1]) + (MeanB - fLab[Index + 2]) *  \
				(MeanB - fLab[Index + 2]);
			Index += 3;
		}
	}
	normalize(dst, dst, 0, 1, NORM_MINMAX);
	dst.convertTo(dst, CV_8UC1, 255);
	//threshold(dst, dst, 0, 255, THRESH_OTSU);
}
