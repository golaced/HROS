/*
 * Logistic.cpp
 *
 *  Created on: 2017��7��27��
 *      Author: JYF
 */

#include "PotDetect/Logistic.h"

Logistic::Logistic() {
	// TODO Auto-generated constructor stub
	featLen=-1;
	params=NULL;
	minibatch=10;
	step=2;
	gamma=0.5;
}

Logistic::Logistic(int featLen){
	this->featLen=featLen;
	this->params=new float[featLen];
	minibatch=10;
	step=2;
	gamma=0.5;
}
Logistic::~Logistic() {
	// TODO Auto-generated destructor stub
	if(params!=NULL){	delete params;	}
}

void Logistic::shuffle(){
	for(int i=0;i<ind.size();++i){
		int swap_ind=rand()%(ind.size());
		swap(ind[i],ind[swap_ind]);
	}
}

void Logistic::train(Mat& sampleMat, Mat& labelMat, int max_iter, float min_err){
	assert(labelMat.rows==1);
	srand(time(0));
	int iter=0;
	float err=10000;
	int cur_index=0;
	int num_samples=sampleMat.rows;
	ind.resize(sampleMat.rows);
	for(int i=0;i<sampleMat.rows;++i){	ind[i]=i;	}
	shuffle();
	init_param();
	float* label_data=labelMat.ptr<float>(0);
	float* batch_error=new float[minibatch];
	float* batch_output=new float[minibatch];
	float* batch_grad=new float[minibatch];
	int step_len=max_iter/step;
	char method=2;
	/*
	 * 0: mse
	 * 1: sda
	 * 2: cross entropy
	 */
	while(iter<max_iter){
		int batch=min(minibatch, num_samples-cur_index);
		err=0;
		for(int k=cur_index;k<cur_index+batch;++k){
			float* data=sampleMat.ptr<float>(ind[k]);
			float sum=0.0;
			for(int j=0;j<featLen;++j){
				sum+=params[j]*data[j];
			}
			float sig=1.0/(1+exp(-sum));
			batch_output[k-cur_index]=sig;
			if(method==0 or method==1)
				batch_error[k-cur_index]=label_data[ind[k]]-sig;
			else{
				if(label_data[ind[k]]>0.5){
					batch_error[k-cur_index]=-log(sig);
					batch_grad[k-cur_index]=-(1-sig);
				}
				else{
					batch_error[k-cur_index]=log(1-sig);	//not -log(1-sig)
					batch_grad[k-cur_index]=sig;
				}
			}
		}
		for(int j=0;j<featLen;++j){
			float sum=0.0;
			for(int k=cur_index;k<cur_index+batch;++k){
				float* data=sampleMat.ptr<float>(ind[k]);
				if(method==1){	sum+=data[j]*batch_error[k-cur_index];	}
				else if(method==0){	sum+=(-data[j])*batch_error[k-cur_index]*batch_output[k-cur_index]*(1-batch_output[k-cur_index]);	}
				else if(method==2){	sum+=data[j]*batch_grad[k-cur_index];	}
			}
			if(method==0 or method==2)
				params[j]-=lr*sum;	//gradient descent
			else
				params[j]+=lr*sum;	//gradient ascent
		}
		if(iter%step_len==0){	lr*=gamma;	}
		cur_index+=batch;
		if(cur_index==num_samples){
			shuffle();
			cur_index=0;
		}
		++iter;
		for(int k=0;k<batch;++k){	err+=abs(batch_error[k]);	}
	}
	cout<<"done. error: "<<err<<endl;
	delete batch_error;
	delete batch_output;
	delete batch_grad;
}

float Logistic::predict(Mat& featMat){
	assert(featMat.rows==1);
	float* data=featMat.ptr<float>(0);
	float sum=0.0;
	for(int j=0;j<featLen;++j)
		sum+=params[j]*data[j];
	return 1.0/(1+exp(-sum));
}

int Logistic::classify(Mat& featMat){
	float prob=predict(featMat);
	return prob>0.5?1:-1;
}

void Logistic::save(const char* file_name){
	ofstream out;
	out.open(file_name, ios::out);
	int i;
	for(i=0;i<featLen-1;++i)
		out<<params[i]<<endl;
	out<<params[i];
	out.close();
}

void Logistic::load(const char* file_name){
	//assert(params!=NULL);
	ifstream in;
	in.open(file_name, ios::in);
	float f;
	if(params==NULL){
		int total=0;
		while(not in.eof()){
			in>>f;
			++total;
		}
		params=new float[total];
		featLen=total;
	}
	in.close();
	in.open(file_name, ios::in);
	for(int i=0;i<featLen;++i){
		in>>f;
		params[i]=f;
	}
	in.close();
}
