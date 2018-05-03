/*
 * util.cpp
 *
 *      @Author: JYF
 */

#include "PotDetect_New/util.h"
#include <string>

void readLabelFile(string label_file, vector<string> &file_names, vector<int> &labels)
{
	ifstream in;
	in.open(label_file.c_str(), ios::in);
	while (not in.eof())
	{
		string file_name;
		int label;
		in >> file_name;
		in >> label;
		if (file_name.length() <= 1)
		{
			continue;
		}
		file_names.push_back(file_name);
		labels.push_back(label);
	}
}

void readParamsFromFile(string param_file, vector<HogParam> &params)
{
	ifstream in;
	in.open(param_file.c_str(), ios::in);
	if (not in)
	{
		cout << "Open file failed" << endl;
		return;
	}
	HogParam param;
	bool bHasElement = false;
	int width, height;
	while (not in.eof())
	{
		string line;
		in >> line;
		if (line.length() <= 1)
		{
			continue;
		}
		if (strcmp("[Width]", line.c_str()) == 0)
		{
			in >> line;
			width = atoi(line.c_str());
			param.winSize.width = width;
		}
		else if (strcmp("[Height]", line.c_str()) == 0)
		{
			in >> line;
			height = atoi(line.c_str());
			param.winSize.height = height;
		}
		else if (strcmp("[HOG]", line.c_str()) == 0)
		{
			if (bHasElement)
			{
				param.winSize.width = width;
				param.winSize.height = height;
				params.push_back(param);
			}
			else
			{
				bHasElement = true;
			}
		}
		else
		{
			int num = atoi(line.substr(line.find(":", 0) + 1).c_str());
			if (line.find("cell") != std::string::npos)
			{
				param.cellSize = num;
			}
			else if (line.find("block") != std::string::npos)
			{
				param.blockSize = num;
			}
			else if (line.find("stride") != std::string::npos)
			{
				param.stride = num;
			}
			else
			{
				assert(0);
			}
		}
	}
	param.winSize.width = width;
	param.winSize.height = height;
	params.push_back(param);
	in.close();
}

int calcFeatLen(vector<HogParam> &params)
{
	int total_len = 0;
	for (int i = 0; i < params.size(); ++i)
	{
		HogParam &p = params[i];
		int len = ((p.winSize.width - p.blockSize) / p.stride + 1) * ((p.winSize.height - p.blockSize) / p.stride + 1) * 36;
		total_len += len;
	}
	return total_len;
}

void printParams(vector<HogParam> &params)
{
	for (unsigned int i = 0; i < params.size(); ++i)
	{
		HogParam &param = params[i];
		cout << "HOG[" << i << "]" << endl;
		cout << "cell:" << param.cellSize << endl;
		cout << "block:" << param.blockSize << endl;
		cout << "stride:" << param.stride << endl;
		cout << "winSize:(" << param.winSize.width << "," << param.winSize.height << ")" << endl;
	}
}
