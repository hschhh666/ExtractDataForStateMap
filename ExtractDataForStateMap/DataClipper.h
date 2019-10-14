#pragma once

#include "define.h"
class DataClipper {
public:
	DataClipper(std::string staticlogFileName, int framenum);
	~DataClipper();
	void getStaticFrames(OneFrameDSVLData* oneFrameDSVData);

private:
	std::ofstream staticlogfp;
	int SegCounter = 0;//��������������м��ξ�ֹ�ģ���1��ʼ��
	double basePosition = -666;//ÿһ֡��λ�������ֵ�Աȣ��������һ����ֵ����Ϊ�����ƶ���
	int FrameCounter = 0;//��ǰ��ȡ�˶���֡�����㿪ʼ��
	int startFrame = -1;//һ�ξ�ֹ���ݵ���ʼ֡
	int endFrame = -1;//һ�ξ�ֹ���ݵĽ���֡


	int cacheSize = 100;//���漤�����ݵ�֡������������Լ10fps����cacheSize=100ʱ��������10s�ļ�������
	int curSize = 0;//һ�ξ�ֹ�����е�ǰ�Ѿ���ֹ��֡��

	int totalFrameNum;

};