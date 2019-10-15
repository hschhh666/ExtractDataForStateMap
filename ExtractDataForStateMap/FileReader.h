/*������������������ӿ�*/
#pragma once
#include "define.h"

//��ȡDSVL�ļ�
//DSV = ����+nav+���б��
class DSVLReader {
public:
	DSVLReader(std::string fileName);
	~DSVLReader();
	void CloseFile();
	OneFrameDSVLData* ReadOneFrameDSVL();//��ȡһ֡����
	double totalframes;//��֡�����ļ���������֡������Ϊ1������ļ��ܹ�1֡
private:
	OneFrameDSVLData *oneFrameDSVLData;//������һ֡���ݾͱ���������
	long long  fileLens;
	std::string fileName;
	std::ifstream filePointer;
	int curFrame;//��ǰ�ǵڼ�֡����0��ʼ��
};


class SegAndTransLogReader {
public:
	SegAndTransLogReader(std::string seglogFileName, std::string translogFileName);
	std::vector<std::map<int,SegInfo>> FramesSegsInfo;//��֡Ϊ��λ���棬ÿ֡�ڵ�����Seg��Ϊһ��map��map��key��prid��value���Ǹ�prid����Ϣ������prid��rangeImage�е�λ�á�������Ϣ��

};



class StaticFrameLogReader {
public:
	StaticFrameLogReader(std::string filename,int maxFrames = 600);//600֡Լ����һ����
	std::vector<int> startFrames;
	std::vector<int> endFrames;

};