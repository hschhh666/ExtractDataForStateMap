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
	double frameNum;
private:
	OneFrameDSVLData *oneFrameDSVLData;//������һ֡���ݾͱ���������
	long long  fileLens;
	std::string fileName;
	std::ifstream filePointer;
};
