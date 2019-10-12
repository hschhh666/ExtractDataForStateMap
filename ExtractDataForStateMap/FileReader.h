/*程序的数据输入和输出接口*/
#pragma once
#include "define.h"

//读取DSVL文件
//DSV = 激光+nav+序列编号
class DSVLReader {
public:
	DSVLReader(std::string fileName);
	~DSVLReader();
	void CloseFile();
	OneFrameDSVLData* ReadOneFrameDSVL();//读取一帧数据
	double frameNum;
private:
	OneFrameDSVLData *oneFrameDSVLData;//读到的一帧数据就保存在这里
	long long  fileLens;
	std::string fileName;
	std::ifstream filePointer;
};
