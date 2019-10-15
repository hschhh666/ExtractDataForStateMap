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
	double totalframes;//该帧所在文件所含的总帧数，若为1则表明文件总共1帧
private:
	OneFrameDSVLData *oneFrameDSVLData;//读到的一帧数据就保存在这里
	long long  fileLens;
	std::string fileName;
	std::ifstream filePointer;
	int curFrame;//当前是第几帧，从0开始计
};


class SegAndTransLogReader {
public:
	SegAndTransLogReader(std::string seglogFileName, std::string translogFileName);
	std::vector<std::map<int,SegInfo>> FramesSegsInfo;//以帧为单位保存，每帧内的所有Seg存为一个map，map的key是prid，value就是该prid的信息，比如prid在rangeImage中的位置、语义信息。

};



class StaticFrameLogReader {
public:
	StaticFrameLogReader(std::string filename,int maxFrames = 600);//600帧约等于一分钟
	std::vector<int> startFrames;
	std::vector<int> endFrames;

};