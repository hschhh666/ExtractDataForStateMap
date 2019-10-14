#pragma once

#include "define.h"
class DataClipper {
public:
	DataClipper(std::string staticlogFileName, int framenum);
	~DataClipper();
	void getStaticFrames(OneFrameDSVLData* oneFrameDSVData);

private:
	std::ofstream staticlogfp;
	int SegCounter = 0;//表明这段数据中有几段静止的，从1开始计
	double basePosition = -666;//每一帧的位置与这个值对比，如果大于一定阈值就认为车辆移动了
	int FrameCounter = 0;//当前读取了多少帧，从零开始计
	int startFrame = -1;//一段静止数据的起始帧
	int endFrame = -1;//一段静止数据的结束帧


	int cacheSize = 100;//缓存激光数据的帧数，激光数据约10fps，故cacheSize=100时表明缓存10s的激光数据
	int curSize = 0;//一段静止数据中当前已经静止的帧数

	int totalFrameNum;

};