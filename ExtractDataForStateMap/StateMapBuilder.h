#pragma once
#include "define.h"

//这里写的很蠢，StateMapBuilder类的数据结构基本和OGMBuilder的一样。那为啥还要新定义这个数据结构？因为我蠢，就是想单纯的区分OGM和占有栅格地图


class StateMapBuilder
{
public:
	StateMapBuilder(std::string p40calibFile, std::vector<std::map<int, SegInfo>> FramesSegsInfo ,std::vector<int> StartFrames,std::vector<int> EndFrames,std::string YmlName,std::string videoFile = "");
	void BuildStateMap(OneFrameDSVLData * frame);
	~StateMapBuilder();
private:

	void GetPlaneEquation();//计算点云的平面方程
	void CalculateSegsCenter();//选择用来构建状态地图的点
	void DoBuildingStateMap();//构建状态地图的主函数
	void ShowStateMap(int channel);//

	double DisToPlane(Point3fi *);//点到面的距离


	MapContainer mapContainer;
	std::vector<std::map<int, SegInfo>> framesSegsInfo;
	double pixelSize = 0.2; //单位：米
	double mapSize = 200; //单位：米
	double planeEquation[4];//保存着本帧提取到的地面方程ax+by+cz+d = 0
	OneFrameDSVLData* oneFrameData;
	Point3fi * PointsForStateMap;//保存着用来构建占有栅格地图的点
	int NumOfPointsForStateMap;//用来构建占有栅格地图的点的个数

	std::vector<int> startFrames;
	std::vector<int> endFrames;
	
	std::string ymlName;
	int curDataSeg;//当前处理的数据段


	Point3d shv;//激光雷达到GPS的外参标定参数 shift
	Point3d shvG;//GPS转全局坐标 shift
	MATRIX	RotLidar2GPS;//激光雷达到GPS旋转矩阵
	MATRIX Rot2Global;//转换到全局坐标的旋转矩阵

	cv::VideoWriter video;
};

