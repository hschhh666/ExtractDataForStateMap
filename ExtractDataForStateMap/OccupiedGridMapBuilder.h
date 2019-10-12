/*构建静态占有栅格地图*/
#pragma once
#include "define.h"

class OGMBuilder {
public:
	OGMBuilder(std::string p40calibFile, std::string videoFile = "");
	void BuildOGM(OneFrameDSVLData * frame);//构建OGM地图
	void ShowOGM();

private:
	void pickUpPointsForStaticOGM();//选择用来构建OGM的点，比如排除人、车的点，排除距离地面过高的点
	void GetPlanEquation();//根据点云计算平面方程，其实就是计算地面的方程
	void DoBuildingOGM();//根据选择出来的点构建OGM
	double DisToPlane(Point3fi *);//计算点到面的距离

	MapContainer mapContainer;
	double pixelSize = 0.2; //单位：米
	double mapSize = 200; //单位：米
	double planeEquation[4];//保存着本帧提取到的地面方程ax+by+cz+d = 0
	OneFrameDSVLData* oneFrameData;
	Point3fi * PointsForOGM;//保存着用来构建占有栅格地图的点
	int NumOfPointsForOGM;//用来构建占有栅格地图的点的个数

	int fno;//frame number

	Point3d shv;//激光雷达到GPS的外参标定参数 shift
	Point3d rot;//激光雷达到GPS的外参标定参数 rotate
	MATRIX	RotLidar2GPS;
	Point3d shvG;//转换到全局坐标
	Point3d rotG;//转换到全局坐标
	MATRIX Rot2Global;//转换到全局坐标的旋转矩阵

	cv::VideoWriter video;

};