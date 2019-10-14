/*定义、生成各种数据结构及数据结构，如读取原始文件的数据结构、不包含动态物体的数据、动态物体的数据*/

#pragma once
#include <iostream>
#include <map>
#include <fstream>
#include "opencvHeader.h"

#define LINES 40
#define LINES_PER_BLOCK 10
#define PTNUM_PER_BLOCK (40*10)
#define BLOCK_PER_FRAME 180

#define UNKNOWN			0
#define NONVALID		-9999
#define GROUND			-999
#define	BACKGROUND		-99
#define PERSION			1
#define RIDER			2
#define CAR				3
#define TRUCKORBUS		4
#define BICYCLE			8
#define FOROGM          -6666


#define	sqr(x)			((x)*(x))
typedef double  MATRIX[3][3];


//3个double
struct Point3d {
	double x;
	double y;
	double z;
};

//3个double + 1个unsigned char
struct Point3fi {
	float x;
	float y;
	float z;
	unsigned char i;
};

//2个double
struct Point2d
{
	double x;
	double y;
};

//一block的dsvl数据,dsvl=激光+nav+label
struct OneBlockDSVLData {
	Point3d ang;
	Point3d shv;
	long millisec;
	Point3fi points[PTNUM_PER_BLOCK];
	int label[PTNUM_PER_BLOCK];
};

//一帧dsvl数据,dsvl=激光+nav+label
struct OneFrameDSVLData {
	OneBlockDSVLData oneBlockDsvData[BLOCK_PER_FRAME];
	Point3d ang;
	Point3d shv;
};

class SegInfo {
public:
	int prid;
	int iminx;
	int iminy;
	int imaxx;
	int imaxy;
	int semanticLabel;
	bool valid;

	SegInfo() {};
	SegInfo(int p, int nx, int ny, int xx, int xy, int s, bool v = true):prid(p),iminx(nx),iminy(ny),imaxx(xx),imaxy(xy),semanticLabel(s),valid(v){};
};

////点的容器，用来保存每帧根据语义信息分类后的点
//struct PointsContainer {
//	Point3fi  points[BLOCK_PER_FRAME*LINES_PER_BLOCK*LINES];
//	int sequenceNum[BLOCK_PER_FRAME*LINES_PER_BLOCK*LINES];//序列的编号，0表示地面、无效点等，>10000才表示有效序列
//	int labels[BLOCK_PER_FRAME*LINES_PER_BLOCK*LINES];//注意，此处labels的定义和dsvl中的不一样，此处的labels指的是语义信息，如人、车、地面等
//	Point3d ang;
//	Point3d shv;
//};
//
//
////根据语义信息，对每帧的点进行分类。
//class PointsClassifyer {
//public:
//	PointsClassifyer(std::string logFile);//读取trans.log文件，并记录序列和类别的映射关系
//	void DoClassify(OneFrameDSVLData *frame);
//	void ClearClassifyer();
//	PointsContainer * oneFrameData;
//private:
//	std::map<int, int> dictionary;//保存着序列和类别的映射关系
//
//};
//
//地图的数据结构
class MapContainer {
public:
	MapContainer();//地图数据结构的默认构造函数，在这里初始化地图
	bool CoordinateConventer(double, double, int&, int&);//全局坐标与图像坐标的转换，若输入的全局坐标点出现在该地图中则返回true，否则返回false
	bool CoordinateConventer(int, int, double&, double&);//全局坐标与图像坐标的转换，若输入的图像坐标在该图像范围内则返回true，否则返回false；
	void ShowMap(int key,std::string imgname);//地图可视化
	int imgSize;//pixel 地图的像素大小
	double mapSize = 600;//meter 地图对应的实际物理环境大小
	double pixelSize = 0.2;//meter 每个像素对应的实际物理尺寸
	double leftUpCornerX, leftUpCornerY;//图像左上角对应的真实世界中的全局坐标
	cv::Mat map;//保存着地图
};

