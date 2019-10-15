#include "DataManager.h"


//地图数据结构的默认构造函数，在这里初始化地图
MapContainer::MapContainer()
{
	imgSize = mapSize / pixelSize;
	OGM = cv::Mat::zeros(cv::Size(imgSize, imgSize), CV_8UC3);
	StateMap = cv::Mat::zeros(cv::Size(imgSize, imgSize), CV_32FC(10));
}

//初始化地图
void MapContainer::InitMap()
{
	OGM = cv::Mat::zeros(cv::Size(imgSize, imgSize), CV_8UC3);
	StateMap = cv::Mat::zeros(cv::Size(imgSize, imgSize), CV_32FC(10));
}

//全局坐标与图像坐标的转换，若输入的全局坐标点出现在该地图中则返回true，否则返回false
//全局地图中，定义横向为x，向右为正；纵向为y，向上为正。采取东北天坐标系，即右东，上北
//imgx指横向坐标，向右为正；imgy指纵向坐标，向下为正。一定要注意这里对坐标系的定义。因为在opencv中，cv::Point(x,y)的x，y坐标与我前面定义的相同。
bool MapContainer::CoordinateConventer(double x, double y , int & imgx, int & imgy)
{
	imgx = (x - leftUpCornerX) / pixelSize;
	imgy = (leftUpCornerY - y) / pixelSize;
	if ((imgx >= imgSize)||(imgy >= imgSize) || (imgx<0) || (imgy <0))
		return false;
	return true;
}

//全局坐标与图像坐标的转换，若输入的图像坐标在该图像范围内则返回true，否则返回false；
//imgx和imgy指图像坐标系，图像中的坐标系；x，y指全局地图的坐标系
//imgx指横向坐标，向右为正；imgy指纵向坐标，向下为正。一定要注意这里对坐标系的定义。因为在opencv中，cv::Point(x,y)的x，y坐标与我前面定义的相同。
//图像定义的地图中，定义横向为x，向右为正；纵向为y，向上为正。采取东北天坐标系，即右东，上北
bool MapContainer::CoordinateConventer(int imgx, int imgy, double &x, double &y)
{
	if (imgx < 0 || imgy < 0 || imgx >= imgSize || imgy >= imgSize)
		return false;

	x = ((double)imgx)*pixelSize + leftUpCornerX;
	y = leftUpCornerY - ((double)imgy)*pixelSize;
	return true;
}

//地图可视化
void MapContainer::ShowMap(int key = 1,std::string imgname = "OGMMap")
{
	cv::imshow(imgname, OGM);
	cv::waitKey(key);
}
