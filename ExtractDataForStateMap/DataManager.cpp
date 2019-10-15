#include "DataManager.h"


//��ͼ���ݽṹ��Ĭ�Ϲ��캯�����������ʼ����ͼ
MapContainer::MapContainer()
{
	imgSize = mapSize / pixelSize;
	OGM = cv::Mat::zeros(cv::Size(imgSize, imgSize), CV_8UC3);
	StateMap = cv::Mat::zeros(cv::Size(imgSize, imgSize), CV_32FC(10));
}

//��ʼ����ͼ
void MapContainer::InitMap()
{
	OGM = cv::Mat::zeros(cv::Size(imgSize, imgSize), CV_8UC3);
	StateMap = cv::Mat::zeros(cv::Size(imgSize, imgSize), CV_32FC(10));
}

//ȫ��������ͼ�������ת�����������ȫ�����������ڸõ�ͼ���򷵻�true�����򷵻�false
//ȫ�ֵ�ͼ�У��������Ϊx������Ϊ��������Ϊy������Ϊ������ȡ����������ϵ�����Ҷ����ϱ�
//imgxָ�������꣬����Ϊ����imgyָ�������꣬����Ϊ����һ��Ҫע�����������ϵ�Ķ��塣��Ϊ��opencv�У�cv::Point(x,y)��x��y��������ǰ�涨�����ͬ��
bool MapContainer::CoordinateConventer(double x, double y , int & imgx, int & imgy)
{
	imgx = (x - leftUpCornerX) / pixelSize;
	imgy = (leftUpCornerY - y) / pixelSize;
	if ((imgx >= imgSize)||(imgy >= imgSize) || (imgx<0) || (imgy <0))
		return false;
	return true;
}

//ȫ��������ͼ�������ת�����������ͼ�������ڸ�ͼ��Χ���򷵻�true�����򷵻�false��
//imgx��imgyָͼ������ϵ��ͼ���е�����ϵ��x��yָȫ�ֵ�ͼ������ϵ
//imgxָ�������꣬����Ϊ����imgyָ�������꣬����Ϊ����һ��Ҫע�����������ϵ�Ķ��塣��Ϊ��opencv�У�cv::Point(x,y)��x��y��������ǰ�涨�����ͬ��
//ͼ����ĵ�ͼ�У��������Ϊx������Ϊ��������Ϊy������Ϊ������ȡ����������ϵ�����Ҷ����ϱ�
bool MapContainer::CoordinateConventer(int imgx, int imgy, double &x, double &y)
{
	if (imgx < 0 || imgy < 0 || imgx >= imgSize || imgy >= imgSize)
		return false;

	x = ((double)imgx)*pixelSize + leftUpCornerX;
	y = leftUpCornerY - ((double)imgy)*pixelSize;
	return true;
}

//��ͼ���ӻ�
void MapContainer::ShowMap(int key = 1,std::string imgname = "OGMMap")
{
	cv::imshow(imgname, OGM);
	cv::waitKey(key);
}
