/*���塢���ɸ������ݽṹ�����ݽṹ�����ȡԭʼ�ļ������ݽṹ����������̬��������ݡ���̬���������*/

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


//3��double
struct Point3d {
	double x;
	double y;
	double z;
};

//3��double + 1��unsigned char
struct Point3fi {
	float x;
	float y;
	float z;
	unsigned char i;
};

//2��double
struct Point2d
{
	double x;
	double y;
};

//һblock��dsvl����,dsvl=����+nav+label
struct OneBlockDSVLData {
	Point3d ang;
	Point3d shv;
	long millisec;
	Point3fi points[PTNUM_PER_BLOCK];
	int label[PTNUM_PER_BLOCK];
};

//һ֡dsvl����,dsvl=����+nav+label
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

////�����������������ÿ֡����������Ϣ�����ĵ�
//struct PointsContainer {
//	Point3fi  points[BLOCK_PER_FRAME*LINES_PER_BLOCK*LINES];
//	int sequenceNum[BLOCK_PER_FRAME*LINES_PER_BLOCK*LINES];//���еı�ţ�0��ʾ���桢��Ч��ȣ�>10000�ű�ʾ��Ч����
//	int labels[BLOCK_PER_FRAME*LINES_PER_BLOCK*LINES];//ע�⣬�˴�labels�Ķ����dsvl�еĲ�һ�����˴���labelsָ����������Ϣ�����ˡ����������
//	Point3d ang;
//	Point3d shv;
//};
//
//
////����������Ϣ����ÿ֡�ĵ���з��ࡣ
//class PointsClassifyer {
//public:
//	PointsClassifyer(std::string logFile);//��ȡtrans.log�ļ�������¼���к�����ӳ���ϵ
//	void DoClassify(OneFrameDSVLData *frame);
//	void ClearClassifyer();
//	PointsContainer * oneFrameData;
//private:
//	std::map<int, int> dictionary;//���������к�����ӳ���ϵ
//
//};
//
//��ͼ�����ݽṹ
class MapContainer {
public:
	MapContainer();//��ͼ���ݽṹ��Ĭ�Ϲ��캯�����������ʼ����ͼ
	bool CoordinateConventer(double, double, int&, int&);//ȫ��������ͼ�������ת�����������ȫ�����������ڸõ�ͼ���򷵻�true�����򷵻�false
	bool CoordinateConventer(int, int, double&, double&);//ȫ��������ͼ�������ת�����������ͼ�������ڸ�ͼ��Χ���򷵻�true�����򷵻�false��
	void ShowMap(int key,std::string imgname);//��ͼ���ӻ�
	int imgSize;//pixel ��ͼ�����ش�С
	double mapSize = 600;//meter ��ͼ��Ӧ��ʵ����������С
	double pixelSize = 0.2;//meter ÿ�����ض�Ӧ��ʵ������ߴ�
	double leftUpCornerX, leftUpCornerY;//ͼ�����ϽǶ�Ӧ����ʵ�����е�ȫ������
	cv::Mat map;//�����ŵ�ͼ
};

