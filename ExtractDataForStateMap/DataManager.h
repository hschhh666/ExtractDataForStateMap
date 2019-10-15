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
	int curFrame;//��ǰ�ǵڼ�֡����0��ʼ��
	int totalframes;//��֡�����ļ���������֡������Ϊ1������ļ��ܹ�1֡
};

class SegInfo {
public:
	int prid;//seg�����б��
	int iminx;//seg��rangeimage�е�λ��
	int iminy;//seg��rangeimage�е�λ��
	int imaxx;//seg��rangeimage�е�λ��
	int imaxy;//seg��rangeimage�е�λ��
	int semanticLabel;//seg�������ǩ
	bool valid;//seg��������Ϣ�Ƿ񶼿��ã���ָcx/cy/cz)����������ò�Ϊtrue����ʼ��Ϊfalse
	double cx;//seg��point cloud�е�����λ�ã�seg��ƽ������λ�ã�
	double cy;//seg��point cloud�е�����λ�ã�seg��ƽ������λ�ã�
	double cz;//seg��point cloud�е�����λ�ã�seg��ƽ������λ�ã�
	int segNum;//seg�ĵ���

	SegInfo() {};
	SegInfo(int p, int nx, int ny, int xx, int xy, int s, bool v = false) :prid(p), iminx(nx), iminy(ny), imaxx(xx), imaxy(xy), semanticLabel(s), valid(v) { cx = cy = cz = 0; segNum = 0; };
};

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
	cv::Mat OGM;//������ռ��դ���ͼ
	cv::Mat StateMap;//������״̬��ͼ
};

