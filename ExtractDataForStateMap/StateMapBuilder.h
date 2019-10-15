#pragma once
#include "define.h"

//����д�ĺܴ���StateMapBuilder������ݽṹ������OGMBuilder��һ������Ϊɶ��Ҫ�¶���������ݽṹ����Ϊ�Ҵ��������뵥��������OGM��ռ��դ���ͼ


class StateMapBuilder
{
public:
	StateMapBuilder(std::string p40calibFile, std::vector<std::map<int, SegInfo>> FramesSegsInfo ,std::vector<int> StartFrames,std::vector<int> EndFrames,std::string YmlName,std::string videoFile = "");
	void BuildStateMap(OneFrameDSVLData * frame);
	~StateMapBuilder();
private:

	void GetPlaneEquation();//������Ƶ�ƽ�淽��
	void CalculateSegsCenter();//ѡ����������״̬��ͼ�ĵ�
	void DoBuildingStateMap();//����״̬��ͼ��������
	void ShowStateMap(int channel);//

	double DisToPlane(Point3fi *);//�㵽��ľ���


	MapContainer mapContainer;
	std::vector<std::map<int, SegInfo>> framesSegsInfo;
	double pixelSize = 0.2; //��λ����
	double mapSize = 200; //��λ����
	double planeEquation[4];//�����ű�֡��ȡ���ĵ��淽��ax+by+cz+d = 0
	OneFrameDSVLData* oneFrameData;
	Point3fi * PointsForStateMap;//��������������ռ��դ���ͼ�ĵ�
	int NumOfPointsForStateMap;//��������ռ��դ���ͼ�ĵ�ĸ���

	std::vector<int> startFrames;
	std::vector<int> endFrames;
	
	std::string ymlName;
	int curDataSeg;//��ǰ��������ݶ�


	Point3d shv;//�����״ﵽGPS����α궨���� shift
	Point3d shvG;//GPSתȫ������ shift
	MATRIX	RotLidar2GPS;//�����״ﵽGPS��ת����
	MATRIX Rot2Global;//ת����ȫ���������ת����

	cv::VideoWriter video;
};

