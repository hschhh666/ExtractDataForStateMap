/*������̬ռ��դ���ͼ*/
#pragma once
#include "define.h"

class OGMBuilder {
public:
	OGMBuilder(std::string p40calibFile, std::string videoFile = "");
	void BuildOGM(OneFrameDSVLData * frame);//����OGM��ͼ
	void ShowOGM();

private:
	void pickUpPointsForStaticOGM();//ѡ����������OGM�ĵ㣬�����ų��ˡ����ĵ㣬�ų����������ߵĵ�
	void GetPlanEquation();//���ݵ��Ƽ���ƽ�淽�̣���ʵ���Ǽ������ķ���
	void DoBuildingOGM();//����ѡ������ĵ㹹��OGM
	double DisToPlane(Point3fi *);//����㵽��ľ���

	MapContainer mapContainer;
	double pixelSize = 0.2; //��λ����
	double mapSize = 200; //��λ����
	double planeEquation[4];//�����ű�֡��ȡ���ĵ��淽��ax+by+cz+d = 0
	OneFrameDSVLData* oneFrameData;
	Point3fi * PointsForOGM;//��������������ռ��դ���ͼ�ĵ�
	int NumOfPointsForOGM;//��������ռ��դ���ͼ�ĵ�ĸ���

	int fno;//frame number

	Point3d shv;//�����״ﵽGPS����α궨���� shift
	Point3d rot;//�����״ﵽGPS����α궨���� rotate
	MATRIX	RotLidar2GPS;
	Point3d shvG;//ת����ȫ������
	Point3d rotG;//ת����ȫ������
	MATRIX Rot2Global;//ת����ȫ���������ת����

	cv::VideoWriter video;

};