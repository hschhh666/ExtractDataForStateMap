/*�����һ�п��ӻ������ɸò������*/
#pragma once
#include "define.h"

void ShowRangeImage(OneFrameDSVLData * frame);// ��ʾԭʼ���Ƶ�rangeImage������ʾ��Ч���ݡ���Ч����ָ������Ϊ0�ģ�
void StatisticLabels(OneFrameDSVLData * frame);//ͳ��ÿһ֡�в�ͬ��ǩ�ĵ�ĸ�����������㡢������ͱ���ע��������Ϣ�ĵ��ж��ٸ�
//void ShowRangeImage(PointsContainer * frame);//��ʾ��������Ϣ��rangeImage
//void ShowPointCloud(PointsContainer * frame);
void ShowPointCloud(Point3fi * frame, int num, std::string windownname);
void print3X3Matrix(MATRIX m);
