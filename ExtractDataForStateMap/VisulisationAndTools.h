/*�����һ�п��ӻ������ɸò������*/
#pragma once
#include "define.h"

void ShowRangeImage(OneFrameDSVLData * frame);// ��ʾԭʼ���Ƶ�rangeImage������ʾ��Ч���ݡ���Ч����ָ������Ϊ0�ģ�
void ShowRangeImageWithSemantic(OneFrameDSVLData * frame, std::map<int,SegInfo> *SegsInfo);
void ShowPointCloud(Point3fi * frame, int num, std::string windownname);
void print3X3Matrix(MATRIX m);
