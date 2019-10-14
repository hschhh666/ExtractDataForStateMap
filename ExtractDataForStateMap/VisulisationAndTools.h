/*程序的一切可视化界面由该部分完成*/
#pragma once
#include "define.h"

void ShowRangeImage(OneFrameDSVLData * frame);// 显示原始点云的rangeImage（不显示无效数据。无效数据指反射率为0的）
void ShowRangeImageWithSemantic(OneFrameDSVLData * frame, std::map<int,SegInfo> *SegsInfo);
void ShowPointCloud(Point3fi * frame, int num, std::string windownname);
void print3X3Matrix(MATRIX m);
