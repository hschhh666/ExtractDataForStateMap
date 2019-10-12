/*程序的一切可视化界面由该部分完成*/
#pragma once
#include "define.h"

void ShowRangeImage(OneFrameDSVLData * frame);// 显示原始点云的rangeImage（不显示无效数据。无效数据指反射率为0的）
void StatisticLabels(OneFrameDSVLData * frame);//统计每一帧中不同标签的点的个数，即地面点、背景点和被标注了语音信息的点有多少个
//void ShowRangeImage(PointsContainer * frame);//显示带语义信息的rangeImage
//void ShowPointCloud(PointsContainer * frame);
void ShowPointCloud(Point3fi * frame, int num, std::string windownname);
void print3X3Matrix(MATRIX m);
