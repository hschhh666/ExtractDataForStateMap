#include "DataManager.h"

////构造函数，读取trans.log文件，并记录序列和类别的映射关系
//PointsClassifyer::PointsClassifyer(std::string logFile)
//{
//	if (!dictionary.empty())
//		dictionary.clear();
//	FILE *fp;
//	std::pair<int, int> tmp_pari;
//	std::cout << "Log file is " << logFile.c_str() << std::endl;
//	fopen_s(&fp,logFile.c_str(), "r");
//	if (fp == NULL) {
//		printf("Error!!! Cannot open log file!\n");
//		getchar();
//		exit(-1);
//	}
//	else {
//		printf("Open log file succeed! Reading log file, it may takes a few secs...\n");
//	}
//	char oneLine[300];
//	int prid, label;
//	while (fgets(oneLine, 300, fp) != NULL) {
//		if (oneLine[0] != 'p') continue;
//		sscanf_s(oneLine, "prid=%d,%*d,%*d,%*d,%d", &prid, &label);
//		tmp_pari = std::make_pair(prid, label);
//		dictionary.insert(tmp_pari);//记录序列和类别的映射关系
//	}
//	if (dictionary.empty()) {
//		printf("Invalid log file, please check. Program exit.\n");
//		exit(-2);
//	}
//	printf("Reading log file finished, Program goes on.\n");
//	fclose(fp);
//
//	oneFrameData = NULL;
//	oneFrameData = new PointsContainer;
//	if (oneFrameData == NULL) {
//		printf("Allocate memory error! Program exit.\n");
//		exit(-3);
//	}
//}
//
////为每个点打上属性标签，如地面、人、车等
//void PointsClassifyer::DoClassify(OneFrameDSVLData* frame)
//{
//	int curLabel;
//	int curNum;
//	Point3fi *p1;
//	Point3fi *p2;
//	std::map<int, int>::iterator iter;
//	oneFrameData->ang.x = frame->ang.x;
//	oneFrameData->ang.y = frame->ang.y;
//	oneFrameData->ang.z = frame->ang.z;
//	oneFrameData->shv.x = frame->shv.x;
//	oneFrameData->shv.y = frame->shv.y;
//	oneFrameData->shv.z = frame->shv.z;
//
//	for (int i = 0; i < BLOCK_PER_FRAME; i++)
//		for (int j = 0; j < LINES_PER_BLOCK; j++)
//			for (int k = 0; k < LINES; k++) {
//				curNum = i * LINES*LINES_PER_BLOCK + j * LINES + k;
//				curLabel = frame->oneBlockDsvData[i].label[j*LINES + k];//curLabel实际指的是prid，即序列的编号。当然，对于地面而言是不存在序列的，所以地面的curLabel就是GROUND
//				p1 = & frame->oneBlockDsvData[i].points[j*LINES + k];
//				p2 = & oneFrameData->points[curNum];
//
//				p2->i = p1->i;
//				p2->x = p1->x;
//				p2->y = p1->y;
//				p2->z = p1->z;
//
//				if (curLabel == UNKNOWN) {
//					oneFrameData->sequenceNum[curNum] = 0;
//					oneFrameData->labels[curNum] = UNKNOWN;
//				}
//				if (curLabel == NONVALID or p1->i == 0) {
//					oneFrameData->sequenceNum[curNum] = 0;
//					oneFrameData->labels[curNum] = NONVALID;
//				}
//				if (curLabel == GROUND) {
//					oneFrameData->sequenceNum[curNum] = 0;
//					oneFrameData->labels[curNum] = GROUND;
//				}
//				if (curLabel == BACKGROUND) {
//					oneFrameData->sequenceNum[curNum] = 0;
//					oneFrameData->labels[curNum] = BACKGROUND;
//				}
//				if (curLabel >= 10000) {
//					iter = dictionary.find(curLabel);
//					if (iter != dictionary.end()) {
//						oneFrameData->sequenceNum[curNum] = curLabel;
//						oneFrameData->labels[curNum] = dictionary[curLabel];//在映射图中找到当前序列对应的语义标签
//					}
//					else {
//						printf("You can see this information because there is a sequence doesn't exit in log file.\n");
//					}
//				}
//			}
//
//}
//
//地图数据结构的默认构造函数，在这里初始化地图
MapContainer::MapContainer()
{
	imgSize = mapSize / pixelSize;
	cv::Mat tmp(cv::Size(imgSize, imgSize), CV_8UC3, cv::Scalar(0, 0, 0));
	map = tmp.clone();
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
	cv::imshow(imgname, map);
	cv::waitKey(key);
}
