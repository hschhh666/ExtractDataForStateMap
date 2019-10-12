#include "DataManager.h"

////���캯������ȡtrans.log�ļ�������¼���к�����ӳ���ϵ
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
//		dictionary.insert(tmp_pari);//��¼���к�����ӳ���ϵ
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
////Ϊÿ����������Ա�ǩ������桢�ˡ�����
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
//				curLabel = frame->oneBlockDsvData[i].label[j*LINES + k];//curLabelʵ��ָ����prid�������еı�š���Ȼ�����ڵ�������ǲ��������еģ����Ե����curLabel����GROUND
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
//						oneFrameData->labels[curNum] = dictionary[curLabel];//��ӳ��ͼ���ҵ���ǰ���ж�Ӧ�������ǩ
//					}
//					else {
//						printf("You can see this information because there is a sequence doesn't exit in log file.\n");
//					}
//				}
//			}
//
//}
//
//��ͼ���ݽṹ��Ĭ�Ϲ��캯�����������ʼ����ͼ
MapContainer::MapContainer()
{
	imgSize = mapSize / pixelSize;
	cv::Mat tmp(cv::Size(imgSize, imgSize), CV_8UC3, cv::Scalar(0, 0, 0));
	map = tmp.clone();
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
	cv::imshow(imgname, map);
	cv::waitKey(key);
}
