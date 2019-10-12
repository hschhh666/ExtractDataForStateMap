#include "VisulisationAndTools.h"


// ��ʾԭʼ���Ƶ�rangeImage������ʾ��Ч���ݡ���Ч����ָ������Ϊ0�ģ�
void ShowRangeImage(OneFrameDSVLData * frame) {
	cv::Mat rangeImage(LINES,BLOCK_PER_FRAME*LINES_PER_BLOCK, CV_8UC1,cv::Scalar(0));
	cv::Mat resultImage;
	Point3fi *p;
	for (int i= 0;i<BLOCK_PER_FRAME;i++)
		for (int j=0;j<LINES_PER_BLOCK;j++)
			for (int k = 0; k < LINES; k++) {
				p = &frame->oneBlockDsvData[i].points[j*LINES + k];
				if (!p->i)
					continue;
				double dis = sqrt(pow(p->x, 2) + pow(p->y, 2) + pow(p->z, 2));
				int x = i * LINES_PER_BLOCK + j;
				int y = k;
				rangeImage.at<uchar>(y, x) = MIN(255, dis * 5.1);
			}

	cv::resize(rangeImage, resultImage, cv::Size(1800, 200));
	cv::imshow("range image", resultImage);
	char WaitKey = cv::waitKey(1);
	//if (WaitKey == 'z')
	//	cv::waitKey(0);
	
}

//ͳ��ÿһ֡�в�ͬ��ǩ�ĵ�ĸ�����������㡢������ͱ���ע��������Ϣ�ĵ��ж��ٸ�
void StatisticLabels(OneFrameDSVLData * frame)
{
	enum LabelName{UnKnow, NonValid, Ground, BackGround, Target};//targetָ��ǩ����10000�����ݣ�������ע������Ϣ�����ݣ����˵���ͱ�����
	int Labels[5];
	for (int i = 0; i < 5; i++) Labels[i] = 0;
	Point3fi *p;
	int curLabel;
	for (int i = 0; i < BLOCK_PER_FRAME; i++)
		for (int j = 0; j < LINES_PER_BLOCK; j++)
			for (int k = 0; k < LINES; k++) {
				curLabel = frame->oneBlockDsvData[i].label[j*LINES + k];
				if (curLabel == UNKNOWN)		Labels[UnKnow]++;
				if (curLabel == NONVALID)	Labels[NonValid]++;
				if (curLabel == GROUND)	Labels[Ground]++;
				if (curLabel == BACKGROUND)	Labels[BackGround]++;
				if (curLabel > 10000)	Labels[Target]++;
			}
	int totalNum = Labels[0] + Labels[1] + Labels[2] + Labels[3] + Labels[4];
	printf("totalNum = %d , it should be %d \n", totalNum, BLOCK_PER_FRAME*LINES_PER_BLOCK*LINES);
	printf("UnKnow = %d\nNonValid = %d\nGround = %d\nBackGround = %d\nTarget = %d\n\n", 
		Labels[0], Labels[1], Labels[2], Labels[3], Labels[4]);
	//���ݱ�����ͳ�ƣ�����������ֻ��UnKnow Ground Target���ֱ�ǩ����������Ч���ݺͱ������ݡ�
}
//
//void ShowRangeImage(PointsContainer * frame)
//{
//	int totalNum = LINES * LINES_PER_BLOCK*BLOCK_PER_FRAME;
//	int imgWidth = LINES_PER_BLOCK * BLOCK_PER_FRAME;
//	int imgHeight = LINES;
//	cv::Mat img(imgHeight, imgWidth,CV_8UC3, cv::Scalar(0, 0, 0));
//	cv::Mat resultImg;
//	Point3fi *p;
//	double dis;
//	for (int i = 0; i < totalNum; i++) {
//		int x = i % imgHeight;
//		int y = i / imgHeight;
//		p = &frame->points[i];
//		dis = sqrt(pow(p->x, 2) + pow(p->y, 2) + pow(p->z, 2));
//
//
//		uchar gray = MIN(255, dis * 10);
//		img.at<cv::Vec3b>(x, y) = {gray,gray ,gray };
//		if (frame->labels[i] == PERSION) img.at<cv::Vec3b>(x, y) = {0,0,255};
//		if (frame->labels[i] == CAR) img.at<cv::Vec3b>(x, y) = { 0,255,0 };
//		if (frame->labels[i] == RIDER) img.at<cv::Vec3b>(x, y) = { 255,0,0 };
//		if (frame->labels[i] == FOROGM) img.at<cv::Vec3b>(x, y) = { 255,255,0 };
//	}
//	cv::resize(img, resultImg, cv::Size(1800, 200));
//	cv::imshow("Semantic image", resultImg);
//	char WaitKey = cv::waitKey(1);
//}
//
////������������Ϣ�ĵ���
//void ShowPointCloud(PointsContainer * frame)
//{
//	double realSize = 80;//meter
//	double pixSize = 0.1;//meter
//	int imgSize = realSize / pixSize;
//	cv::Mat img(cv::Size(imgSize, imgSize), CV_8UC3, cv::Scalar(0, 0, 0));
//	int totalNum = LINES * LINES_PER_BLOCK*BLOCK_PER_FRAME;
//	Point3fi *p;
//	int curLabel;
//	int x, y;
//	for (int i = 0; i < totalNum; i++) {
//		p = &frame->points[i];
//		curLabel = frame->labels[i];
//		x = imgSize / 2 + (-p->x / pixSize);
//		y = imgSize / 2 - (-p->y / pixSize);
//		if(curLabel == PERSION)
//			cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);
//		else if (curLabel == CAR)
//			cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1);
//		else if (curLabel == RIDER)
//			cv::circle(img, cv::Point(x, y), 1, cv::Scalar(255, 0, 0), -1);
//		else if(curLabel == FOROGM)
//			cv::circle(img, cv::Point(x, y), 1, cv::Scalar(255, 255, 255), -1);
//	}
//	cv::imshow("point cloud with semantic", img);
//	cv::waitKey(1);
//}

//�����ƣ�ע������Ķ���
void ShowPointCloud(Point3fi * frame, int num, std::string windowname)
{
	double realSize = 80;//meter
	double pixSize = 0.1;//meter
	int imgSize = realSize / pixSize;
	cv::Mat img(cv::Size(imgSize, imgSize), CV_8UC3, cv::Scalar(0, 0, 0));
	Point3fi *p;
	int x, y;
	for (int i = 0; i < num; i++) {
		p = &frame[i];
		x = imgSize/2 + (p->x / pixSize);
		y = imgSize/2 - (p->y / pixSize);
		cv::circle(img, cv::Point(x, y), 1, cv::Scalar(255, 255, 255), -1);
	}
	cv::imshow(windowname, img);
	cv::waitKey(1);
}

void print3X3Matrix(MATRIX m)
{
	for (int i = 0; i < 3; i++)
		printf("%-10.3f %-10.3f %-10.3f\n", m[i][0], m[i][1],m[i][2]);
	printf("\n");
}


