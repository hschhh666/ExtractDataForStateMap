#include "VisulisationAndTools.h"


// 显示原始点云的rangeImage（不显示无效数据。无效数据指反射率为0的）
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

void ShowRangeImageWithSemantic(OneFrameDSVLData * frame, std::map<int,SegInfo> *SegsInfo)
{

	std::map<int, SegInfo>::iterator mapiter;
	cv::Mat rangeImage(LINES, BLOCK_PER_FRAME*LINES_PER_BLOCK, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat resultImage;
	Point3fi *p;
	for (int i = 0; i < BLOCK_PER_FRAME; i++)
		for (int j = 0; j < LINES_PER_BLOCK; j++)
			for (int k = 0; k < LINES; k++) {
				p = &frame->oneBlockDsvData[i].points[j*LINES + k];
				if (!p->i)
					continue;
				double dis = sqrt(pow(p->x, 2) + pow(p->y, 2) + pow(p->z, 2));
				int x = i * LINES_PER_BLOCK + j;
				int y = k;
				int color = MIN(255, dis * 5.1);
				rangeImage.at<cv::Vec3b>(y, x)[0] = color;
				rangeImage.at<cv::Vec3b>(y, x)[1] = color;
				rangeImage.at<cv::Vec3b>(y, x)[2] = color;

				int curprid = frame->oneBlockDsvData[i].label[j*LINES + k];
				if (curprid < 10000) continue;
				mapiter = (*SegsInfo).find(curprid);//根据该点的prid在map中寻找它的语义标签

				if (mapiter != (*SegsInfo).end()) {
					int label = (*SegsInfo)[curprid].semanticLabel;//取出该点的语义信息
					if (label == CAR) {
						rangeImage.at<cv::Vec3b>(y, x)[0] = 255;
						rangeImage.at<cv::Vec3b>(y, x)[1] = 0;
						rangeImage.at<cv::Vec3b>(y, x)[2] = 0;
					}
					else if (label == PERSION) {
						rangeImage.at<cv::Vec3b>(y, x)[0] = 0;
						rangeImage.at<cv::Vec3b>(y, x)[1] = 255;
						rangeImage.at<cv::Vec3b>(y, x)[2] = 0;
					}
					else if (label == RIDER) {
						rangeImage.at<cv::Vec3b>(y, x)[0] = 0;
						rangeImage.at<cv::Vec3b>(y, x)[1] = 0;
						rangeImage.at<cv::Vec3b>(y, x)[2] = 255;
					}
				}
			}

	cv::resize(rangeImage, resultImage, cv::Size(1800, 200));
	cv::imshow("range image with semantic label", resultImage);
	char WaitKey = cv::waitKey(1);
	//if (WaitKey == 'z')
	//	cv::waitKey(0);
}



//画点云，注意坐标的定义
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


