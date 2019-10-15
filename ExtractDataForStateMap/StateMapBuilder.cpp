#include "StateMapBuilder.h"



StateMapBuilder::StateMapBuilder(std::string p40calibFile, std::vector<std::map<int, SegInfo>> FramesSegsInfo, std::vector<int> StartFrames, std::vector<int> EndFrames,std::string YmlName,std::string videoFile)
{
	curDataSeg = 0;
	framesSegsInfo = FramesSegsInfo;
	startFrames = StartFrames;
	endFrames = EndFrames;
	ymlName = YmlName;
	PointsForStateMap = new Point3fi[LINES*LINES_PER_BLOCK*BLOCK_PER_FRAME];
	
	Point3d rot;//�����״ﵽGPS����α궨���� rotate

	//������α궨�ļ�
	std::ifstream filePoint;
	char buff[200];
	filePoint.open(p40calibFile.c_str());
	if (filePoint.is_open()) {
		printf("Open calib file succeed.\n");
		filePoint.getline(buff, 200);
		float t1, t2, t3;
		sscanf_s(buff, "rot %f %f %f", &t1, &t2, &t3);
		rot.x = t1; rot.y = t2; rot.z = t3;
		filePoint.getline(buff, 200);
		sscanf_s(buff, "shv %f %f %f", &t1, &t2, &t3);
		shv.x = t1; shv.y = t2; shv.z = t3;
		printf("rot.x = %f, rot.y = %f, rot.z = %f\nshv.x = %f, shv.y = %f, shv.z = %f\n", rot.x, rot.y, rot.z, shv.x, shv.y, shv.z);
		printf("The unit of angle should be degree, not radian, please make sure it's correct. If it's right, just ignore this message.\n");
		printf("Stop to confirm calib file. Press any key to continue;\n");
		double coefficent = 3.1415926535 / 180.0;
		rot.x *= coefficent;//convert degree to radian
		rot.y *= coefficent;
		rot.z *= coefficent;
		createRotMatrix_ZYX(RotLidar2GPS, rot.x, rot.y, rot.z);//������ε���ת����
		//getchar();
	}
	else {
		printf("Open calib file failed. Program exit.\n");
		exit(-4);
	}

	//�Ƿ񱣴潨ͼ����
	if (videoFile != "")
		if (!video.open(videoFile, CV_FOURCC('X', 'V', 'I', 'D'), 20, cv::Size(600, 600))) {
			std::cout << "Cannot open video file " << videoFile << ", program exit.\n";
			exit(-4);
		}
}

void StateMapBuilder::BuildStateMap(OneFrameDSVLData * frame)
{
	if (endFrames.at(endFrames.size() - 1) < frame->curFrame) return;


	oneFrameData = frame;

	Point3d rotG;//ת����ȫ������
	shvG.x = oneFrameData->shv.x; shvG.y = oneFrameData->shv.y; shvG.z = oneFrameData->shv.z;
	rotG.x = oneFrameData->ang.x; rotG.y = oneFrameData->ang.y; rotG.z = oneFrameData->ang.z;
	createRotMatrix_ZYX(Rot2Global, rotG.x, rotG.y, -rotG.z);//������ת��ȫ�������µ���ת����

	if (oneFrameData->curFrame == startFrames.at(curDataSeg)) {
		mapContainer.leftUpCornerX = shvG.x - mapContainer.mapSize / 2;//ȡ��һ֡λ����Ϊ��ͼ�Ļ�׼����
		mapContainer.leftUpCornerY = shvG.y + mapContainer.mapSize / 2;
		mapContainer.InitMap();
	}

	//GetPlaneEquation();//������淽��
	CalculateSegsCenter();//ѡ����������״̬��ͼ�ĵ�
	DoBuildingStateMap();//����״̬��ͼ��������

}

void StateMapBuilder::CalculateSegsCenter()
{
	int curFrame = oneFrameData->curFrame;
	std::map<int, SegInfo>::iterator iter;
	for (iter = framesSegsInfo.at(curFrame).begin(); iter != framesSegsInfo.at(curFrame).end(); iter++) {
		if ((*iter).second.semanticLabel == PERSION) 
		{
			int iminx = (*iter).second.iminx;
			int iminy = (*iter).second.iminy;
			int imaxx = (*iter).second.imaxx;
			int imaxy = (*iter).second.imaxy;
			int segNum = 0;
			double cx = 0, cy = 0, cz = 0;
			for (int x = iminx;x<=imaxx;x++)
				for (int y = iminy; y <= imaxy; y++) {
					int i = x / LINES_PER_BLOCK;
					int j = (x%LINES_PER_BLOCK)*LINES + y;
					if (oneFrameData->oneBlockDsvData[i].label[j] == (*iter).first) {
						cx += oneFrameData->oneBlockDsvData[i].points[j].x;
						cy += oneFrameData->oneBlockDsvData[i].points[j].y;
						cz += oneFrameData->oneBlockDsvData[i].points[j].z;
						segNum++;
					}
				}
			cx /= segNum;
			cy /= segNum;
			cz /= segNum;
			(*iter).second.cx = cx;
			(*iter).second.cy = cy;
			(*iter).second.cz = cz;
			(*iter).second.segNum = segNum;
			(*iter).second.valid = true;
		}
	}
}

void StateMapBuilder::DoBuildingStateMap()
{
	int curFrame = oneFrameData->curFrame;
	int lastFrame = curFrame - 1;
	if (curFrame == 0)return;


	std::map<int, SegInfo>::iterator curIter,lastIter;
	for (curIter = framesSegsInfo.at(curFrame).begin(); curIter != framesSegsInfo.at(curFrame).end(); curIter++) {//������֡�е�����seg
		if ((*curIter).second.semanticLabel == PERSION)//ֻ����
		{
			int prid = (*curIter).first;//��ȡ��seg�����б��
			lastIter = framesSegsInfo.at(lastFrame).find(prid);//��ѯ��һ֡���Ƿ��б�seg
			if (lastIter == framesSegsInfo.at(lastFrame).end()) continue;
			if ((*lastIter).second.valid == false) continue;
			Point3d pc, pl;
			pc.x = (*curIter).second.cx; pc.y = (*curIter).second.cy; pc.z = (*curIter).second.cz;//��ȡ��֡��seg�����ĵ�����
			pl.x = (*lastIter).second.cx; pl.y = (*lastIter).second.cy; pl.z = (*lastIter).second.cz;//��ȡ��һ֡ͬһseg�����ĵ�����

			//ת��GPS������
			rotatePoint3d(pc, RotLidar2GPS);
			shiftPoint3d(pc, shv);
			rotatePoint3d(pl, RotLidar2GPS);
			shiftPoint3d(pl, shv);

			//ת��ȫ�������¡�ע�⣬������ܻ������⣬��Ϊ��һ֡��shift��rotate�õ��Ǳ�֡�ġ�������Ϊ���賵����ֹ�������������Ҳ����
			rotatePoint3d(pc, Rot2Global);
			shiftPoint3d(pc, shvG);
			rotatePoint3d(pl, Rot2Global);
			shiftPoint3d(pl, shvG);

			Point3d dirVector;
			dirVector.x = pc.x - pl.x;
			dirVector.y = pc.y - pl.y;
			dirVector.z = pc.z - pl.z;
			double moveDis = sqrt(dirVector.x*dirVector.x + dirVector.y*dirVector.y);
			double theta = acos(dirVector.x / moveDis) * 180 / CV_PI;
			if (dirVector.y < 0) theta = 360 - theta;
			theta += 22.5;
			theta = theta - ((int)(theta / 360.0))*360.0;
			int dir = theta / 45;

			if (moveDis > 0.1) {

				int iminx = (*curIter).second.iminx;
				int iminy = (*curIter).second.iminy;
				int imaxx = (*curIter).second.imaxx;
				int imaxy = (*curIter).second.imaxy;

				for (int x = iminx; x <= imaxx; x++)
					for (int y = iminy; y <= imaxy; y++) {
						int i = x / LINES_PER_BLOCK;
						int j = (x%LINES_PER_BLOCK)*LINES + y;
						if (oneFrameData->oneBlockDsvData[i].label[j] == (*curIter).first) {
							Point3d p;
							p.x = oneFrameData->oneBlockDsvData[i].points[j].x;
							p.y = oneFrameData->oneBlockDsvData[i].points[j].y;
							p.z = oneFrameData->oneBlockDsvData[i].points[j].z;

							rotatePoint3d(p, RotLidar2GPS);
							shiftPoint3d(p, shv);
							rotatePoint3d(p, Rot2Global);
							shiftPoint3d(p, shvG);
							
							int x, y;
							if (mapContainer.CoordinateConventer(p.x, p.y, x, y)) {
								mapContainer.StateMap.at<cv::Vec<float, 10>>(cv::Point(x, y))[dir] += 1;
								mapContainer.StateMap.at<cv::Vec<float, 10>>(cv::Point(x, y))[8] += 1;
							}
						}
					}
			}
		}
	}

	//ShowStateMap(0);
	//ShowStateMap(2);

	
	if (curFrame == endFrames.at(curDataSeg)) {
		//�����yaml�ļ�
		int x, y;
		double mapSize = 100;//meter ����ָ��ͼ��ʵ�ʴ�С�Ƕ�����,�����Գ���Ϊ���ĵ�mapSize x mapSize ƽ���״�С�ĵ�ͼ
		int imgSize = mapSize / mapContainer.pixelSize;//pixel ����ָ�ڵ�ͼ�ߴ�ΪmapSize��ʱ��Ӧ����ʵ��ͼ���ش�С
		if (mapContainer.CoordinateConventer(shvG.x, shvG.y, x, y)) {
			x = x - imgSize / 2;
			y = y - imgSize / 2;
			if (x < 0) x = 0;
			if (y < 0)y = 0;
			if (x + imgSize > mapContainer.imgSize) x = mapContainer.imgSize - imgSize - 1;
			if (y + imgSize > mapContainer.imgSize) y = mapContainer.imgSize - imgSize - 1;
			cv::Rect rect(x, y, imgSize, imgSize);
			double shvGx, shvGy;
			mapContainer.CoordinateConventer(x, y, shvGx, shvGy);
			cv::Mat img = mapContainer.StateMap(rect);
			std::string filename = ymlName + "_part" + std::to_string(curDataSeg) + ".yml";
			cv::FileStorage fs(filename, cv::FileStorage::WRITE);
			fs << "leftUpCornerX" << shvGx;
			fs << "leftUpCornerY" << shvGy;
			fs << "pixelSize" << mapContainer.pixelSize;
			fs << "imgSize" << imgSize;
			fs << "channelNum" << 10;
			fs << "startFrame" << startFrames.at(curDataSeg);
			fs << "endFrame" << endFrames.at(curDataSeg);
			fs << "framesNum" << endFrames.at(curDataSeg) - startFrames.at(curDataSeg) + 1;
			fs << "img" << img.reshape(1);
			fs.release();
			std::cout << "Out put yaml file to " << filename << std::endl;
		}


		//���³�ʼ����ͼ
		mapContainer.InitMap();
		curDataSeg++;
	}

}

StateMapBuilder::~StateMapBuilder()
{
}

void StateMapBuilder::GetPlaneEquation()
{
	double *x;
	double *y;
	double *z;
	int totalNum = LINES * LINES_PER_BLOCK*BLOCK_PER_FRAME;
	int GroundNum = 0;
	for (int i = 0; i < BLOCK_PER_FRAME; i++)
		for (int j = 0; j < PTNUM_PER_BLOCK; j++)
			if (oneFrameData->oneBlockDsvData[i].label[j] == GROUND)
				GroundNum++;

	x = new double[GroundNum];
	y = new double[GroundNum];
	z = new double[GroundNum];
	int GroundPointCounter = 0;

	for (int i = 0; i < BLOCK_PER_FRAME; i++)
		for (int j = 0; j < PTNUM_PER_BLOCK; j++)
			if (oneFrameData->oneBlockDsvData[i].label[j] == GROUND) {
				x[GroundPointCounter] = oneFrameData->oneBlockDsvData[i].points[j].x;
				y[GroundPointCounter] = oneFrameData->oneBlockDsvData[i].points[j].y;
				z[GroundPointCounter] = oneFrameData->oneBlockDsvData[i].points[j].z;
				GroundPointCounter++;
			}

	Calculate_Plane(GroundNum, x, y, z, 0, planeEquation);
	delete[] x;
	delete[] y;
	delete[] z;
}



void StateMapBuilder::ShowStateMap(int channel)
{

	static cv::Mat channels[10];
	cv::split(mapContainer.StateMap, channels);

	int x, y;
	double mapSize = 100;//meter ����ָ��ͼ��ʵ�ʴ�С�Ƕ�����
	int finalImgSize = 600;//pixel ����ָ���տ��ӻ������ĵ�ͼ�Ƕ�������
	int imgSize = mapSize / mapContainer.pixelSize;//pixel ����ָ�ڵ�ͼ�ߴ�ΪmapSize��ʱ��Ӧ����ʵ��ͼ���ش�С
	if (mapContainer.CoordinateConventer(shvG.x, shvG.y, x, y)) {
		x = x - imgSize / 2;
		y = y - imgSize / 2;
		if (x < 0) x = 0;
		if (y < 0)y = 0;
		if (x + imgSize > mapContainer.imgSize) x = mapContainer.imgSize - imgSize - 1;
		if (y + imgSize > mapContainer.imgSize) y = mapContainer.imgSize - imgSize - 1;
		cv::Mat img = channels[channel](cv::Rect(x, y, imgSize, imgSize));
		cv::resize(img, img, cv::Size(finalImgSize, finalImgSize));
		cv::imshow("state map channel " + std::to_string(channel), img);
		cv::waitKey(1);

		if(video.isOpened())
			video.write(img);
	}
}

double StateMapBuilder::DisToPlane(Point3fi *p)
{
	//ֱ�߷��̵���ʽΪax+by+cz+d = 0 ,����equation��0,1,2,3�ֱ��Ӧa,b,c,d
	//point��0,1,2�ֱ��Ӧx,y,z
	double dis;
	double a = planeEquation[0];
	double b = planeEquation[1];
	double c = planeEquation[2];
	double d = planeEquation[3];
	double denominator = a * a + b * b + c * c;
	denominator = sqrt(denominator);
	dis = a * p->x + b * p->y + c * p->z + d;
	dis = dis / denominator;
	return fabs(dis);
}


