#include"OccupiedGridMapBuilder.h"

//Ĭ�Ϲ��캯����Ϊ���ڹ���ռ��դ���ͼ�ĵ���ָ������ڴ�
OGMBuilder::OGMBuilder(std::string p40calibFile, std::string videoFile)
{
	PointsForOGM = new Point3fi[LINES*LINES_PER_BLOCK*BLOCK_PER_FRAME];
	fno = 0;
	
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


//����ռ��դ���ͼ�������������ȼ����ƽ�淽�̣�Ȼ����ݵ㵽����ĸ߶Ⱥ͵�ı�ǩ��ѡ���ڹ���ռ��դ���ͼ�ĵ㣬��󹹽�ռ��դ���ͼ��
void OGMBuilder::BuildOGM(OneFrameDSVLData * frame)
{
	oneFrameData = frame;
	Point3d  rotG;//ת����ȫ������	
	shvG.x = oneFrameData->shv.x; shvG.y = oneFrameData->shv.y; shvG.z = oneFrameData->shv.z;
	rotG.x = oneFrameData->ang.x; rotG.y = oneFrameData->ang.y; rotG.z = oneFrameData->ang.z;
	createRotMatrix_ZYX(Rot2Global, rotG.x, rotG.y, -rotG.z);//������ת��ȫ�������µ���ת����
	
	if (fno == 0) {
		mapContainer.leftUpCornerX = shvG.x - mapContainer.mapSize / 2;//ȡ��һ֡λ����Ϊ��ͼ�Ļ�׼����
		mapContainer.leftUpCornerY = shvG.y + mapContainer.mapSize / 2;
	}


	GetPlanEquation();//������淽��
	pickUpPointsForStaticOGM();//����һ������ѡ�����ڹ���ռ��դ���ͼ�ĵ�
	DoBuildingOGM();//����ռ��դ���ͼ��������

	fno++;
}

//���ݵ㵽����ĸ߶Ⱥ͵�ı�ǩ��ѡ����������ռ��դ���ͼ�ĵ�
void OGMBuilder::pickUpPointsForStaticOGM()
{
	NumOfPointsForOGM = 0;
	Point3fi * p1;
	int totalNum = LINES * LINES_PER_BLOCK*BLOCK_PER_FRAME;

	for (int i = 0; i < BLOCK_PER_FRAME; i++)
		for (int j = 0; j < PTNUM_PER_BLOCK; j++){
			if (oneFrameData->oneBlockDsvData[i].label[j] == GROUND) continue;
			p1 = &oneFrameData->oneBlockDsvData[i].points[j];
			double dis = DisToPlane(p1);
			if ((dis > 2.0) || (dis < 0.2)) continue;//�������������Զ�ĵ㲻Ҫ
			PointsForOGM[NumOfPointsForOGM].x = p1->x;
			PointsForOGM[NumOfPointsForOGM].y = p1->y;
			PointsForOGM[NumOfPointsForOGM].z = p1->z;
			PointsForOGM[NumOfPointsForOGM].i = p1->i;
			NumOfPointsForOGM++;
		}
}

//����ռ��դ���ͼ��������
void OGMBuilder::DoBuildingOGM()
{
	Point3fi *p1;
	Point3d p;
	//ʹ��PointsForOGM����ռ��դ���ͼ
	Point3fi * transferdPoints = new Point3fi[NumOfPointsForOGM];
	int x, y;
	for (int i = 0; i < NumOfPointsForOGM; i++) {
		p1 = &PointsForOGM[i];
		p.x = p1->x; p.y = p1->y; p.z = p1->z;
		rotatePoint3d(p, RotLidar2GPS);
		shiftPoint3d(p, shv);//����������ת����GPS����
		p1 = &transferdPoints[i];
		p1->x = p.x; p1->y = p.y; p1->z = p.z;
		rotatePoint3d(p, Rot2Global);
		shiftPoint3d(p, shvG);//���ֲ�����ת����ȫ������
		uchar prob;
		if (mapContainer.CoordinateConventer(p.x, p.y, x, y)) {
			prob = mapContainer.OGM.at<cv::Vec3b>(cv::Point(x, y))[0];
			if (prob < 255)
			{
				prob++;
				mapContainer.OGM.at<cv::Vec3b>(cv::Point(x, y)) = { prob,prob,prob };
			}
				
			else
				mapContainer.OGM.at<cv::Vec3b>(cv::Point(x, y)) = { 255,255,255 };
		}
	}
	ShowOGM();
	ShowPointCloud(transferdPoints,NumOfPointsForOGM,"Vehicle head up point cloud");
	delete[] transferdPoints;

}


void OGMBuilder::ShowOGM()
{
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
		cv::Mat img = mapContainer.OGM(cv::Rect(x,y, imgSize, imgSize));
		cv::resize(img, img, cv::Size(finalImgSize, finalImgSize));
		cv::imshow("OGM", img);
		cv::waitKey(1);
		video.write(img);
	}

}

//���ݵ������ƽ�棬����ƽ�淽��ax+by+zc+d = 0;
void OGMBuilder::GetPlanEquation()
{
	double *x;
	double *y;
	double *z;
	int totalNum = LINES * LINES_PER_BLOCK*BLOCK_PER_FRAME;
	int GroundNum = 0;
	for(int i=0;i<BLOCK_PER_FRAME;i++)
		for (int j=0;j<PTNUM_PER_BLOCK;j++)
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

//����㵽ƽ��ľ���
double OGMBuilder::DisToPlane(Point3fi *p)
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