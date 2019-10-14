#include "FileReader.h"

//�Զ����Ƹ�ʽ��DSVL�ļ����ļ�ָ���У�������ļ���С������֡��
DSVLReader::DSVLReader(std::string file)
{
	oneFrameDSVLData = new OneFrameDSVLData;

	fileName = file;
	std::cout << "DSVL file is " << fileName << std::endl;
	filePointer.open(fileName.c_str(), std::ifstream::binary);//�Զ����ƴ�

	if (filePointer.is_open()) {	
		filePointer.seekg(0, std::ios_base::end);//���ļ�ָ��ָ���ļ�ĩβ�����ڼ����ļ���С
		fileLens = filePointer.tellg();//�����ļ�ָ�����ڵ�λ��(�ֽ�����������ļ���С
		printf("Open DSVL file succeed! File size = %f GB, %f KB.  ", fileLens / 1024 / 1024 / 1024.0, fileLens/1024.0);
		filePointer.seekg(0, std::ios_base::beg);//���ļ�ָ�������ļ���ʼ
		frameNum = (double)fileLens / (double)sizeof(OneBlockDSVLData) / (double)BLOCK_PER_FRAME;
		printf("Total frames = %f \n", frameNum);//���ݼ���֡���Ƿ�Ϊ�����ж��ļ��Ƿ���/��Ч
		if (( fileLens % (sizeof(OneBlockDSVLData) * BLOCK_PER_FRAME))) {
			printf("Warning!!! DSVL file maybe broken!!! Program will still run, but there may be some errors. Press any key to continue.\n");
			getchar();
		}
			
	}
	else
		std::cout << "Open DSVL file failed!!! File doesn't exit.\n";
}

DSVLReader::~DSVLReader()
{
	if (filePointer.is_open())
		filePointer.close();
	delete oneFrameDSVLData;
}

void DSVLReader::CloseFile()
{
	if(filePointer.is_open())
		filePointer.close();
}

//��ȡһ֡��DSVL�ļ�,һ֡DSVL�ļ���BLOCK_PER_FRAME��block���
OneFrameDSVLData* DSVLReader::ReadOneFrameDSVL()
{
	long long curfp = filePointer.tellg();
	if (curfp >= fileLens)
		return nullptr;
	oneFrameDSVLData->ang.x = 0;
	oneFrameDSVLData->ang.y = 0;
	oneFrameDSVLData->ang.z = 0;
	oneFrameDSVLData->shv.x = 0;
	oneFrameDSVLData->shv.y = 0;
	oneFrameDSVLData->shv.z = 0;

	for (int i = 0; i < BLOCK_PER_FRAME; i++) {
		filePointer.read((char *)&oneFrameDSVLData->oneBlockDsvData[i], sizeof(OneBlockDSVLData));//��ȡһ��block����
		if (sizeof(OneBlockDSVLData) != filePointer.gcount()) {//����Ƿ��������ȷsize������
			double tmp = filePointer.gcount();
			printf("Read one frame/block data error! Its size should be %d B, but it is %d B. Program exit.", sizeof(OneBlockDSVLData), filePointer.gcount());
			return nullptr;
		}
		oneFrameDSVLData->ang.x += oneFrameDSVLData->oneBlockDsvData[i].ang.x;
		oneFrameDSVLData->ang.y += oneFrameDSVLData->oneBlockDsvData[i].ang.y;
		oneFrameDSVLData->ang.z += oneFrameDSVLData->oneBlockDsvData[i].ang.z;
		oneFrameDSVLData->shv.x += oneFrameDSVLData->oneBlockDsvData[i].shv.x;
		oneFrameDSVLData->shv.y += oneFrameDSVLData->oneBlockDsvData[i].shv.y;
		oneFrameDSVLData->shv.z += oneFrameDSVLData->oneBlockDsvData[i].shv.z;
	}
	oneFrameDSVLData->ang.x /= BLOCK_PER_FRAME;//���㱾֡������block��nav��Ϣƽ��ֵ
	oneFrameDSVLData->ang.y /= BLOCK_PER_FRAME;
	oneFrameDSVLData->ang.z /= BLOCK_PER_FRAME;
	oneFrameDSVLData->shv.x /= BLOCK_PER_FRAME;
	oneFrameDSVLData->shv.y /= BLOCK_PER_FRAME;
	oneFrameDSVLData->shv.z /= BLOCK_PER_FRAME;

	return oneFrameDSVLData;
}

StaticFrameLogReader::StaticFrameLogReader(std::string filename)
{

	if (filename != "") {
		FILE *fp;
		fp = fopen(filename.c_str(), "r");
		if (fp == NULL) {
			printf("Error! cannot open static log file.\nPress any key to exit.\n");
			getchar();
			exit(-5);
		}
		
		char oneline[200];
		int startframe, endframe;
		while (fgets(oneline, 200, fp) != NULL) {
			if (sscanf(oneline, "%d %d", &startframe, &endframe) == 2) {
				startFrames.push_back(startframe);
				endFrames.push_back(endframe);
			}
		}
		fclose(fp);
	}
}



SegAndTransLogReader::SegAndTransLogReader(std::string seglogFileName, std::string translogFileName)
{
	//���ļ�����
	FILE *seglogfp, *translogfp;
	seglogfp = fopen(seglogFileName.c_str(), "r");
	if (seglogfp == NULL) {
		printf("Error! cannot open seg log file.\nPress any key to exit.\n");
		getchar();
		exit(-5);
	}
	translogfp = fopen(translogFileName.c_str(), "r");
	if (translogfp == NULL) {
		printf("Error! cannot open trans log file.\nPress any key to exit.\n");
		getchar();
		exit(-5);
	}

	//���ļ�����
	char oneline[200];
	int fno, rno,  icx, icy, prid, ptnum, iminx, iminy, imaxx, imaxy;
	double cx, cy, cz, rng, wid, hei;

	int lastfno = 1;

	std::map<int, SegInfo> SegsInfo;
	std::map<int, int> dictionary;
	std::map<int, int>::iterator iter;

	//��trans log���ҽ���prid��semanticLabel��ӳ���ϵ
	

	int semanticLabel;
	while (fgets(oneline, 200, translogfp) != NULL) {
		if (sscanf(oneline, "prid=%d,%*d,%*d,%*d,%d", &prid, &semanticLabel) == 2) {
			std::pair<int, int> pair(prid, semanticLabel);
			dictionary.insert(pair);
		}
	}

	SegInfo tmpSegInfo(-1, -1, -1, -1, -1, -1, false);//����seg log��frame 1��ʼ������Ϊ�˱�֤vector���±����õ���fno�������ʼ���һ����Ч����
	std::pair<int, SegInfo> tmpair(-1, tmpSegInfo);
	SegsInfo.insert(tmpair);
	FramesSegsInfo.push_back(SegsInfo);
	SegsInfo.clear();

	//��seg log���Ҳ�ѯprid��Ӧ��semanticLabel
	std::cout << "Mapping segmentation ID and semantic label, it may take some time...\n";
	while (fgets(oneline, 200, seglogfp)!=NULL) {
		if (sscanf(oneline, "%d,%d,%lf,%lf,%lf,%d,%d,%d,%d,%lf,%d,%d,%d,%d,%lf,%lf", &fno, &rno, &cx, &cy, &cz, &icx, &icy, &prid, &ptnum, &rng, &iminx, &iminy, &imaxx, &imaxy, &wid, &hei) == 16) {

			SegInfo segInfo(prid, iminx, iminy, imaxx, imaxy, 0);//semanticLabelȫ��ʼ��Ϊ0����unlabeled
			iter = dictionary.find(prid);
			if (iter != dictionary.end()) 
				segInfo.semanticLabel = dictionary[prid];

			if (lastfno != fno) {//��������µ�һ֡����ô�Ͱ���һ֡������Segs��push��vector�У�������һ֡����Ϣ
				FramesSegsInfo.push_back(SegsInfo);
				SegsInfo.clear();
			}
			SegsInfo.insert(std::make_pair(prid,segInfo));//����seg���뵽��ǰ֡��Segs��map��
			lastfno = fno;
		}
	}
	std::cout <<"Mapping finished. Continue process.\n ";

	FramesSegsInfo.push_back(SegsInfo);//push���һ֡
	
	fclose(seglogfp);
	fclose(translogfp);

}
