#include "FileReader.h"

//以二进制格式打开DSVL文件到文件指针中，并检查文件大小，计算帧数
DSVLReader::DSVLReader(std::string file)
{
	oneFrameDSVLData = new OneFrameDSVLData;

	fileName = file;
	std::cout << "DSVL file is " << fileName << std::endl;
	filePointer.open(fileName.c_str(), std::ifstream::binary);//以二进制打开

	if (filePointer.is_open()) {	
		filePointer.seekg(0, std::ios_base::end);//将文件指针指向文件末尾，用于计算文件大小
		fileLens = filePointer.tellg();//返回文件指针所在的位置(字节数），检查文件大小
		printf("Open DSVL file succeed! File size = %f GB, %f KB.  ", fileLens / 1024 / 1024 / 1024.0, fileLens/1024.0);
		filePointer.seekg(0, std::ios_base::beg);//将文件指针置于文件开始
		frameNum = (double)fileLens / (double)sizeof(OneBlockDSVLData) / (double)BLOCK_PER_FRAME;
		printf("Total frames = %f \n", frameNum);//根据激光帧数是否为整数判断文件是否损坏/有效
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

//读取一帧的DSVL文件,一帧DSVL文件由BLOCK_PER_FRAME个block组成
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
		filePointer.read((char *)&oneFrameDSVLData->oneBlockDsvData[i], sizeof(OneBlockDSVLData));//读取一个block数据
		if (sizeof(OneBlockDSVLData) != filePointer.gcount()) {//检查是否读到了正确size的数据
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
	oneFrameDSVLData->ang.x /= BLOCK_PER_FRAME;//计算本帧中所有block的nav信息平均值
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
	//打开文件部分
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

	//读文件部分
	char oneline[200];
	int fno, rno,  icx, icy, prid, ptnum, iminx, iminy, imaxx, imaxy;
	double cx, cy, cz, rng, wid, hei;

	int lastfno = 1;

	std::map<int, SegInfo> SegsInfo;
	std::map<int, int> dictionary;
	std::map<int, int>::iterator iter;

	//读trans log，且建立prid和semanticLabel的映射关系
	

	int semanticLabel;
	while (fgets(oneline, 200, translogfp) != NULL) {
		if (sscanf(oneline, "prid=%d,%*d,%*d,%*d,%d", &prid, &semanticLabel) == 2) {
			std::pair<int, int> pair(prid, semanticLabel);
			dictionary.insert(pair);
		}
	}

	SegInfo tmpSegInfo(-1, -1, -1, -1, -1, -1, false);//由于seg log从frame 1开始，所以为了保证vector的下标正好等于fno，所以最开始填充一个无效数据
	std::pair<int, SegInfo> tmpair(-1, tmpSegInfo);
	SegsInfo.insert(tmpair);
	FramesSegsInfo.push_back(SegsInfo);
	SegsInfo.clear();

	//读seg log，且查询prid对应的semanticLabel
	std::cout << "Mapping segmentation ID and semantic label, it may take some time...\n";
	while (fgets(oneline, 200, seglogfp)!=NULL) {
		if (sscanf(oneline, "%d,%d,%lf,%lf,%lf,%d,%d,%d,%d,%lf,%d,%d,%d,%d,%lf,%lf", &fno, &rno, &cx, &cy, &cz, &icx, &icy, &prid, &ptnum, &rng, &iminx, &iminy, &imaxx, &imaxy, &wid, &hei) == 16) {

			SegInfo segInfo(prid, iminx, iminy, imaxx, imaxy, 0);//semanticLabel全初始化为0，即unlabeled
			iter = dictionary.find(prid);
			if (iter != dictionary.end()) 
				segInfo.semanticLabel = dictionary[prid];

			if (lastfno != fno) {//如果进入新的一帧，那么就把上一帧的所有Segs给push到vector中，并且上一帧的信息
				FramesSegsInfo.push_back(SegsInfo);
				SegsInfo.clear();
			}
			SegsInfo.insert(std::make_pair(prid,segInfo));//将此seg插入到当前帧的Segs的map中
			lastfno = fno;
		}
	}
	std::cout <<"Mapping finished. Continue process.\n ";

	FramesSegsInfo.push_back(SegsInfo);//push最后一帧
	
	fclose(seglogfp);
	fclose(translogfp);

}
