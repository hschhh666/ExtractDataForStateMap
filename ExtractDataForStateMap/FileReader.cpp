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

