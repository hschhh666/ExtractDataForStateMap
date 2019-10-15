#include "inifile.h"
#include "define.h"

int main(int argc,char **argv)
{

	//读配置文件部分
	if (argc != 2) {//程序参数是配置文件
		printf("Error! Please add config file! \nPress any key to exit.\n");
		getchar();
		return (-1);
	}
	inifile::IniFile configfile;
	if (!configfile.load(argv[1])) {
		printf("Press any key to exit.\n");
		getchar();
		return(-2);
	}

	std::string dsvlFileName = "", seglogFileName = "", translogFileName = "", staticlogFileName = "", p40calibFileName = "",ogmFileName = "";

	if (!configfile.getValue("param", "dsvl", dsvlFileName)) {
		printf("Error! Cannot get parameter dsvl in config file.\nPress any key to exit.\n");
		getchar();
		return (-3);
	}
	if(!configfile.getValue("param", "seglog", seglogFileName)) {
		printf("Error! Cannot get parameter seglog in config file.\nPress any key to exit.\n");
		getchar();
		return (-3);
	}
	if(!configfile.getValue("param", "translog", translogFileName)) {
		printf("Error! Cannot get parameter translog in config file.\nPress any key to exit.\n");
		getchar();
		return (-3);
	}
	if (!configfile.getValue("param", "staticlog", staticlogFileName)) {
		printf("No static log file, so process all frames, else only process static frames.\n");
	}
	if(!configfile.getValue("param", "p40calib", p40calibFileName)) {
		printf("Error! Cannot get parameter p40calib in config file.\nPress any key to exit.\n");
		getchar();
		return (-3);
	}
	if (!configfile.getValue("param", "ogm", ogmFileName)) {
		printf("Donot save ogm building video. \n");
	}


	//主程序部分
	DSVLReader dsvlFile(dsvlFileName);
	StaticFrameLogReader staticFrameLog(staticlogFileName);
	SegAndTransLogReader segAndTransLog(seglogFileName, translogFileName);
	StateMapBuilder stateMapBuilder(p40calibFileName, segAndTransLog.FramesSegsInfo);

	//OGMBuilder ogm(p40calibFileName, ogmFileName);
	//DataClipper dataclipper(staticlogFileName,dsvlFile.frameNum);//输出分割后的数据log
	OneFrameDSVLData * oneFrameDSVData;


	int curFrame = 0;

	for (oneFrameDSVData = dsvlFile.ReadOneFrameDSVL(); oneFrameDSVData != nullptr; oneFrameDSVData = dsvlFile.ReadOneFrameDSVL()) {
		curFrame = oneFrameDSVData->curFrame;
		//ShowRangeImage(oneFrameDSVData);
		ShowRangeImageWithSemantic(oneFrameDSVData, &segAndTransLog.FramesSegsInfo.at(curFrame));


		stateMapBuilder.BuildStateMap(oneFrameDSVData);
		//ogm.BuildOGM(oneFrameDSVData);
		if (curFrame % 100 == 0)
			printf("Frame %d/%d\n", curFrame,(int)dsvlFile.totalframes-1);
	}

	return 0;
}
