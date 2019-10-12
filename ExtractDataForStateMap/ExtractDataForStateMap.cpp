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

	std::string dsvlFileName, seglogFileName, translogFileName, p40calibFileName,ogmFileName;
	ogmFileName = "";

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
	OGMBuilder ogm(p40calibFileName, ogmFileName);
	OneFrameDSVLData * oneFrameDSVData;

	for (oneFrameDSVData = dsvlFile.ReadOneFrameDSVL(); oneFrameDSVData != nullptr; oneFrameDSVData = dsvlFile.ReadOneFrameDSVL()) {
		ShowRangeImage(oneFrameDSVData);
		ogm.BuildOGM(oneFrameDSVData);
	}

	return 0;
}
