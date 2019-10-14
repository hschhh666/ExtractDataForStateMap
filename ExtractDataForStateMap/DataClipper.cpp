#include "DataClipper.h"

DataClipper::DataClipper(std::string staticlogFileName, int framenum)
{	
	staticlogfp.open(staticlogFileName.c_str(), std::ios::out);
	totalFrameNum = framenum;

}

DataClipper::~DataClipper()
{
	staticlogfp.close();
}

void DataClipper::getStaticFrames(OneFrameDSVLData * oneFrameDSVLData)
{
	double curPosition = oneFrameDSVLData->shv.x + oneFrameDSVLData->shv.y + oneFrameDSVLData->shv.z;
	if ((startFrame >= 0) && (FrameCounter == totalFrameNum - 1)) {
		SegCounter++;
		char tmp[200];
		sprintf(tmp, "Static seg %d, from frame %d to frame %d. Total frame = %d, duration = %.1f s\n", SegCounter, startFrame, endFrame, 1 + endFrame - startFrame, (1 + endFrame - startFrame) / 10.0);

		staticlogfp << std::string(tmp);
		staticlogfp << startFrame << " " << endFrame << "\n";
		printf("%s", tmp);
		staticlogfp.flush();
	}


	if (fabs(curPosition - basePosition) < 0.1){
		if (startFrame < 0) 
			startFrame = FrameCounter;
		endFrame = FrameCounter;
	}
	else {
		if (((startFrame >= 0) && (endFrame - startFrame > cacheSize))) {
			SegCounter++;
			char tmp[200];
			sprintf(tmp, "Static seg %d, from frame %d to frame %d. Total frame = %d, duration = %.1f s\n", SegCounter, startFrame, endFrame, 1 + endFrame - startFrame, (1 + endFrame - startFrame) / 10.0);

			staticlogfp << std::string(tmp);
			staticlogfp << startFrame << " " << endFrame << "\n";
			printf("%s",tmp);
			staticlogfp.flush();
		}
		//重新初始化位置和始末帧
		basePosition = curPosition;
		startFrame = -1;
		endFrame = -1;
	}
	FrameCounter++;
}
