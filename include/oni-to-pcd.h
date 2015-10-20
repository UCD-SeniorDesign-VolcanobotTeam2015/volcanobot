/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.h
Description:	Reads an oni file recorded using the Openni2 library and outputs point clouds (pcd files)
*/

#ifndef _ONI_TO_PCD
#define _ONI_TO_PCD

#include <vector>
#include "pcl/io/openni2_grabber.h"
#include "pcl/point_cloud.h"

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;


extern const char* const oniFileBeingRead;

namespace vba {
	/*
	oni to pcd namespace
	*/
	namespace oni2pcd {
		extern int totalFrames,
			framesToRead,
			currentFrame,
			currentReadFrame,
			frameSkip;

		char* pcdWriteDirPath = NULL;

		const int DEFAULT_FRAME_SKIP = 25;


		/*
		read single oni and write pcds
		*/
		void readOni (const char* const oniFile, 
			const char* writeToDirPath = NULL, 
			const int framesToSkip = DEFAULT_FRAME_SKIP);

		/*
		return directory path to write pcd files to
		*/
		char* getWriteDirPath (char* const writeToDir = NULL);



		/*
		set number of frames to skip
		*/
		void setFrameSkip (const int framesToSkip);

		/*
		assigns totalFrames, framestoRead, and currentFrame based on framesInOni
		*/
		void setFrameInfo (const int framesInOni);

		/*
		callback for our readOniFile, actually writes the pointcloud
		*/
		void writeCloudCb (const CloudConstPtr& cloud);	
	};
};

#endif