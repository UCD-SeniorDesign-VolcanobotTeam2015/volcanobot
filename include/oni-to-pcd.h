/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.h
Description:	Reads an oni file recorded using the Openni2 library and outputs point clouds (pcd files)
*/

#ifndef _ONI_TO_PCD
#define _ONI_TO_PCD

#include <pcl/io/openni2_grabber.h>
#include <pcl/point_cloud.h>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;

namespace vba {
	namespace oni2pcd {
		const int DEFAULT_FRAME_SKIP = 25;

		int totalFrames = 0,
			framesToRead = 0,
			currentFrame = 0,
			currentReadFrame = 0,
			frameSkip = 0,
			timeout = 0;

		char* pcdWriteDirPath = NULL;

		/*
		read single oni and write pcds
		*/
		void readOni (const char* oniFile, 
			char* writeToDirPath = NULL, 
			const int framesToSkip = DEFAULT_FRAME_SKIP);

		/*
		return directory path to write pcd files to
		*/
		char* getWriteDirPath (char* writeToDir = NULL);

		/*
		set number of frames to skip in writing pcd files
		*/
		void setFrameSkip (const int framesToSkip);

		/*
		assigns totalFrames, framestoRead, and currentFrame based on framesInOni
		*/
		void setFrameInfo (const int framesInOni);

		/*
		sets timeout based on processing
		*/
		void setTimeout (const int to);

		/*
		callback for our readOniFile, actually writes the pointcloud
		*/
		void writeCloudCb (const CloudConstPtr& cloud);	
	};
};

#endif