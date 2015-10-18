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

namespace vba {
	/*
	oni to pcd class
	*/
	class oni2pcd {
	public:
		/*
		constructors
		*/
		oni2pcd () : oniFilesBeingRead(), 
			pcdWriteDir(),
			totalFrames(0),
			framesToRead(0),
			currentFrame(0),
			frameSkip(0) {} 

		/*
		read single oni and write pcds
		*/
		void readOni (const char* const oniFile, 
			const int framesToSkip = DEFAULT_FRAME_SKIP);

		// /*
		// read multiple oni files
		// */
		// void readOnis (const std::vector<const char*>  oniFiles[], 
		// 	const int framesToSkip = DEFAULT_FRAME_SKIP);

		// /*
		// get oni files being read
		// */
		// const std::vector<const char*> getOniFilesBeingRead () const;

		static const int DEFAULT_FRAME_SKIP = 25;

	private:
		const std::vector<const char*> oniFilesBeingRead;
		const std::vector<const char*> pcdWriteDir;

		int totalFrames,
			framesToRead,
			currentFrame,
			frameSkip;

		typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
		typedef Cloud::ConstPtr CloudConstPtr;

		void writeCloudCb (const CloudConstPtr& cloud);

		void setFrameSkip (const int framesToSkip);

	};
};

#endif