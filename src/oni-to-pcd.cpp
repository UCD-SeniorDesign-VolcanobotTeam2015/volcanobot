/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.cpp
Description:	Reads an oni file recorded using the Openni2 or Openni library and outputs point clouds (pcd files)
*/

#include "oni-to-pcd.h"
#include "errorMsgHandler.h"
#include "pcl/point_types.h"
#include <iostream>
#include <fstream>
#include <cstdlib>

int main (int argc, char* argv[]) {
	/*
	expect 2 additional arguments, ie. <executable> <oniFile> <pcdWriteDir>
	*/
	const int REQ_ARGS = 3;
	if (argc < REQ_ARGS) {
 		std::cout << "\nNot enough arguments provided.\n";
 		return EXIT_FAILURE;
	}



	return EXIT_SUCCESS;
}

/*
public members of oni2pcd
*/
void vba::oni2pcd::readOni (const char* const oniFile, 
		const int framesToSkip) {
	if (vba::isNull(oniFile)) {
		handleEmptyFilePtr (oniFile);
	}

	setFrameSkip (framesToSkip);

	

}

/*
private members of oni2pcd
*/
void vba::oni2pcd::writeCloudCb (const vba::oni2pcd::CloudConstPtr& cloud) {

}

void setFrameSkip (const int framesToSkip) {
	if (framesToSkip < DEFAULT_FRAME_SKIP) {
		frameSkip = DEFAULT_FRAME_SKIP;
		return;
	}

	frameSkip = framesToSkip
}