/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.cpp
Description:	Reads an oni file recorded using the Openni2 library and outputs point clouds (pcd files)
*/

#include "oni-to-pcd.h"
#include "pcl/point_types.h"
#include <iostream>
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
void oni2pcd::readOni (const char* const oniFile, 
		const int framesToSkip) {

}

/*
private members of oni2pcd
*/
void oni2pcd::writeCloudCb (const oni2pcd::CloudConstPtr& cloud) {

}