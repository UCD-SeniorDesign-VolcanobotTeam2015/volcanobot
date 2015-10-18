/*	Team:			VolcanoBot A
	File:			oni-to-pcd
	Description:	Reads an oni file recorded using the Openni2 library and outputs point clouds (pcd files)
*/

#include "openni2_grabber.h"
#include <iostream>
#include <cstdlib>

int main (int argc, char* argv[]) {
	// expect 2 additional arguments, ie. <executable> <oniFile> <pcdWriteDir>
	const int REQ_ARGS = 3;
	if (argc < REQ_ARGS) {

	}

	return EXIT_SUCCESS;
}