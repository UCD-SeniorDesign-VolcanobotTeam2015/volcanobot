/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.cpp
Description:	Reads an oni file recorded using the Openni2 or Openni library and outputs point clouds (pcd files)
*/

#include "oni-to-pcd.h"
#include "errorMsgHandler.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <string>
#include <cstring>

int main (int argc, char* argv[]) {
	/*
	expect 2 additional arguments, ie. <executable> <oniFile> <pcdWriteDir>
	*/
	const int REQ_ARGS = 3;
	if (argc < REQ_ARGS) {
 		std::cout << "\nNot enough arguments provided.\n";
 		return EXIT_FAILURE;
	}
	std::cout << '\n' << vba::oni2pcd::getWriteDirPath() << '\n';

	return EXIT_SUCCESS;
}

/*
vba::oni2pcd namespace functions
*/
int vba::oni2pcd::totalFrames,
	vba::oni2pcd::framesToRead,
	vba::oni2pcd::currentFrame,
	vba::oni2pcd::currentReadFrame,
	vba::oni2pcd::frameSkip;

/*
read the given oni file, write the output pcd files to the given directory,
if no directory is given will write to pcdTemp in the executable directory
*/
void vba::oni2pcd::readOni (const char* const oniFile, 
		const char* writeToDirPath,
		const int framesToSkip) {
	if (vba::isNull(oniFile)) {
		vba::handleEmptyFilePtr (oniFile);
	}

	/*
	create a pointer to an Openni2 grabber instance, register a callback for the grabber, bind the callback writeCloudCb from our vba::oni2pcd namespace
	*/
	pcl::io::OpenNI2Grabber *grabber = new pcl::io::OpenNI2Grabber (oniFile);
	boost::function<void (const CloudConstPtr&) > f = boost::bind (&vba::oni2pcd::writeCloudCb, _1);
	boost::signals2::connection c = grabber->registerCallback (f);

	/*
	setup frame info
	*/
	vba::oni2pcd::setFrameSkip (framesToSkip);
	vba::oni2pcd::setFrameInfo(grabber->getDevice()->getDepthFrameCount());

	while (currentReadFrame < framesToRead) {
		grabber->start();
	}

	grabber->stop();
}

/*
write pcd files to pcdTemp under given directory path if it exists,
write pcd files to pcdTemp under executable directoy path alternatively
*/
char* vba::oni2pcd::getWriteDirPath (char* writeToDir) {

	if (writeToDir != NULL) {
		boost::filesystem::path writeToDirPath (writeToDir);

		if (boost::filesystem::exists(writeToDirPath) 
			&& boost::filesystem::is_directory (writeToDirPath)) {
			boost::filesystem::path pcdWriteToDirPath(writeToDirPath);
			pcdWriteToDirPath += "pcdTemp";

			writeToDir = new char [pcdWriteToDirPath.string().length()];
			return strcpy (writeToDir, pcdWriteToDirPath.string().c_str());
		}
	}

	writeToDir = new char [boost::filesystem::current_path().string().length()];
	return strcpy (writeToDir, boost::filesystem::current_path().string().c_str());
}

/*
sets the number of frames to skip in the oni file, if a number less than 
default, which is minimum, then the frame skip is set to default
*/
void vba::oni2pcd::setFrameSkip (const int framesToSkip) {
	if (framesToSkip < vba::oni2pcd::DEFAULT_FRAME_SKIP) {
		vba::oni2pcd::frameSkip = vba::oni2pcd::DEFAULT_FRAME_SKIP;
		return;
	}

	vba::oni2pcd::frameSkip = framesToSkip;
}

/*
total frames := the number of frames in the oni file,
frames to read := the number of frames to actually create pcd files from
	derived from total frames / frame skip,
current frame := current frame in total frames,
current read frame := current frame to write pcd file for
*/
void vba::oni2pcd::setFrameInfo (const int framesInOni) {
	vba::oni2pcd::totalFrames = framesInOni;
	vba::oni2pcd::framesToRead = vba::oni2pcd::totalFrames / vba::oni2pcd::frameSkip;
	vba::oni2pcd::currentFrame = 0;
	vba::oni2pcd::currentReadFrame = 0;
}

void vba::oni2pcd::writeCloudCb (const CloudConstPtr& cloud) {
	static std::string buffer;
	static std::stringstream ss;
	static pcl::PCDWriter w;

	if (vba::oni2pcd::currentFrame % vba::oni2pcd::frameSkip == 0) {

		ss << vba::oni2pcd::pcdWriteDirPath << "frame_" << std::setfill ('0') << std::setw(5) << vba::oni2pcd::currentFrame << ".pcd";
		std::cout <<"Wrote a coud to " << ss.str() << '\n';

		++vba::oni2pcd::currentFrame;
	}
	++vba::oni2pcd::currentFrame;
}