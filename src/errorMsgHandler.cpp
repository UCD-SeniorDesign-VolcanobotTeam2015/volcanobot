/*	
Team:			VolcanoBot A
Project:		TBD
File:			errorMsgHandler.cpp
Description:	Provides error messaging and handling
*/

#ifndef _ERROR_MSG_HANDLE
#define _ERROR_MSG_HANDLE

#include <cstdlib>
#include <iostream>

namespace vba {
	/*
	check if file is null
	*/
	bool isNull (const char* const file) {
		return file == NULL;
	}

	/*
	print error message if file pointer was null
	*/
	void handleEmptyFilePtr (const char* const filename) {
		std::cerr << "\nError opening " << filename << '\n';
		exit(EXIT_FAILURE);
	}
};

#endif