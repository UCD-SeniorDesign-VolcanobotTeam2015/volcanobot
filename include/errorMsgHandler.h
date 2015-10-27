/*	
Team:			VolcanoBot A
Project:		TBD
File:			errorMsgHandler.h
Description:	Provides error messaging and handling
*/

namespace vba {
	/*
	file is null
	*/
	bool isNull (const char* const file);

	/*
	handler for empty file pointer
	*/
	void handleEmptyFilePtr (const char* const filename);

}