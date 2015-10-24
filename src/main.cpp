

#include "CloudStitcher.h"
#include <string>

int main( int argc , char* argv[] )
{
	vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;

	std::string dir( argv[1] );
	mCloudStitcher->setOutputPath( "/dev/null" );
	mCloudStitcher->stitchPCDFiles( dir );
	delete mCloudStitcher;


	return 0;

}
