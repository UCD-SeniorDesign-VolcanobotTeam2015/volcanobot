

#include "CloudStitcher.h"
#include <string>

int main( int argc , char* argv[] )
{
	vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;

	std::string dir( argv[1] );
	std::string output( argv[2] );
	mCloudStitcher->setOutputPath( output );
	mCloudStitcher->stitchPCDFiles( dir );
	delete mCloudStitcher;



	return 0;

}
