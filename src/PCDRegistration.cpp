

#include "PCDRegistration.h"

namespace vba
{

	PCDRegistration::PCDRegistration( const std::vector< std::string >& files , std::string output_file )
	{
		this->file_list = new std::vector< std::string >( files );
		this->output_filename = output_file;
		this->point_cloud_models = new std::vector<PCD, Eigen::aligned_allocator<PCD> >();
	}

	PCDRegistration::~PCDRegistration()
	{
		delete this->file_list;
	}

	int PCDRegistration::start()
	{
		//load in the point cloud models from the supplied pcd files
		this->loadPCDData( file_list->at(3) , file_list->at(4));

		//check to make sure we actually loaded some point cloud models in
		if( this->point_cloud_models->empty() )
		{
			std::cerr << "Error: Problem with PCL loading in the pcd files.\n";
			return -1;
		}

		std::cout << "Loaded in " << this->point_cloud_models->size() << " models.\n";



		return 0;
	}

	void PCDRegistration::loadPCDData( std::string first , std::string second )
	{
		std::vector< std::string >::iterator itr;
		for ( itr = this->file_list->begin() ; itr != this->file_list->end() ; ++itr )
		{
			// Load the cloud and saves it into the global list of models
			PCD m;
			m.f_name = *itr;
			pcl::io::loadPCDFile( *itr , *m.cloud);

			//remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

			this->point_cloud_models->push_back( m );
		}

		std::cout << "made it here\n";
	}
}
