

#ifndef PCDREGISTRATION_H_
#define PCDREGISTRATION_H_

//some generic includes
#include <iostream>
#include <vector>
#include <string>

//point cloud data types includes
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

//include to perform standard io
#include <pcl/io/pcd_io.h>

//includes for point cloud filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

//include for surface normals operations
#include <pcl/features/normal_3d.h>

//includes for point cloud registration (stitching)
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

//include for the actual viewing of point clouds on screen
#include <pcl/visualization/pcl_visualizer.h>






namespace vba
{

	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef pcl::PointNormal PointNormalT;
	typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

	struct PCD
	{
	  PointCloud::Ptr cloud;
	  std::string f_name;

	  PCD() : cloud (new PointCloud) {};
	};

	class PCDRegistration
	{
		public:


			PCDRegistration( const std::vector< std::string >& files , std::string output_file );
			~PCDRegistration();

			int start();

		private:

			void loadPCDData( std::string first , std::string second );

			std::vector< std::string >* file_list;
			std::string output_filename;

			std::vector<PCD, Eigen::aligned_allocator<PCD> >* point_cloud_models;
	};

}

#endif
