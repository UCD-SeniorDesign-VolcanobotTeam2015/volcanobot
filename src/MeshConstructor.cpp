/*
 * MeshConstructor.cpp
 *
 *  Created on: Nov 10, 2015
 *      Author: matt
 */

#include "MeshConstructor.h"

namespace vba
{

	MeshConstructor::MeshConstructor()
	{
		this->output_filetype = PLY;
		this->input_filename = "";
		this->output_filename = "";
	}

	MeshConstructor::~MeshConstructor()
	{

	}

	int MeshConstructor::setInputFilename( std::string filename )
	{
		if( !boost::filesystem::exists( filename ) )
		{
			std::cerr << "Error: Provided input file: " << filename << " does not exist.\n";
			return -1;
		}

		if( filename.find( ".pcd" ) == std::string::npos )
		{
			std::cerr << "Error: Provided input file: " << filename << " is not a .pcd file.\n";
			return -1;
		}

		this->input_filename = filename;
		return 0;
	}

	int MeshConstructor::setOutputFilename( std::string filename , MESH_FILETYPE type )
	{
		if( filename.size() == 0 )
		{
			std::cerr << "Error: Empty string provided as filename.\n";
			return -1;
		}

		if( type == OBJ )
		{
			if( filename.find( ".obj" ) == std::string::npos )
			{
				std::cerr << "Error: Provided output filename does not have .obj file extension.\n";
				return -1;
			}
		}

		if( type == VTK )
		{
			if( filename.find( ".vtk" ) == std::string::npos )
			{
				std::cerr << "Error: Provided output filename does not have .obj file extension.\n";
				return -1;
			}
		}

		if( type == PLY )
		{
			if( filename.find( ".ply" ) == std::string::npos )
			{
				std::cerr << "Error: Provided output filename does not have .obj file extension.\n";
				return -1;
			}
		}


		this->output_filetype = type;
		this->output_filename = filename;
		return 0;
	}

	int MeshConstructor::constructMesh()
	{
		if( this->input_filename.size() == 0 )
		{
			std::cerr << "Error: No input file has been specified yet.\n";
			return -1;
		}

		if( this->output_filename.size() == 0 )
		{
			std::cerr << "Error: No output filename has been specified yet.\n";
			return -1;
		}


		PointCloud::Ptr cloud( new PointCloud );
		pcl::io::loadPCDFile( this->input_filename , *cloud );
		std::cout << "Original Cloud Size: " << cloud->size() << "\n";

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_remove_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cond_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );

		pcl::VoxelGrid< pcl::PointXYZRGB  > voxel_filter;
		voxel_filter.setLeafSize( 0.1 , 0.1 , 0.1 );
		voxel_filter.setInputCloud( cloud  );
		voxel_filter.filter( *filtered_cloud );

		std::cout << "New Cloud Size: " << filtered_cloud->size() << "\n";
		std::cout << "Removed " << cloud->size() - filtered_cloud->size() << " points.\n";


		pcl::RadiusOutlierRemoval< pcl::PointXYZRGB > radius_remove;

		radius_remove.setInputCloud( filtered_cloud );
		radius_remove.setRadiusSearch( 0.5 );
		radius_remove.setMinNeighborsInRadius( 75  );
		radius_remove.filter( *radius_remove_cloud );

		std::cout << "\nRadius Removal Cloud Size: " << radius_remove_cloud->size() << "\n";
		std::cout << "Removed " << filtered_cloud->size() - radius_remove_cloud->size() <<  " points.\n";

		pcl::NormalEstimation< PointT , PointNormal > normal_estimator;
		PointCloudNormals::Ptr normals( new PointCloudNormals );
		pcl::search::KdTree< PointT >::Ptr tree( new pcl::search::KdTree< PointT > );
		tree->setInputCloud( radius_remove_cloud );
		normal_estimator.setInputCloud( radius_remove_cloud );
		normal_estimator.setSearchMethod( tree );
		normal_estimator.setKSearch( 50 );
		normal_estimator.compute( *normals );

		pcl::PointCloud< PointNormal >::Ptr cloud_normals( new pcl::PointCloud< PointNormal > );
		concatenateFields (*normals , *radius_remove_cloud, *cloud_normals);

		pcl::search::KdTree< PointNormal >::Ptr tree2 (new pcl::search::KdTree< PointNormal >);
		tree2->setInputCloud( cloud_normals );


		pcl::Poisson<PointNormal> poisson;
		poisson.setInputCloud( cloud_normals );
		poisson.setSearchMethod( tree2 );

		pcl::PolygonMesh mesh_poisson;
		poisson.reconstruct (mesh_poisson);

		std::cout << "\nFinished mesh reconstruction.\n";
		std::cout << "Mesh contains " << mesh_poisson.polygons.size() << " polygons.\n";

		switch( this->output_filetype )
		{
		case OBJ:
			pcl::io::saveOBJFile( this->output_filename , mesh_poisson );
			std::cout << "\nSaved new polygon mesh to: " << this->output_filename << "\n";
			break;

		case VTK:
			pcl::io::saveVTKFile( this->output_filename , mesh_poisson );
			std::cout << "\nSaved new polygon mesh to: " << this->output_filename << "\n";
			break;

		case PLY:
			pcl::io::savePLYFile( this->output_filename , mesh_poisson );
			std::cout << "\nSaved new polygon mesh to: " << this->output_filename << "\n";
			break;

		default:
			pcl::io::savePLYFile( this->output_filename , mesh_poisson );
			std::cout << "\nSaved new polygon mesh to: " << this->output_filename << "\n";
			break;
		}

		return 0;

	}

} /* namespace vba */
