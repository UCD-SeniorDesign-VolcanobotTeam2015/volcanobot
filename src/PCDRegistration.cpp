

#include "PCDRegistration.h"

namespace vba
{

	PCDRegistration::PCDRegistration( const std::vector< std::string >& files , std::string output_file )
	{
		this->file_list = new std::vector< std::string >( files );
		this->output_filename = output_file;
		this->redirect_output_flag = false;
	}

	PCDRegistration::~PCDRegistration()
	{
		delete this->file_list;
	}

	int PCDRegistration::start()
	{
		std::stringstream output;
		output << "processing " << this->file_list->size() << " frames.\n";
		this->sendOutput( output.str() , false );

		PCD first;
		Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
		PointCloud::Ptr result ( new PointCloud );

		first.filename = this->file_list->at(0);

		//load in the the first point cloud file
		this->loadPCDData(  &first );

		std::vector< std::string >::iterator itr;
		for( itr = this->file_list->begin() + 1 ; itr != this->file_list->end() ; ++itr )
		{
			PCD second;
			second.filename = *itr;

			if( this->loadPCDData( &second ) != 0 )
			{
				continue;
			}

			PointCloud::Ptr temp (new PointCloud);
			this->pairAlign (first.cloud , second.cloud , temp , pairTransform, true);


			//transform current pair into the global transform
			pcl::transformPointCloud (*temp , *result , GlobalTransform );

			//update the global transform
			GlobalTransform = GlobalTransform * pairTransform;
		}

		pcl::io::savePCDFile ( this->output_filename , *result , true );

		return 0;
	}

	int PCDRegistration::loadPCDData( PCD* target  )
	{

		if( pcl::io::loadPCDFile( target->filename , *target->cloud ) == 0 )
		{
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud( *target->cloud , *target->cloud , indices );
			return 0;
		}
		else
		{
			return -1;
		}

	}

	void PCDRegistration::pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample )
	{
		//
		  // Downsample for consistency and speed
		  // \note enable this for large datasets
		  PointCloud::Ptr src (new PointCloud);
		  PointCloud::Ptr tgt (new PointCloud);
		  pcl::VoxelGrid<PointT> grid;
		  if (downsample)
		  {
		    grid.setLeafSize (0.05, 0.05, 0.05);
		    grid.setInputCloud (cloud_src);
		    grid.filter (*src);

		    grid.setInputCloud (cloud_tgt);
		    grid.filter (*tgt);
		  }
		  else
		  {
		    src = cloud_src;
		    tgt = cloud_tgt;
		  }


		  // Compute surface normals and curvature
		  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
		  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

		  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
		  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		  norm_est.setSearchMethod (tree);
		  norm_est.setKSearch (30);

		  norm_est.setInputCloud (src);
		  norm_est.compute (*points_with_normals_src);
		  pcl::copyPointCloud (*src, *points_with_normals_src);

		  norm_est.setInputCloud (tgt);
		  norm_est.compute (*points_with_normals_tgt);
		  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

		  //
		  // Instantiate our custom point representation (defined above) ...
		  MyPointRepresentation point_representation;
		  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
		  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
		  point_representation.setRescaleValues (alpha);

		  //
		  // Align
		  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
		  reg.setTransformationEpsilon (1e-3);
		  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
		  // Note: adjust this based on the size of your datasets
		  reg.setMaxCorrespondenceDistance (0.5);
		  // Set the point representation
		  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

		  reg.setInputSource (points_with_normals_src);
		  reg.setInputTarget (points_with_normals_tgt);



		  //
		  // Run the same optimization in a loop and visualize the results
		  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
		  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
		  reg.setMaximumIterations (2);
		  for (int i = 0; i < 30; ++i)
		  {
		    //PCL_INFO ("Iteration Nr. %d.\n", i);

		    // save cloud for visualization purpose
		    points_with_normals_src = reg_result;

		    // Estimate
		    reg.setInputSource (points_with_normals_src);
		    reg.align (*reg_result);

				//accumulate transformation between each Iteration
		    Ti = reg.getFinalTransformation () * Ti;

				//if the difference between this transformation and the previous one
				//is smaller than the threshold, refine the process by reducing
				//the maximal correspondence distance
		    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
		      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

		    prev = reg.getLastIncrementalTransformation ();

		    // visualize current state
		    //showCloudsRight(points_with_normals_tgt, points_with_normals_src);
		  }

			//
		  // Get the transformation from target to source
		  targetToSource = Ti.inverse();

		  //
		  // Transform target back in source frame
		  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

		  //p->removePointCloud ("source");
		  //p->removePointCloud ("target");

		  //PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
		  //PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
		  //p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
		  //p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);
		//HERE
		//	PCL_INFO ("Press q to continue the registration.\n");
		//  p->spin ();

		  //p->removePointCloud ("source");
		  //p->removePointCloud ("target");

		  //add the source to the transformed target
		  *output += *cloud_src;

		  final_transform = targetToSource;
	}


	void PCDRegistration::setOutputFunction( outputFunction function_pointer )
	{
		this->user_output_function = function_pointer;
		this->redirect_output_flag = true;
	}


	void PCDRegistration::sendOutput( std::string output , bool is_error )
	{
		if( this->redirect_output_flag )
		{
			this->user_output_function( output , is_error );
		}

		else
		{
			if( is_error == true )
			{
				std::cerr << output;
			}
			else
			{
				std::cout << output;
			}
		}
	}
}
