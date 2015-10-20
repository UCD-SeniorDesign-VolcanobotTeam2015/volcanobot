/*
 * CloudStitcher.h
 *
 *  Created on: Oct 12, 2015
 *      Author: matt
 */

//Generic includes
#include <vector>
#include <string>
#include <utility>
#include <iostream>
#include <algorithm>
#include <cmath>

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

//include some of the boost library to help with searching filesystem directories
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

//include some of the boost library for multithreading
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/locale.hpp>


#ifndef CLOUDSTITCHER_H_
#define CLOUDSTITCHER_H_

namespace vba
{

	enum THREAD_COUNT
	{
		THREAD_1,
		THREAD_2,
		THREAD_4,
		THREAD_8,
		THREAD_16
	};


	class CloudStitcher
	{

		public:


			/*Default constructor for this class, which just initializes the members to default values.
			 *
			 * @param:
			 * @param:
			 * @return:
			 */
			CloudStitcher();

			/*Default destructor that deallocates the dynamically allocated members of the class.
			 *
			 * @param:
			 * @param:
			 * @return:
			 */
			virtual ~CloudStitcher();


			/*Public facing function that allows the user to select which directory contains target pcd files.
			 * The function then checks the existence of the directory and extracts all files with .pcd file
			 * extension.
			 *
			 * @param: The string containing the absolute path pointing towards the directory containing
			 * 			the target pcd files.
			 * @return: 0 if the operation was successful, -1 if not.
			 */
			int setPCDDirectory( const std::string directory_path );

			//methods to toggle configuration settings

			/*Public facing function that allows the user to toggle the utilization of concurrency while
			 * stitching together clouds.
			 *
			 * @param: true if concurrency should be utilized, false otherwise. The default value is true.
			 * @return:none
			 */
			void enableMultithreading( const bool choice );

			/*This function allows the user to initiate the cloud stitching operation. This is the function
			 * that will instantiate all threads, supply their input, and monitor the threads to completion.
			 *
			 * @return: 0 if operation was successful, -1 otherwise
			 */
			int stitchPCDFiles();


			//class setters and getters


			/*Just a getter function to return the number of read in files from the directory.
			 *
			 * @return: the number of pcd files that had been read in from the directory supplied to the
			 * 			setPCDDirectory() function call.
			 */
			unsigned int getNumberofFilesRead();

		private:

			std::string pcd_files_directory;
			std::vector<std::string>* pcd_filenames;


			//configurations
			bool multithreading_enabled;
			THREAD_COUNT num_threads;

			/*This is a helper function used internally within CloudStitcher. It subdivides all the pcd
			 * files to be worked on among the threads.
			 *
			 * @param: the number of threads that are going to be used
			 * @param: the total number of files to be stitched by the threads
			 * @return: none
			 */
			void setupWorkerThreads( unsigned int thread_count , unsigned int num_files );


			//This is a class that wraps all the operations associated with one thread. Since there can be multiple
			//instances of this class that are all owned by a single CloudStitcher object, we encapsulated the
			//class definition within CloudStitcher.
			class CloudStitchingThread
			{
				public:

					/*Class Constructor
					 *
					 * @param: a copy of the vector containing the pcd files to stich together
					 * @return: none
					 */
					CloudStitchingThread( const std::vector< std::string >& files );

					/*Default destructor that deallocates the objects allocated dynamically throughout this instances
					 * lifespan
					 *
					 * @param: none
					 * @return: none
					 */
					~CloudStitchingThread();

					/*This method starts up the thread to stitch clouds together. It also calls detectExitThread() to
					 * wait on the new thread. For this reason,  this function will return immediately to the calling
					 * function instead of blocking until the thread is finished.
					 *
					 * @param: none
					 * @return: none
					 */
					void start();

					/*This function can be repeatedly called to check if the worker thread has finished yet.
					 *
					 * @param: none
					 * @return: true if the worker thread is finished, false otherwise
					 */
					bool isFinished();

					/*
					 *
					 * @param:
					 * @param:
					 * @return:
					 */
					void detectThreadExit();

					/*
					 *
					 * @param:
					 * @param:
					 * @return:
					 */
					void stitchTargetClouds();

				private:

					bool worker_thread_is_finished;
					std::vector< std::string >* file_list;

					boost::thread worker_thread;
					boost::thread exit_detect_thread;
			};

			std::vector<CloudStitcher::CloudStitchingThread* >* worker_threads;

	};

} /* namespace vba */

#endif /* CLOUDSTITCHER_H_ */
