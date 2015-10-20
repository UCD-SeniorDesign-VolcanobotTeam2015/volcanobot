/*
 * CloudStitcher.cpp
 *
 *  Created on: Oct 12, 2015
 *      Author: matt
 */

#include "CloudStitcher.h"

namespace vba
{

	CloudStitcher::CloudStitcher()
	{
		this->pcd_filenames = new std::vector< std::string >();
		this->worker_threads = new std::vector< CloudStitcher::CloudStitchingThread* >();

		this->num_threads = THREAD_COUNT::THREAD_1;
		this->multithreading_enabled = true;

	}

	CloudStitcher::~CloudStitcher()
	{
		delete pcd_filenames;

		for( auto itr = this->worker_threads->begin() ; itr != this->worker_threads->end() ; ++itr )
		{
			CloudStitcher::CloudStitchingThread* temp = *itr;
			delete temp;
		}
	}

	int CloudStitcher::setPCDDirectory( const std::string directory_path )
	{
		std::string file_extension = ".pcd";
		std::string current_path;

		//check to make sure this instance of cloud stitcher hasn't already read in files
		if( this->pcd_filenames->size() != 0 )
		{
			std::cerr << "Error: This instance has already read in .pcd files.\n";
			return -1;
		}

		//check to make sure there actually are characters in the function parameter
		if( directory_path.size() == 0 )
		{
			std::cerr << "Error: Provided empty string as directory.\n";
			return -1;
		}

		//check to make sure the directory is real
		if( boost::filesystem::exists( directory_path ))
		{
			this->pcd_files_directory = directory_path;

			boost::filesystem::directory_iterator end;

			//iterate over all entries in the directory
			for( boost::filesystem::directory_iterator itr( directory_path ) ; itr != end ; ++itr )
			{
				//check to make sure the entry is not another directory
				if( !boost::filesystem::is_directory( *itr ) )
				{
					boost::filesystem::path p = itr->path();
					current_path = p.string();

					//convert to all lowercase
					std::transform ( current_path.begin(), current_path.end(), current_path.begin(), (int(*)(int))tolower);

					if( current_path.find( file_extension , 0 ) == std::string::npos )
					{
						continue;
					}
					else
					{
						//add the filename to the vector
						current_path = p.string();
						this->pcd_filenames->push_back( current_path );
					}
				}
			}
		}

		else
		{
			//return if the directory path cannot be found
			std::cerr << "Error: Given directory: " << directory_path << " does not exits.\n";
			return -1;
		}

		//make sure that the filenames are sorted in correct order
		std::sort( this->pcd_filenames->begin() , this->pcd_filenames->end() );

		//now we will set the number of threads to be used based on how many files were found
		const unsigned int num_files = this->getNumberofFilesRead();

		if( num_files <= 10 )
		{
			this->num_threads = THREAD_COUNT::THREAD_1;
		}
		else if( num_files <= 20 )
		{
			this->num_threads = THREAD_COUNT::THREAD_2;
		}
		else if( num_files <= 40 )
		{
			this->num_threads = THREAD_COUNT::THREAD_4;
		}
		else if( num_files <= 80 )
		{
			this->num_threads = THREAD_COUNT::THREAD_8;
		}
		else if( num_files <= 160 )
		{
			this->num_threads = THREAD_COUNT::THREAD_16;
		}
		else if( num_files > 160 )
		{
			this->num_threads = THREAD_COUNT::THREAD_16;
		}
		else
		{
			this->num_threads = THREAD_COUNT::THREAD_1;
		}

		return 0;
	}

	int CloudStitcher::stitchPCDFiles()
	{
		/*
		std::vector< boost::thread > thread_vector;

		std::vector< std::string > new_vec;
		new_vec.push_back( this->pcd_filenames->at(0 ) );
		CloudStitchingThread* ct = new CloudStitchingThread( new_vec );
		ct->start();

		while( ct->isFinished() == false )
		{

		}

		delete ct;
		std::cout << "all threads have rejoined main\n";
		*/

		switch( this->num_threads )
		{
		case THREAD_COUNT::THREAD_1:
			this->setupWorkerThreads( 1 , this->getNumberofFilesRead() );
			break;

		case THREAD_COUNT::THREAD_2:
			this->setupWorkerThreads( 2 , this->getNumberofFilesRead() );
			break;

		case THREAD_COUNT::THREAD_4:
			this->setupWorkerThreads( 4 , this->getNumberofFilesRead() );
			break;

		case THREAD_COUNT::THREAD_8:
			this->setupWorkerThreads( 8 , this->getNumberofFilesRead() );
			break;

		case THREAD_COUNT::THREAD_16:
			this->setupWorkerThreads( 16 , this->getNumberofFilesRead() );
			break;

		default:
			this->setupWorkerThreads( 1 , this->getNumberofFilesRead() );
			break;
		}
	}

	void CloudStitcher::enableMultithreading( const bool choice )
	{
		this->multithreading_enabled = choice;
	}

	unsigned int CloudStitcher::getNumberofFilesRead()
	{
		if( !this->pcd_filenames->empty() )
		{
			return this->pcd_filenames->size();
		}

		return 0;
	}

	void CloudStitcher::setupWorkerThreads( unsigned int thread_count , unsigned int num_files )
	{
		//offset is the number of pcd files that should be allocated to each thread
		unsigned int offset = (int)std::ceil( num_files / thread_count );

		//current offset will be a changing variable representing where in the filename array we are looking at
		auto current_offset = this->pcd_filenames->begin();

		//if we are using one thread, then just copy all of the filenames into the single thread
		if( thread_count == 1 )
		{
			std::vector< std::string > param_vec( current_offset , this->pcd_filenames->end() );
			this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec ));
		}

		//otherwise we are going to divide up the filename array as evenly as possible among all the threads
		else
		{
			for( int i = 0 ; i < thread_count - 1 ; ++i )
			{
				auto last = ( current_offset + offset );
				std::vector< std::string > param_vec( current_offset , last );
				this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec ));
				current_offset = last;
			}

			//to prevent a seg-fault we just copy whatever is left of the filename array into the last thread
			std::vector< std::string > param_vec( current_offset , this->pcd_filenames->end() );
			this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec ));

		}
	}








	CloudStitcher::CloudStitchingThread::CloudStitchingThread( const std::vector< std::string >& files )
	{
		//we will make a copy of the list of target filenames for this instance of the class
		this->file_list = new std::vector< std::string >( files );
		this->worker_thread_is_finished = false;
	}

	CloudStitcher::CloudStitchingThread::~CloudStitchingThread()
	{
		delete file_list;
	}

	void CloudStitcher::CloudStitchingThread::start()
	{
		//spin off the function that performs the stitching on all the target clouds
		this->worker_thread = boost::thread( &CloudStitcher::CloudStitchingThread::stitchTargetClouds , this );

		//spin off another thread that periodically checks if the worker thread has finished yet.
		this->exit_detect_thread = boost::thread( &CloudStitcher::CloudStitchingThread::detectThreadExit , this );
	}

	bool CloudStitcher::CloudStitchingThread::isFinished()
	{
		if( this->worker_thread_is_finished == true )
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void CloudStitcher::CloudStitchingThread::detectThreadExit()
	{
		//Since this function sits inside its own thread, it just spins while waiting for the worker thread to rejoin
		worker_thread.join();

		//when the worker thread has finished, this boolean is set to true, so we know its done
		this->worker_thread_is_finished = true;
	}

	void CloudStitcher::CloudStitchingThread::stitchTargetClouds()
	{
		std::cout << "worker is going to sleep\n";
		boost::posix_time::seconds workTime( 10);
		boost::this_thread::sleep( workTime );
		std::cout << "worker woke up\n";
	}

} /* namespace vba */
