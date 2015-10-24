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
		this->output_path = "";
		this->temp_directories = new std::vector< std::string >();

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

	int CloudStitcher::setPCDDirectory( std::string directory_path )
	{
		std::string file_extension = ".pcd";
		std::string current_path;


		//check to make sure there actually are characters in the function parameter
		if( directory_path.size() == 0 )
		{
			std::cerr << "Error: Provided empty string as directory.\n";
			return -1;
		}

		//now lets check if the last character in the directory path is '/'. If there is none
		//we need to add it in for path concatenation later on
		if( directory_path.at( directory_path.size() - 1 ) != '/' )
		{
			directory_path += "/";
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


	int CloudStitcher::setOutputPath( const std::string output_path )
	{
		std::string path;
		std::string filename;

		//first we must split the filename from the directory path
		std::size_t found = output_path.find_last_of( "/\\" );
		if( found != std::string::npos )
		{
			//make a substring of just the path and make sure it exists
			path = output_path.substr( 0 , found );
			if( !boost::filesystem::exists( output_path ) )
			{
				std::cerr << "Error: specified output directory does not exist.\n";
				return -1;
			}
		}

		//make a substring of just the filename and make sure it isn't empty
		filename = output_path.substr( found + 1 );
		if( filename.size() < 1 )
		{
			std::cerr << "Error: no filename was specified.\n";
			return -1;
		}

		//if everything goes well, we will set the class member to the provided output path
		//for use later
		this->output_path = output_path;
		return 0;
	}


	int CloudStitcher::stitchPCDFiles( const std::string directory_path )
	{
		//do some basic error handling first and make sure the user gave us an output path to work with
		if( this->output_path == "" )
		{
			std::cerr << "Error: No output path has been specified yet.\n";
			return -1;
		}

		//lets open up the directory and see how many files are in there
		int result = this->setPCDDirectory( directory_path );
		if( result != 0 )
		{
			return -1;
		}

		unsigned int file_count = this->getNumberofFilesRead();

		//Since this function will be setup recursively we have to start with a base case to
		//know when to stop and start rewinding the stack. If we only read in one file, then
		//we know that all pcd files have been combined and we are finished. This would also
		//apply if the user only supplied us with one pcd file.
		if( file_count == 1 )
		{
			//TODO we need to catch errors thrown by rename() like trying to put the file in a dir without the right access permissions

			//send our single created file to the desired output location
			boost::filesystem::rename( this->pcd_filenames->at(0) , this->output_path );
			return 0;
		}

		//we will create a new temporary directory to contain the output of each recursive step.
		//The first temp dir will be located in the original pcd directory. The second will be placed
		//in the first temp dir and so on.
		unsigned int temp_dir_count = this->temp_directories->size();

		//if this is the first recursive step we will name the temp dir "temp_dir1"
		if( temp_dir_count == 0 )
		{
			std::string temp_dir_name = this->pcd_files_directory;
			temp_dir_name += "temp_dir1";
			this->temp_directories->push_back( temp_dir_name );
			boost::filesystem::path temp_dir( temp_dir_name );
			if( boost::filesystem::create_directory( temp_dir ))
			{
				std::cout << "Success creating temp dir: " << temp_dir_name << "\n";
			}
			//TODO create the new temp dir on the filesystem
		}
		//if this is not the first recursive step then we will name the new temp dir based on how many
		//times we have recursed
		else
		{
			std::string new_temp_dir_name = this->pcd_files_directory;
			new_temp_dir_name += "temp_dir";
			new_temp_dir_name += temp_dir_count + 1;

			boost::filesystem::path temp_dir( new_temp_dir_name );
			if( boost::filesystem::create_directory( temp_dir ))
			{
				std::cout << "Success creating temp dir: " << new_temp_dir_name << "\n";
			}

			//we have to keep track of the paths to all these temp dirs for cleanup later
			this->temp_directories->push_back( new_temp_dir_name );

			//TODO create the new temp dir on the filesystem
		}



		switch( this->num_threads )
		{
		case THREAD_COUNT::THREAD_1:
			this->setupWorkerThreads( 1 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;

		case THREAD_COUNT::THREAD_2:
			this->setupWorkerThreads( 2 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;

		case THREAD_COUNT::THREAD_4:
			this->setupWorkerThreads( 4 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;

		case THREAD_COUNT::THREAD_8:
			this->setupWorkerThreads( 8 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;

		case THREAD_COUNT::THREAD_16:
			this->setupWorkerThreads( 16 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;

		default:
			this->setupWorkerThreads( 1 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;
		}

		//spin up all the allocated threads
		for( auto itr = this->worker_threads->begin() ; itr != this->worker_threads->end() ; ++itr )
		{
			CloudStitchingThread* temp = *itr;
			temp->start();
		}


		//wait for all the threads to finish up there work and delete each thread from the container as
		//it finishes
		while( this->worker_threads->size() > 0 )
		{
			for( auto itr = this->worker_threads->begin() ; itr != this->worker_threads->end() ; ++itr )
			{
				CloudStitchingThread* temp = *itr;
				if( temp->isFinished() )
				{
					worker_threads->erase( itr );
					delete temp;
					break;
				}
			}
		}


		//TODO add in the recursive function call here

		//TODO after we hit the base case, we need to cleanup those temp directories that were created as we unwind the stack
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

	void CloudStitcher::setupWorkerThreads( unsigned int thread_count , unsigned int num_files , std::string output_dir )
	{
		//offset is the number of pcd files that should be allocated to each thread
		unsigned int offset = (int)std::ceil( num_files / thread_count );

		//current offset will be a changing variable representing where in the filename array we are looking at
		auto current_offset = this->pcd_filenames->begin();

		//if we are using one thread, then just copy all of the filenames into the single thread
		if( thread_count == 1 )
		{
			std::vector< std::string > param_vec( current_offset , this->pcd_filenames->end() );
			this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec , output_dir ));
		}

		//otherwise we are going to divide up the filename array as evenly as possible among all the threads
		else
		{
			for( int i = 0 ; i < thread_count - 1 ; ++i )
			{
				auto last = ( current_offset + offset );
				std::vector< std::string > param_vec( current_offset , last );
				this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec , output_dir ));
				current_offset = last;
			}

			//to prevent a seg-fault we just copy whatever is left of the filename array into the last thread
			std::vector< std::string > param_vec( current_offset , this->pcd_filenames->end() );
			this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec , output_dir ));

		}
	}








	CloudStitcher::CloudStitchingThread::CloudStitchingThread( const std::vector< std::string >& files , std::string output_dir )
	{
		//we will make a copy of the list of target filenames for this instance of the class
		this->file_list = new std::vector< std::string >( files );
		this->worker_thread_is_finished = false;

		//make sure we save a copy of the path to the output directory
		this->output_directory = output_dir;
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
		boost::filesystem::path current_path( this->file_list->at(0) );
		std::string curr_file = boost::filesystem::basename( current_path );
		std::string filename = this->output_directory;
		filename += curr_file;
		std::cout << "new file: " << filename << "\n";
		std::ofstream outfile( filename );
		outfile << "hello world\n";
		outfile.close();

		/*
		std::cout << "worker is going to sleep\n";
		boost::posix_time::seconds workTime( 10);
		boost::this_thread::sleep( workTime );
		std::cout << "worker woke up\n";
		*/
	}

} /* namespace vba */
