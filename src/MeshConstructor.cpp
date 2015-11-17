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
		this->output_filetype = OBJ;
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

		std::cout << "made it here\n";
	}

} /* namespace vba */
