/*
 * MeshConstructor.h
 *
 *  Created on: Nov 10, 2015
 *      Author: matt
 */

#ifndef MESHCONSTRUCTOR_H_
#define MESHCONSTRUCTOR_H_

#include <string>
#include <iostream>

#include <boost/filesystem.hpp>

namespace vba
{

	enum MESH_FILETYPE
	{
		OBJ
	};

	class MeshConstructor
	{
		public:

			MeshConstructor();
			~MeshConstructor();

			int setInputFilename( std::string filename );

			int setOutputFilename( std::string filename , MESH_FILETYPE type );

			int constructMesh();

		private:

			std::string input_filename;
			std::string output_filename;

			MESH_FILETYPE output_filetype;

	};

} /* namespace vba */

#endif /* MESHCONSTRUCTOR_H_ */
