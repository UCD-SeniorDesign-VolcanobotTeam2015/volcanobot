cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project( Cloud_Stitcher )

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(../include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable ( Cloud_Stitcher 
	main.cpp 
	CloudStitcher.cpp
	../include/CloudStitcher.h)
target_link_libraries ( Cloud_Stitcher ${PCL_LIBRARIES})
