project(Volcanobot)
set	(PROJECT_SOURCES /media/psf/Home/Documents/Github/cs4738/volcanobot/src)
set (PROJECT_INCLUDES /media/psf/Home/Documents/Github/cs4738/volcanobot//include)

cmake_minimum_required(VERSION 3.2.2 FATAL_ERROR)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${PROJECT_INCLUDES})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(vba oni-to-pcd.cpp)
target_link_libraries(vba ${PCL_LIBRARIES} ${PROJECT_SOURCES})
