cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project( VBA )

find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)

set (PROJECT_SOURCES 
main.cpp oni-to-pcd.cpp 
mainwindow.cpp 
errorMsgHandler.cpp 
CloudStitcher.cpp 
PCDRegistration.cpp )

set (project_HEADERS 
../include/errorMsgHandler.h 
../include/oni-to-pcd.h 
../include/mainwindow.h 
../include/CloudStitcher.h 
../include/PCDRegistration.h)

set (project_FORMS   mainwindow.ui)
set (VTK_LIBRARIES   vtkRendering 
vtkGraphics 
vtkHybrid 
QVTK)

set (PROJECT_EXECS ../src/main.cpp 
../src/oni-to-pcd.cpp 
../src/errorMsgHandler.cpp 
../src/mainwindow.cpp 
../src/CloudStitcher.cpp 
../src/PCDRegistration.cpp)

include_directories (${PCL_INCLUDE_DIRS} 
../include)

include_directories (${QT_USE_FILE})

link_directories (${PCL_LIBRARY_DIRS})

add_definitions (${PCL_DEFINITIONS})

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI (project_FORMS_HEADERS ${project_FORMS})


add_definitions (${QT_DEFINITIONS})

add_executable (vba ${project_SOURCES}
${PROJECT_EXECS} )

target_link_libraries(vba ${PCL_LIBRARIES} ${QT_LIBRARIES} ${VTK_LIBRARIES})
