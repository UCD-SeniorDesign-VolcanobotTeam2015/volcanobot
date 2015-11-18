#-------------------------------------------------
#
# Project created by QtCreator 2015-10-08T15:45:14
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = vbaUI
TEMPLATE = app

# Input
INCLUDEPATH += /usr/local/Cellar/boost/1.59.0/include \
			/usr/local/Cellar/pcl/HEAD/include/pcl-1.8 \
			/usr/local/Cellar/eigen/3.2.6/include/eigen3 \ 
			/usr/local/Cellar/flann/1.8.4_1/include \
			/usr/local/Cellar/vtk/6.3.0/include/vtk-6.3 \
# 			/usr/local/Cellar/openni2/2.2.0.33/include/ni2

# LIBS +=  /usr/local/Cellar/boost/1.59.0/lib \
# 			/usr/local/Cellar/pcl/HEAD/lib \
# 			/usr/local/Cellar/eigen/3.2.6/lib \
# 			/usr/local/Cellar/flann/1.8.4_1/lib \
# 			/usr/local/Cellar/vtk/6.3.0/lib \
# 			/usr/local/Cellar/openni2/2.2.0.33/lib

HEADERS += ../include/oni-to-pcd.h \
           ../include/errorMsgHandler.h \
           ../include/mainwindow.h \
           ../include/CloudStitcher.h \
           ../include/PCDRegistration.h

FORMS += ../src/mainwindow.ui

SOURCES += ../src/cloud_registration.cpp \
           ../src/CloudStitcher.cpp \
           ../src/errorMsgHandler.cpp \
           ../src/main.cpp \
           ../src/mainwindow.cpp \
           ../src/oni-to-pcd.cpp \
           ../src/PCDRegistration.cpp 