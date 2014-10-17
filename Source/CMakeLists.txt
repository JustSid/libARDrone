#Basic project setup
cmake_minimum_required(VERSION 2.6)
project(ARDrone)

#Specify all source files to compile
set(LIBARDRONE_SOURCES
	ARATService.h
	ARATService.cpp
	ARConfigService.h
	ARConfigService.cpp
	ARControlService.h
	ARControlService.cpp
	ARDrone.h
	ARDrone.cpp
	ARNavdataService.h
	ARNavdataService.cpp
	ARService.h
	ARService.cpp
	ARSocket.h
	ARSocket.cpp
	ARVector.h
	ARVideoService.h
	ARVideoService.cpp)

#Set include folders
set(LIBARDRONE_INCLUDE_PATHS ${CMAKE_CURRENT_SOURCE_DIR})

#Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -lpthread")

#Set include folders
include_directories(${LIBARDRONE_INCLUDE_PATHS})

#Set architecture
set(CMAKE_OSX_ARCHITECTURES x86_64)

#Create a shared library
add_library(ARDrone STATIC ${LIBARDRONE_SOURCES})
