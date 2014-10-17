#Basic project setup
cmake_minimum_required(VERSION 2.6)
project(Example)

#Specify all source files to compile
set(EXAMPLE_SOURCES main.cpp)

#Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -lpthread")

#Set include folders
set(EXAMPLE_INCLUDE_PATHS ${CMAKE_CURRENT_SOURCE_DIR}
	"${CMAKE_CURRENT_SOURCE_DIR}/../Source")

#Set include folders
include_directories(${EXAMPLE_INCLUDE_PATHS})

add_executable(Example ${EXAMPLE_SOURCES})
target_link_libraries(Example ARDrone)
