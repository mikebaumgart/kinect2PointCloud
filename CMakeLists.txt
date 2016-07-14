cmake_minimum_required(VERSION 2.8)
project(demo CXX)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")

option(WITH_OPENCL "adds opencl support for freenect2" OFF)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Freenect2 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${FREENECT2_INCLUDE_DIRS})

add_executable(demo demo.cpp)

target_link_libraries(demo ${OpenCV_LIBS} ${FREENECT2_LIBRARY} ${PCL_LIBRARIES})
