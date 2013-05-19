cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(vision_lego_kinect)
#PCL
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_DEFINITIONS})

#OPENCV


add_executable(grasper kinectpointcloud.cpp)
target_link_libraries(grasper ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
