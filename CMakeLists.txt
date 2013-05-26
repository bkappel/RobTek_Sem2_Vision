cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(vision_lego_kinect)
#PCL
find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#OPENCV
find_package(OpenCV REQUIRED)


#ROS
#find_package(roscpp REQUIRED)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
#rosbuild_add_executable(testTF src/test/main.cpp)
#target_link_libraries(testTF TF)

#add_executable(grasper kinectpointcloud.cpp)
rosbuild_add_executable(grasper kinectpointcloud.cpp)
target_link_libraries(grasper ${PCL_LIBRARIES} ${OpenCV_LIBS})
