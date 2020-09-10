cmake_minimum_required(VERSION 3.10)
project(RS_D435_Basics)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV 4.3.0 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(RSD435_OpenCV_And_PCL src/RSD435_OpenCV_And_PCL.cpp)
target_link_libraries(RSD435_OpenCV_And_PCL ${realsense2_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(RS_D435_PCL src/RS_D435_PCL.cpp include/render/render.cpp include/processPointClouds.cpp)
target_link_libraries(RS_D435_PCL ${realsense2_LIBRARY} ${PCL_LIBRARIES})
