cmake_minimum_required(VERSION 3.5)
project(RS_D435_Basics)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

find_package(PCL REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(DEPENDENCIES ${realsense2_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(RSD435_OpenCV_And_PCL src/RSD435_OpenCV_And_PCL.cpp)
target_link_libraries(RSD435_OpenCV_And_PCL ${DEPENDENCIES})

add_executable(RS_D435_PCL src/RS_D435_PCL.cpp include/render/render.cpp include/processPointClouds.cpp)
target_link_libraries(RS_D435_PCL ${DEPENDENCIES})

add_executable(Drone_Environment src/Drone_Environment.cpp include/render/render.cpp include/processPointClouds.cpp)
target_link_libraries(Drone_Environment ${DEPENDENCIES})
