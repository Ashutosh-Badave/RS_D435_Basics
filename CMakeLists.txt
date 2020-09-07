cmake_minimum_required(VERSION 3.10)
project(RS_D435_Basics)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV 4.3.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

#add_executable(jetson_camera Jetson_camera_launch.cpp)
#target_link_libraries(jetson_camera ${OpenCV_LIBS})

