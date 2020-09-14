//
// Created by ashutosh on 14.09.20.
//

#include <iostream>
#include <numeric>
#include <pthread.h>
#include<string>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../include/processPointClouds.h"
#include "../include/processPointClouds.cpp"
#include "../include/render/render.h"


int main() {
    std::cout << "starting enviroment" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, -5, -5, 0, 0, 0, 0, 0, 1);
    viewer->addCoordinateSystem(1.0);

    ProcessPointClouds<pcl::PointXYZRGB> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZRGB>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud = pointProcessorI->loadPcd("../Data/Captured_Frame1.pcd");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.18,
                                                                                        Eigen::Vector4f(-7.5, -5, -2,
                                                                                                        1),
                                                                                        Eigen::Vector4f(32, 6.5, 2, 1));

    viewer->addPointCloud<pcl::PointXYZRGB>(filteredCloud, "Data");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Data");


    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
    return 0;
}