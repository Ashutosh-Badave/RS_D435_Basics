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

    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = pointProcessorI->loadPcd("../Captured_Frame_test.pcd");
    //ProcessPointClouds<pcl::PointXYZRGB> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZRGB>();
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud = pointProcessorRGB->loadPcd("../Captured_Frame_rgb_1.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noRGB = pointProcessor->loadPcd("../Captured_Frame_norgb_1.pcd");
    // TODO: Downsizing of pointclouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = pointProcessor->FilterCloud(cloud_noRGB, 0.18,
                                                                                    Eigen::Vector4f(-2, -2, 0,
                                                                                                    1),
                                                                                    Eigen::Vector4f(2, 2, 10, 1));

    // TODO: Segmentation of pointclouds
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentCloud = pointProcessor->SegmentPlane(
            filteredCloud, 100, 0.5);

    /*cloud_noRGB->width = inputCloud->width;
    cloud_noRGB->height = inputCloud->height;
    cloud_noRGB->is_dense = false;
    cloud_noRGB->points.resize(inputCloud->size());

    for(int i = 0; i < inputCloud->size(); ++i)
    {
        cloud_noRGB->points[i].x = inputCloud->points[i].x;
        cloud_noRGB->points[i].y = inputCloud->points[i].y;
        cloud_noRGB->points[i].z = inputCloud->points[i].z;
    }*/

    // Visualization of pointclouds
/*    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, -5, -5, 0, 0, 0, 0, 0, 1);
    viewer->addCoordinateSystem(1.0);
    viewer->addPointCloud<pcl::PointXYZRGB>(inputCloud, "Data");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Data");*/

    pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("3D Viewer1"));
    viewer1->setBackgroundColor(0, 0, 0);
    viewer1->setCameraPosition(0, -5, -5, 0, 0, 0, 0, 0, 1);
    viewer1->addCoordinateSystem(1.0);
    renderPointCloud(viewer1, SegmentCloud.first, "planeCloud", Color(1, 0, 0));
    while (!viewer1->wasStopped()) {

        viewer1->spinOnce();
    }
    return 0;
}