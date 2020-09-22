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
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include "../include/processPointClouds.h"
#include "../include/processPointClouds.cpp"
#include "../include/render/render.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points &points, const rs2::video_frame &color) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto &p : cloud->points) {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

void DroneEnvironment(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZ> *pointProcessor,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_noRGB) {
    // ------------------------------------------------------
    // -----Open 3D viewer and display Drone Enviroment -----
    // ------------------------------------------------------
    //  Downsizing of pointclouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = pointProcessor->FilterCloud(cloud_noRGB, 0.18,
                                                                                    Eigen::Vector4f(-2, -2, 0, 1),
                                                                                    Eigen::Vector4f(2, 2, 10, 1));
    //renderPointCloud(viewer,filteredCloud,"filteredCloud");

    // Segmentation of pointclouds
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentCloud = pointProcessor->SegmentPlane(
            filteredCloud, 100, 0.5);
    //renderPointCloud(viewer, SegmentCloud.first, "ObstCloud", Color(1, 0, 0));

    // Clustering of pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(SegmentCloud.first, 0.4,
                                                                                                25, 2000);
    int clusterId = 0, colorid = 2;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {

        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[colorid]);
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        //BoxQ boxQ = pointProcessorI->BoundingBoxQ(cluster);
        // renderBox(viewer,boxQ,clusterId);
        ++clusterId;
        colorid = clusterId % 3;
    }
}

int main() {
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer1"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, -5, -5, 0, 0, 0, 0, 0, 1);
    viewer->addCoordinateSystem(1.0);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 848, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);
    pipe.start(cfg);


    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noRGB = pointProcessor->loadPcd("../Captured_Frame_norgb_1.pcd");
    //ProcessPointClouds<pcl::PointXYZRGB> *pointProcessorRGB = new ProcessPointClouds<pcl::PointXYZRGB>();
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB = pointProcessorRGB->loadPcd("../Captured_Frame_rgb_1.pcd");
    //  Downsizing of pointclouds
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = pointProcessor->FilterCloud(cloud_noRGB, 0.18,
                                                                                    Eigen::Vector4f(-2, -2, 0,
                                                                                                    1),
                                                                                    Eigen::Vector4f(2, 2, 10, 1));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud_RGB = pointProcessorRGB->FilterCloud(cloud_RGB, 0.18,
                                                                                    Eigen::Vector4f(-2, -2, 0,
                                                                                                    1),
                                                                                    Eigen::Vector4f(2, 2, 10, 1));

    // Segmentation of pointclouds
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentCloud = pointProcessor->SegmentPlane(
            filteredCloud, 100, 0.5);
    std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> SegmentCloudRGB = pointProcessorRGB->SegmentPlane(
            filteredCloud_RGB, 100, 0.5);
    //renderPointCloud(viewer, SegmentCloud.first, "ObstCloud", Color(1, 0, 0));
    // Clustering of pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(SegmentCloud.first, 0.4,
                                                                                                12, 1000);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClustersRGB = pointProcessorRGB->Clustering(SegmentCloudRGB.first, 0.4,
                                                                                                12, 1000);
    int clusterId = 0, colorid = 2;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {

        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[colorid]);
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        //BoxQ boxQ = pointProcessorI->BoundingBoxQ(cluster);
        // renderBox(viewer,boxQ,clusterId);
        ++clusterId;
        colorid = clusterId % 3;
    }*/
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

    /* // Visualization of pointclouds
     pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
     viewer->setBackgroundColor(0, 0, 0);
     viewer->setCameraPosition(0, -5, -5, 0, 0, 0, 0, 0, 1);
     viewer->addCoordinateSystem(1.0);
     viewer->addPointCloud<pcl::PointXYZRGB>(SegmentCloudRGB.first, "Filtered");
     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Filtered");

     //viewer1->addPointCloud<pcl::PointXYZ>(SegmentCloud.first, "Data1");
     //viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Data1");
     */
    while (!viewer->wasStopped()) {

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        pc.map_to(color);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noRGB = points_to_pcl(points, color);
        DroneEnvironment(viewer, pointProcessor, cloud_noRGB);

        viewer->spinOnce();
    }
    return 0;
}