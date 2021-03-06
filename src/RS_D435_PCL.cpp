//
// Base: Intel tutorial of realsense with PCL.
// Modification: Ashutosh Badave
//

#include <iostream>
#include <fstream>
#include <numeric>
#include <pthread.h>
#include<string>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include "../include/processPointClouds.h"
#include "../include/processPointClouds.cpp"

static volatile bool keep_running = true;
static volatile bool captureLoop = false;

class advanced_mode;

class advanced_mode;

static void *userInput_thread(void *) {
    //bool captureLoop;
    bool inputCheck = false;
    char takeFrame; // Utilize to trigger frame capture from key press ('t')
    do {

        // Prompt User to execute frame capture algorithm
        cout << endl;
        cout << "Generate a Point Cloud? [y/n] ";
        cin >> takeFrame;
        cout << endl;

        // Condition [Y] - Capture frame, store in PCL object and display
        if (takeFrame == 'y' || takeFrame == 'Y') {
            captureLoop = true;
            keep_running = true;
            takeFrame = 0;

        }
            // Condition [N] - Exit Loop and close program
        else if (takeFrame == 'n' || takeFrame == 'N') {
            captureLoop = false;
            keep_running = true;
            takeFrame = 0;
        }
            // Invalid Input, prompt user again.
        else {
            keep_running = false;
            cout << "Invalid Input." << endl;
            takeFrame = 0;
        }

    } while(keep_running == true);
/*    while(keep_running) {
        if (std::cin.get() == 's')
        {
            captureLoop = true;
            //! desired user input 's' received

        }
        if (std::cin.get() == 'q')
        {
            //! desired user input 'q' received
            keep_running = false;
        }

    }*/
}


std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords) {
    const int w = texture.get_width(), h = texture.get_height();
    int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);
    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t *>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(
            texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pclrgb(const rs2::points &points, const rs2::video_frame &color) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();

    for (int i = 0; i < points.size(); ++i) {
        cloud->points[i].x = vertices[i].x;
        cloud->points[i].y = vertices[i].y;
        cloud->points[i].z = vertices[i].z;

        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(color, tex_coords[i]);

        cloud->points[i].r = std::get<0>(current_color);
        cloud->points[i].g = std::get<1>(current_color);
        cloud->points[i].b = std::get<2>(current_color);
    }
    return cloud;
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                 std::string name, Color color = Color(1, 1, 1)) {

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}


pcl::visualization::PCLVisualizer::Ptr simpleVis() {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    //viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->setCameraPosition(0, -5, -5, 0, 0, 0, 0, 0, 1);
    viewer->addCoordinateSystem(1.0);
    return (viewer);
}

int main() try {
    pthread_t tId;
    (void) pthread_create(&tId, 0, userInput_thread, 0);
    //======================
    // Variable Declaration
    //======================

    int i = 1;

    //====================
    // Object Declaration
    //====================
    ProcessPointClouds<pcl::PointXYZRGB> *pointProcessorRGB = new ProcessPointClouds<pcl::PointXYZRGB>();
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, -5, -5, 0, 0, 0, 0, 0, 1);
    viewer->addCoordinateSystem(1.0);

    pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("3D Viewer1"));
    viewer1->setBackgroundColor(0, 0, 10);
    viewer1->setCameraPosition(0, -5, -5, 0, 0, 0, 0, 0, 1);
    viewer1->addCoordinateSystem(1.0);
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
    // Start streaming with default recommended configuration
    /*std::ifstream file("./camera_settings.json");
    if (file.good())
    {
        std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        auto prof = cfg.resolve(pipe);
        if (auto advanced = prof.get_device().as<rs400::advanced_mode>())
        {
            advanced.load_json(str);
        }
    }
    else
    {
        std::cout << "Couldn't find camera-settings.json, skipping custom settings!" << std::endl;
    }*/

    pipe.start(cfg);
    // Loop and take frame captures upon user input
    while (!viewer->wasStopped()) {


        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer1->removeAllPointClouds();
        viewer1->removeAllShapes();


//        // Wait for frames from the camera to settle
//        for (int i = 0; i < 30; i++) {
//            auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
//        }

        // Wait for the next set of frames from the camerae
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        pc.map_to(color);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_points_RGB = points_to_pclrgb(points, color);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points_noRGB = points_to_pcl(points, color);
        /*pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_points);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter(*cloud_filtered);

        std::vector<pcl_ptr> layers;
        layers.push_back(pcl_points);
        layers.push_back(cloud_filtered);*/

        renderPointCloud(viewer, pcl_points_RGB, "data");


        viewer1->addPointCloud<pcl::PointXYZ>(pcl_points_noRGB, "Data1");
        viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Data1");

        if (captureLoop == true) {
            std::string cloudFile_RGB = "../Captured_Frame_rgb_" + std::to_string(i) + ".pcd";
            std::string cloudFile_noRGB = "../Captured_Frame_norgb_" + std::to_string(i) + ".pcd";
            pointProcessorRGB->savePcd(pcl_points_RGB, cloudFile_RGB);
            pointProcessor->savePcd(pcl_points_noRGB, cloudFile_noRGB);
            captureLoop == false;
            i++;

        }
        viewer->spinOnce(100);
        viewer1->spinOnce(100);


    }
    (void) pthread_join(tId, NULL);

    return 0;

} catch (const rs2::error &e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
              << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}