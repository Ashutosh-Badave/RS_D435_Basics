//
// Base: Intel tutorial of realsense with OpenCV and PCL.
// Modification: Ashutosh Badave
//

#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

#include "../include/cv-helper.hpp" // from Intel OpenCV samples for Intel realsense camera


using namespace cv;
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

struct Color
{

    float r, g, b;

    Color(float setR, float setG, float setB)
            : r(setR), g(setG), b(setB)
    {}
};

pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1,1,1))
{

    viewer->addPointCloud<pcl::PointXYZ> (cloud, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}


pcl::visualization::PCLVisualizer::Ptr simpleVis ()
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem (1.0);
  return (viewer);
}


int main(int argc, char * argv[]) try
{
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis();

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();


    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    //resizeWindow(window_name,1200,800);
    //int k = cv::waitKey(0); // Wait for a keystroke in the window
    Mat depth_color,color_mat,depth_mat;

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0 && !viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        // Try to get a frame of a depth image
        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();
        rs2::frame depth = depth_frame.apply_filter(color_map);

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth_frame);

        auto pcl_cloud = points_to_pcl(points);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        depth_color = Mat(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

        color_mat = frame_to_mat(color_frame);
        depth_mat = depth_frame_to_meters(depth_frame);

        // Query the distance from the camera to the object in the center of the image
        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth_frame.get_distance(w / 2, h / 2);
        // Update the window with new data
        imshow(window_name, depth_color);

        //convert to grayscale
        cvtColor(depth_color, depth_mat, COLOR_BGR2GRAY);

        imshow("depth window",depth_mat);
        imshow("color window",color_mat);
        // Print the distance
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \n";

        renderPointCloud(viewer,pcl_cloud,"data");

        viewer->spinOnce ();


    }
    /*if (k=='s')
    {
       // Mat dst;

        imwrite("../output/depth_White.png",depth_mat);
        imwrite("../output/color_img.png",color_mat);
        imwrite("../output/depth_color.png",depth_color);
        std::cout << "Image is saved " << std::endl;
    }*/

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}