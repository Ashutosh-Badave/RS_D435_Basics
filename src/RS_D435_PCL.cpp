//
// Base: Intel tutorial of realsense with PCL.
// Modification: Ashutosh Badave
//

#include <iostream>
#include <numeric>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;

struct Color
{

    float r, g, b;

    Color(float setR, float setG, float setB)
            : r(setR), g(setG), b(setB)
    {}
};


std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(
            texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();

    for (int i = 0; i < points.size(); ++i)
    {
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
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::string name, Color color = Color(1,1,1))
{

    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, name);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
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

int main()try
{
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis();

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    while (!viewer->wasStopped ())
    {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();

    auto depth = frames.get_depth_frame();
    auto color = frames.get_color_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);
    pc.map_to(color);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_points = points_to_pcl(points,color);

    /*pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_points);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud_filtered);

    std::vector<pcl_ptr> layers;
    layers.push_back(pcl_points);
    layers.push_back(cloud_filtered);*/

    renderPointCloud(viewer,pcl_points,"data");


        viewer->spinOnce ();
    }


}catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}