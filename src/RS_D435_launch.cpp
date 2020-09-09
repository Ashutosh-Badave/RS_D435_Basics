//
// Base: Intel tutorial of Hello realsense.
// Modification: Ashutosh Badave
//

#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <librealsense2/rs.hpp>
#include "../include/cv-helper.hpp" // from Intel OpenCV samples for Intel realsense camera


using namespace cv;

int main(int argc, char * argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();


    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    //resizeWindow(window_name,1200,800);
    int k = cv::waitKey(0); // Wait for a keystroke in the window
    Mat depth_color,color_mat,depth_mat;

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        // Try to get a frame of a depth image
        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();
        rs2::frame depth = depth_frame.apply_filter(color_map);

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


    }
    if (k=='s')
    {
       // Mat dst;

        imwrite("../output/depth_White.png",depth_mat);
        imwrite("../output/color_img.png",color_mat);
        imwrite("../output/depth_color.png",depth_color);
        std::cout << "Image is saved " << std::endl;
    }

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