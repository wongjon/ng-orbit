// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <iostream>
#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char * argv[]) try
{
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    //cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Configure and start the pipeline
    p.start(cfg);

    // https://github.com/IntelRealSense/librealsense/blob/master/doc/stepbystep/getting_started_with_openCV.md
    rs2::frameset frames;

    
    for (int i = 0; i < 30; i++){
        frames = p.wait_for_frames();
    }

    rs2::frame color_frame = frames.get_color_frame();

    namedWindow("Display D435i", WINDOW_AUTOSIZE);

    Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    imshow("Display D435i", color);

    waitKey(0);

    // https://learnopencv.com/edge-detection-using-opencv/
    Mat img_gray;
    cvtColor(color, img_gray, COLOR_BGR2GRAY);

    Mat img_blur;
    GaussianBlur(img_gray, img_blur, Size(3,3), 0);

    Mat sobelx, sobely, sobelxy;
    Sobel(img_blur, sobelx, CV_64F, 1, 0, 5);
    Sobel(img_blur, sobely, CV_64F, 0, 1, 5);
    Sobel(img_blur, sobelxy, CV_64F, 1, 1, 5);
    imshow("Sobel X", sobelx);
    waitKey(0);
    imshow("Sobel Y", sobely);
    waitKey(0);
    imshow("Sobel XY using Sobel() function", sobelxy);

    waitKey(0);

    Mat edges;
    Canny(img_blur, edges, 100, 200, 3, false);
    imshow("Canny edge detection", edges);
    waitKey(0);

    destroyAllWindows();

    while (!true)
    {
        
        /*
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        float width = depth.get_width();
        float height = depth.get_height();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);

        // Print the distance 
        std::cout << "The camera is facing an object " << dist_to_center << " meters away         \r";
        */

        /*rs2::frameset data = p.wait_for_frames().
                            apply_filter(printer).
                            apply_filter(color_map);*/
        
        
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}