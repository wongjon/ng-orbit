// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

// https://github.com/IntelRealSense/librealsense/issues/2637
// need to build librealsense2 sdk with https://github.com/IntelRealSense/librealsense/blob/master/doc/rs400/rs400_advanced_mode.md

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <algorithm>            // std::min, std::max
#include <iostream>
#include "open3d/Open3D.h"
#include <librealsense2/rs_advanced_mode.hpp>

int main(int argc, char * argv[]) try
{
    rs2::pointcloud pc;
    rs2::points points;
    rs2::config cfg;

    rs2::context ctx;
    rs2::pipeline pipe;
    
    bool depth_found = false;
    bool color_found = false;
    rs2::sensor depth_sensor;
    rs2::sensor color_sensor;
    /*
    for (auto&& dev : ctx.query_devices()){
        auto advanced_dev = dev.as<rs400::advanced_mode>();
        auto advanced_sensors = advanced_dev.query_sensors();

        for (auto&& sensor : advanced_sensors){
            std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);

            if(module_name == "Stereo Module"){
                depth_sensor = sensor;
                depth_found = true;
            } else if(module_name == "RGB Camera"){
                color_sensor = sensor;
                color_found = true;
            }
        }
    }

    color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
    color_sensor.set_option(RS2_OPTION_EXPOSURE, 4000);
    */
    cfg.enable_stream(RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_BGR8 , 30);


    pipe.start(cfg);
    // pipe.start();

    auto threshold_filter = rs2::threshold_filter(0.78f, 1.2f);
    auto frames = pipe.wait_for_frames();
    auto color = frames.get_color_frame();
    if (!color)
        color = frames.get_infrared_frame();

    pc.map_to(color);

    auto depth = frames.get_depth_frame();
    auto filtered_depth = threshold_filter.process(depth);

    points = pc.calculate(filtered_depth);

    points.export_to_ply("roomwithbox.ply", color);

    auto cloud_ptr = open3d::io::CreatePointCloudFromFile("roomwithbox.ply");
    open3d::visualization::DrawGeometries({cloud_ptr}, "TestPLYFileFormat", 1920, 1080);

    // auto cloud_ptr2 = open3d::io::CreatePointCloudFromFile("Box1.PLY");
    // open3d::visualization::DrawGeometries({cloud_ptr2}, "Box", 1920, 1080);

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
