// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <iostream>
#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>
#include "open3d/Open3D.h"

using namespace std;
using namespace cv;

int main(int argc, char * argv[]) //try
{
    open3d::t::io::RealSenseSensor::ListDevices();
    
    /*while (!true){      
        open3d::t::io::RealSenseSensorConfig rs_cfg;
        open3d::io::ReadIJsonConvertible(config_filename, rs_cfg);
    }*/

    return EXIT_SUCCESS;
}
/*catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}*/