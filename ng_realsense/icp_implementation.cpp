// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

// https://github.com/IntelRealSense/librealsense/issues/2637
// need to build librealsense2 sdk with https://github.com/IntelRealSense/librealsense/blob/master/doc/rs400/rs400_advanced_mode.md

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <algorithm>            // std::min, std::max
#include <iostream>
#include "open3d/Open3D.h"
#include <librealsense2/rs_advanced_mode.hpp>
#include <Eigen/Dense>

using namespace Eigen;

int main(int argc, char * argv[]) try
{
    rs2::pointcloud pc;
    rs2::points points;
    rs2::config cfg;

    rs2::context ctx;
    rs2::pipeline pipe;

    open3d::geometry::PointCloud pointcloud_1;
    open3d::geometry::PointCloud pointcloud_2;

    open3d::io::ReadPointCloud("roomwithbox.ply", pointcloud_1);
    open3d::io::ReadPointCloud("Box1.PLY", pointcloud_2);

    std::cout << "finished reading pointclouds" << std::endl;
    std::cout << "begin preprocessing" << std::endl;

    auto pointcloud_nonoutlier = pointcloud_1.RemoveStatisticalOutliers(100, 2.0);
    pointcloud_1 = *(std::get<0>(pointcloud_nonoutlier));

    auto voxel_ds = 0.02;
    auto pc1_down = *pointcloud_1.VoxelDownSample(voxel_ds);

    auto sp1 = std::make_shared<open3d::geometry::PointCloud>(pc1_down);
    // open3d::visualization::DrawGeometries({sp1}, "Downsampled", 1920, 1080);


    pc1_down.EstimateNormals();
    pointcloud_1.EstimateNormals();
    pointcloud_2.EstimateNormals();

    // auto pointcloud_1fpfh = open3d::pipelines::registration::ComputeFPFHFeature(pointcloud_1);
    auto pc1_down_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(pc1_down);
    auto pointcloud_2fpfh = open3d::pipelines::registration::ComputeFPFHFeature(pointcloud_2);

    auto pointpointdist = pc1_down.ComputePointCloudDistance(pointcloud_2);
    std::cout << std::accumulate(pointpointdist.begin(), pointpointdist.end(), 0)/pointpointdist.size() << std::endl;
    std::cout << pc1_down.ComputePointCloudDistance(pointcloud_2).at(10) << std::endl;

    std::cout << "begin RANSAC based on feature matching" << std::endl;


    auto distance_threshold = 1000;
    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> checkers;
    // auto basedonlength = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto basedondistance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
    // checkers.push_back(basedonlength);
    checkers.push_back(basedondistance);

    auto result_RANSAC = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(pc1_down, pointcloud_2, *pc1_down_fpfh, *pointcloud_2fpfh, true, distance_threshold, open3d::pipelines::registration::TransformationEstimationPointToPoint(false), 3.0, checkers);

    std::cout << "finished RANSAC" << std::endl;
    std::cout << result_RANSAC.transformation_ << std::endl;

    Eigen::Vector3d center;
    center << 0.0, 0.0, 0.0;
    pointcloud_2.Scale(0.0006, center);
    
    auto threshold = 0.5;
    auto trans_init = result_RANSAC.transformation_;

    auto reg_p2l = open3d::pipelines::registration::RegistrationICP(pointcloud_2, pointcloud_1, threshold, trans_init, open3d::pipelines::registration::TransformationEstimationPointToPlane());
    
    pointcloud_2.Transform(reg_p2l.transformation_);
    std::cout << reg_p2l.fitness_ << std::endl;
    std::cout << reg_p2l.transformation_ << std::endl;

    pointpointdist = pc1_down.ComputePointCloudDistance(pointcloud_2);
    std::cout << std::accumulate(pointpointdist.begin(), pointpointdist.end(), 0)/pointpointdist.size() << std::endl;

    Eigen::Vector3d color; 
    color << 0.22, 1.0, 0.08;
    pointcloud_2.PaintUniformColor(color);

    open3d::io::WritePointCloudToPLY("transformedbox.ply", pointcloud_2, open3d::io::WritePointCloudOption());


    auto cloud_ptr1 = open3d::io::CreatePointCloudFromFile("roomwithbox.ply");
    auto cloud_ptr2 = open3d::io::CreatePointCloudFromFile("transformedbox.ply");
    open3d::visualization::DrawGeometries({cloud_ptr1, cloud_ptr2}, "TestPLYFileFormat", 1920, 1080);
    
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
