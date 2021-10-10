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

    open3d::geometry::PointCloud pc1;
    open3d::geometry::PointCloud pc2;

    open3d::io::ReadPointCloud("multiway1.ply", pc1);
    open3d::io::ReadPointCloud("multiway2.ply", pc2);

    auto pointcloud_nonoutlier = pc1.RemoveStatisticalOutliers(100, 2.0);
    pc1 = *(std::get<0>(pointcloud_nonoutlier));
    pointcloud_nonoutlier = pc2.RemoveStatisticalOutliers(100, 2.0);
    pc2 = *(std::get<0>(pointcloud_nonoutlier));

    // downsize
    auto voxel_ds = 0.02;
    auto pc1_down = *pc1.VoxelDownSample(voxel_ds);
    auto pc2_down = *pc2.VoxelDownSample(voxel_ds);

    auto sp1 = std::make_shared<open3d::geometry::PointCloud>(pc1);
    auto sp2 = std::make_shared<open3d::geometry::PointCloud>(pc2);

    sp1 = std::make_shared<open3d::geometry::PointCloud>(pc1_down);
    sp2 = std::make_shared<open3d::geometry::PointCloud>(pc2_down);
    // open3d::visualization::DrawGeometries({sp1, sp2}, "Outliers Removed", 1920, 1080);
    // open3d::visualization::DrawGeometries({sp1, sp2}, "Downsampled PCs", 1920, 1080);

    // estimate normals for the pointclouds
    pc1.EstimateNormals();
    pc2.EstimateNormals();
    pc1_down.EstimateNormals();
    pc2_down.EstimateNormals();

    std::cout << "Number of points in box pc: " << pc1_down.points_.size() << std::endl;
    std::cout << "Number of points in room pc: " << pc2_down.points_.size() << std::endl;

    // compute features
    auto pc1_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(pc1);
    auto pc2_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(pc2);
    auto pc1_downfpfh = open3d::pipelines::registration::ComputeFPFHFeature(pc1_down);
    auto pc2_downfpfh = open3d::pipelines::registration::ComputeFPFHFeature(pc2_down);
    
    // RANSAC global registration
    auto distance_threshold = voxel_ds;
    auto result_RANSAC = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(pc1_down, pc2_down, *pc1_downfpfh, *pc2_downfpfh, true, distance_threshold, open3d::pipelines::registration::TransformationEstimationPointToPlane(), 100);
    
    open3d::io::WritePointCloudToPLY("transformedpc1.ply", pc1_down.Transform(result_RANSAC.transformation_), open3d::io::WritePointCloudOption());
    open3d::io::WritePointCloudToPLY("transformedpc2.ply", pc2_down, open3d::io::WritePointCloudOption());
    auto cloud_ptr1 = open3d::io::CreatePointCloudFromFile("transformedpc1.ply");
    auto cloud_ptr2 = open3d::io::CreatePointCloudFromFile("transformedpc2.ply");
    std::cout << result_RANSAC.transformation_ << std::endl;
    open3d::visualization::DrawGeometries({cloud_ptr1, cloud_ptr2}, "global registration", 1920, 1080);

    // ICP params 
    auto thresh_coarse = 5*voxel_ds;
    auto thresh_fine = voxel_ds;
    open3d::pipelines::registration::ICPConvergenceCriteria criteria;
    criteria.max_iteration_ = 1000;

    // // coarse point to plane ICP 
    // auto icp_coarse = open3d::pipelines::registration::RegistrationICP(pc1_down, pc2_down, thresh_coarse, result_RANSAC.transformation_, open3d::pipelines::registration::TransformationEstimationPointToPlane(), criteria);
    
    // open3d::io::WritePointCloudToPLY("transformedpc1.ply", pc1.Transform(icp_coarse.transformation_), open3d::io::WritePointCloudOption());
    // cloud_ptr1 = open3d::io::CreatePointCloudFromFile("transformedpc1.ply");
    // cloud_ptr2 = open3d::io::CreatePointCloudFromFile("multiway2.ply");
    // std::cout << icp_coarse.transformation_ << std::endl;
    // open3d::visualization::DrawGeometries({cloud_ptr1, cloud_ptr2}, "rough ICP", 1920, 1080);
    
    // // fine point to plane ICP
    // auto icp_fine = open3d::pipelines::registration::RegistrationICP(pc1_down, pc2_down, thresh_fine, icp_coarse.transformation_, open3d::pipelines::registration::TransformationEstimationPointToPlane(), criteria);
    
    // open3d::io::WritePointCloudToPLY("transformedpc1.ply", pc1.Transform(icp_fine.transformation_), open3d::io::WritePointCloudOption());
    // open3d::io::WritePointCloudToPLY("transformedpc2.ply", pc2, open3d::io::WritePointCloudOption());
    // cloud_ptr1 = open3d::io::CreatePointCloudFromFile("transformedpc1.ply");
    // cloud_ptr2 = open3d::io::CreatePointCloudFromFile("transformedpc2.ply");
    // std::cout << icp_fine.transformation_ << std::endl;
    // open3d::visualization::DrawGeometries({cloud_ptr1, cloud_ptr2}, "fine ICP", 1920, 1080);

    // fine point to point ICP on full pointclouds
    pc1.Transform(result_RANSAC.transformation_);
    auto icp = open3d::pipelines::registration::RegistrationICP(pc1, pc2, 0.01, Eigen::Matrix4d::Identity(), open3d::pipelines::registration::TransformationEstimationPointToPlane(), criteria);
    open3d::io::WritePointCloudToPLY("transformedpc1.ply", pc1.Transform(icp.transformation_), open3d::io::WritePointCloudOption());
    open3d::io::WritePointCloudToPLY("transformedpc2.ply", pc2, open3d::io::WritePointCloudOption());
    cloud_ptr1 = open3d::io::CreatePointCloudFromFile("transformedpc1.ply");
    cloud_ptr2 = open3d::io::CreatePointCloudFromFile("transformedpc2.ply");
    std::cout << icp.transformation_ << std::endl;
    open3d::visualization::DrawGeometries({cloud_ptr1, cloud_ptr2}, "fine ICP", 1920, 1080);

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
