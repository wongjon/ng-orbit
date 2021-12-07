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

    open3d::io::ReadPointCloud("ybeam2.ply", pc2);
    open3d::io::ReadPointCloud("finer_Yoke.PLY", pc1);

    //Eigen::Vector3d center;
    //center << 0.0, 0.0, 0.0;
    //pc1.Scale(1000, center);
    
    

    // remove outliers from pc1
    // auto pointcloud_nonoutlier = pointcloud_1.RemoveStatisticalOutliers(100, 2.0);
    // pointcloud_1 = *(std::get<0>(pointcloud_nonoutlier));

    // downsize pc1
    auto voxel_ds = 0.02;
    auto pc2_down = *pc2.VoxelDownSample(voxel_ds);

    // auto sp1 = std::make_shared<open3d::geometry::PointCloud>(pc1);
    // auto sp2 = std::make_shared<open3d::geometry::PointCloud>(pc2);
    // open3d::visualization::DrawGeometries({sp1, sp2}, "Raw", 1920, 1080);

    // estimate normals for the pointclouds
    pc2_down.EstimateNormals();
    pc1.EstimateNormals();
    pc2.EstimateNormals();

    auto pc1_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(pc1);
    auto pc2_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(pc2);
    auto pc2_down_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(pc2_down);

    // auto pointpointdist = pc1_down.ComputePointCloudDistance(pointcloud_2);
    // std::cout << std::accumulate(pointpointdist.begin(), pointpointdist.end(), 0)/pointpointdist.size() << std::endl;
    std::cout << "Number of points in box pc: " << pc1.points_.size() << std::endl;
    std::cout << "Number of points in room pc: " << pc2.points_.size() << std::endl;
    // std::cout << "Number of points in downsampled room pc: " << pc2_down.points_.size() << std::endl;
    // std::cout << pc1_down.ComputePointCloudDistance(pointcloud_2).at(10) << std::endl;
    // std::cout << pc2.points_.at(10) << std::endl;

    std::cout << pc2.points_.at(10) << std::endl;
    for (int i = 0; i < pc2.points_.size(); i++){
        if (pc2.points_.at(i)[0] > 0.3){ //left right
            pc2.points_.at(i) << 0, 0, 0;
        } else if (pc2.points_.at(i)[0] < -0.2){
            pc2.points_.at(i) << 0, 0, 0;
        }
    }
    
    std::cout << "starting RANSAC" << std::endl;
    
    // RANSAC global registration
    auto distance_threshold = 1;
    // std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> checkers;
    // auto basedonlength = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
    // auto basedondistance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
    // checkers.push_back(basedonlength);
    // checkers.push_back(basedondistance);

    // open3d::pipelines::registration::RANSACConvergenceCriteria ransac_criteria;
    // auto result_RANSAC = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(pointcloud_1, pc2_down, *pointcloud_1fpfh, *pc2_down_fpfh, true, distance_threshold, open3d::pipelines::registration::TransformationEstimationPointToPoint(false), 10000);
    

    auto result_RANSAC = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(pc1, pc2, *pc1_fpfh, *pc2_fpfh, true, distance_threshold, open3d::pipelines::registration::TransformationEstimationPointToPoint(false), 1000);

    // output 4x4 matrix for rotation/translation 
    std::cout << result_RANSAC.transformation_ << std::endl;
    open3d::io::WritePointCloudToPLY("transformedbox.ply", pc1.Transform(result_RANSAC.transformation_), open3d::io::WritePointCloudOption());
    open3d::io::WritePointCloudToPLY("transformedroom.ply", pc2, open3d::io::WritePointCloudOption());
    auto cloud_ptr1 = open3d::io::CreatePointCloudFromFile("transformedroom.ply");
    auto cloud_ptr2 = open3d::io::CreatePointCloudFromFile("transformedbox.ply");
    open3d::visualization::DrawGeometries({cloud_ptr1, cloud_ptr2}, "Global Registration", 1920, 1080);

    // point to plane ICP 
    // auto threshold = 2*voxel_ds;
    // auto trans_init = result_RANSAC.transformation_;
    
    std::cout << "starting ICP" << std::endl;

    open3d::pipelines::registration::ICPConvergenceCriteria icp_criteria;
    icp_criteria.max_iteration_ = 2000;
    auto reg_p2l = open3d::pipelines::registration::RegistrationICP(pc1, pc2, 0.15, Eigen::Matrix4d::Identity(), open3d::pipelines::registration::TransformationEstimationPointToPoint(false), icp_criteria);
    
    pc1.Transform(reg_p2l.transformation_);


    // pointcloud_2.Transform(result_RANSAC.transformation_);

    // pointpointdist = pc1_down.ComputePointCloudDistance(pointcloud_2);
    // std::cout << std::accumulate(pointpointdist.begin(), pointpointdist.end(), 0)/pointpointdist.size() << std::endl;
    std::cout << reg_p2l.transformation_ << std::endl;

    open3d::io::WritePointCloudToPLY("transformedroom.ply", pc2, open3d::io::WritePointCloudOption());
    open3d::io::WritePointCloudToPLY("transformedbox.ply", pc1, open3d::io::WritePointCloudOption());

    cloud_ptr1 = open3d::io::CreatePointCloudFromFile("transformedroom.ply");
    cloud_ptr2 = open3d::io::CreatePointCloudFromFile("transformedbox.ply");
    open3d::visualization::DrawGeometries({cloud_ptr1, cloud_ptr2}, "Local Registration", 1920, 1080);
    
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
