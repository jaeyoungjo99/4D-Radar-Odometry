/****************************************************************************/
// Module:      registration.hpp
// Description: registration algorithm
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      June 19, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX - 
/****************************************************************************/

#ifndef __REGISTRATION_HPP__
#define __REGISTRATION_HPP__
#pragma once

#include <Eigen/Core>
#include <utility>
#include <vector>

// PCL
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include "types/point_type.hpp"
#include "algorithm/voxel_hash_map.hpp"

namespace radar_odometry {

// Using 06/26
Eigen::Matrix4d RegisterScan2Map(const std::vector<RadarPoint> &frame,
                                    const VoxelHashMap &voxel_map,
                                    const  Eigen::Matrix4d &initial_guess,
                                    double max_correspondence_distance,
                                    double kernel);

// New KISS ICP Syle
Eigen::Matrix4d RegisterFrame(const std::vector<RadarPoint> &frame,
                           const VoxelHashMap &voxel_map,
                           const Eigen::Matrix4d &initial_guess,
                           double max_correspondence_distance,
                           double kernel);

Eigen::Matrix4d RegisterFrameDoppler(const std::vector<RadarPoint> &frame,
                           const VoxelHashMap &voxel_map,
                           const Eigen::Matrix4d &initial_guess,
                           const Velocity &sensor_velocity,
                           double max_correspondence_distance,
                           double kernel);

Eigen::Matrix4d RegisterFrame3DoF(const std::vector<RadarPoint> &frame,
                           const VoxelHashMap &voxel_map,
                           const Eigen::Matrix4d &initial_guess,
                           double max_correspondence_distance,
                           double kernel);

Eigen::Matrix4d RegisterFrameDoppler3DoF(const std::vector<RadarPoint> &frame,
                           const VoxelHashMap &voxel_map,
                           const Eigen::Matrix4d &initial_guess,
                           const Eigen::Matrix4d &last_pose,
                           double max_correspondence_distance,
                           double kernel);

}

#endif  // __REGISTRATION_HPP__