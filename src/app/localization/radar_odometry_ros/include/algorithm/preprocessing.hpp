/****************************************************************************/
// Module:      preprocessing.hpp
// Description: preprocessing algorithm
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      June 19, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX - 
/****************************************************************************/

#ifndef __PREPROCESSING_HPP
#define __PREPROCESSING_HPP
#pragma once

#include <Eigen/Core>
#include <utility>
#include <vector>
#include <unordered_map>

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

namespace radar_odometry {

/// Crop the frame with max/min ranges
std::vector<Eigen::Vector3d> Preprocess(const std::vector<Eigen::Vector3d> &frame,
                                        double max_range,
                                        double min_range);

std::vector<SRadarPoint> Preprocess(const std::vector<SRadarPoint> &frame,
                                        double max_range,
                                        double min_range);

/// Voxelize point cloud keeping the original coordinates
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
                                             double voxel_size);


}

#endif  // __PREPROCESSING_HPP