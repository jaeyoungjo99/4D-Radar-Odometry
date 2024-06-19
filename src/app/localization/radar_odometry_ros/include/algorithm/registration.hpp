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

namespace radar_odometry {

Eigen::Matrix4f RegisterPoints(const pcl::PointCloud<PointXYZPRVAE>::Ptr i_radar_points,
                                const Eigen::Matrix4f &initial_guess,
                                double max_correspondence_distance,
                                double kernel);
}

#endif  // __REGISTRATION_HPP__