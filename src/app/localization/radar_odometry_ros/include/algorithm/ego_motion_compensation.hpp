/****************************************************************************/
// Module:      ego_motion_compensation.hpp
// Description: ego_motion_compensation algorithm
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      June 19, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX - 
/****************************************************************************/

#ifndef __EGO_MOTION_COMPENSATION_HPP__
#define __EGO_MOTION_COMPENSATION_HPP__
#pragma once

#include <Eigen/Core>
#include <utility>
#include <vector>
#include <random>

#include "types/point_type.hpp"

namespace radar_odometry {

pcl::PointCloud<PointXYZPRVAE>::Ptr VelFiltering(const pcl::PointCloud<PointXYZPRVAE>::Ptr& cloud, 
                                                const Eigen::Matrix4d &predicted_vel,
                                                float margin);

// Return coeff of y = c_0 * cos(x) + c_1 * sin(x)
Eigen::Vector2d FitSine(const pcl::PointCloud<PointXYZPRVAE>::Ptr& cloud);

// Return coeff of y = c_0 * cos(x) + c_1 * sin(x)
Eigen::Vector2d FitSine(const pcl::PointCloud<PointXYZPRVAE>::Ptr& cloud, const std::vector<int>& indices);

// Return Ransac outlier removed pointcloud
pcl::PointCloud<PointXYZPRVAE>::Ptr RansacFit(const pcl::PointCloud<PointXYZPRVAE>::Ptr& cloud, float margin, int max_iterations);

}

#endif  // __EGO_MOTION_COMPENSATION_HPP__