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
// #include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

#include "types/point_type.hpp"

namespace radar_odometry {

std::vector<SRadarPoint> VelFiltering(const std::vector<SRadarPoint> cloud, 
                                                const Velocity &predicted_vel,
                                                float margin);

// Return coeff of y = c_0 * cos(x) + c_1 * sin(x)
Eigen::Vector2d FitSine(const std::vector<SRadarPoint> cloud);

// Return coeff of y = c_0 * cos(x) + c_1 * sin(x)
Eigen::Vector2d FitSine(const std::vector<SRadarPoint> cloud, const std::vector<int>& indices);

// Return Ransac outlier removed pointcloud
std::vector<SRadarPoint> RansacFit(const std::vector<SRadarPoint> cloud, float margin, int max_iterations);

Velocity EgoMotionEstimation(const std::vector<SRadarPoint> cloud, const double ego_to_radar_x_m, const double ego_to_radar_yaw_rad);

bool CheckVelValidation(const Eigen::Matrix4d & est_motion);
bool CheckVelValidation(const Velocity & est_motion);

}

#endif  // __EGO_MOTION_COMPENSATION_HPP__