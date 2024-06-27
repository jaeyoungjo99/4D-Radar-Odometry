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

std::vector<RadarPoint> VelFiltering(const std::vector<RadarPoint> cloud, 
                                                const Eigen::Matrix4d &predicted_vel,
                                                float margin);

std::vector<RadarPoint> VelFiltering(const std::vector<RadarPoint> cloud, 
                                                const Velocity &predicted_vel,
                                                float margin);

// Return coeff of y = c_0 * cos(x) + c_1 * sin(x)
Eigen::Vector2d FitSine(const std::vector<RadarPoint> cloud);

// Return coeff of y = c_0 * cos(x) + c_1 * sin(x)
Eigen::Vector2d FitSine(const std::vector<RadarPoint> cloud, const std::vector<int>& indices);

// Return Ransac outlier removed pointcloud
std::vector<RadarPoint> RansacFit(const std::vector<RadarPoint> cloud, float margin, int max_iterations);



bool CheckMotionEstimation(const Eigen::Matrix4d & cv_prediction, const Eigen::Matrix4d & lsq_prediction, double delta_radar_time_sec);
bool CheckVelValidation(const Eigen::Matrix4d & est_motion);
bool CheckVelValidation(const Velocity & est_motion);

}

#endif  // __EGO_MOTION_COMPENSATION_HPP__