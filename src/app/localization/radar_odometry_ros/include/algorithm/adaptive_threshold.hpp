/****************************************************************************/
// Module:      adaptive_threshold.hpp
// Description: adaptive_threshold algorithm
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      June 19, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX - 
/****************************************************************************/

#ifndef __ADAPTIVE_THRESHOLD_HPP__
#define __ADAPTIVE_THRESHOLD_HPP__
#pragma once

#include <Eigen/Core>
#include <utility>
#include <vector>
#include <deque>
#include <numeric>

#include "types/point_type.hpp"

namespace radar_odometry {

struct AdaptiveThreshold {
    explicit AdaptiveThreshold(double initial_threshold, double min_motion_th, double max_range)
        : initial_threshold_(initial_threshold),
          min_motion_th_(min_motion_th),
          max_range_(max_range) {}

    /// Update the current belief of the deviation from the prediction model
    inline void UpdateModelDeviation(const Eigen::Matrix4d &current_deviation) {
        model_deviation_ = current_deviation;
    }

    /// Returns the KISS-ICP adaptive threshold used in registration
    double ComputeThreshold();

    // configurable parameters
    double initial_threshold_;
    double min_motion_th_;
    double max_range_;

    // Local cache for ccomputation
    double model_error_sse2_ = 0;
    int num_samples_ = 0;
   
    std::deque<double> deque_model_error_sse2_;

    Eigen::Matrix4d model_deviation_ = Eigen::Matrix4d::Identity();
};
}

#endif  // __ADAPTIVE_THRESHOLD_HPP__