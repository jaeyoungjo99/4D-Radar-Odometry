#include "algorithm/adaptive_threshold.hpp"

// Variables and functions only used in this cpp
namespace {
double ComputeModelError(const Eigen::Matrix4f &model_deviation, double max_range) {
    const float theta = Eigen::AngleAxisf(model_deviation.block<3,3>(0,0)).angle();
    const float delta_rot = 2.0 * max_range * std::sin(theta / 2.0);
    const float delta_trans = model_deviation.block<3,1>(0,3).norm();
    return double(delta_trans + delta_rot);
}
}  // namespace

namespace radar_odometry {

// if model_deviation < min_motion_th_, use default threshold
double AdaptiveThreshold::ComputeThreshold() {
    double model_error = ComputeModelError(model_deviation_, max_range_);
    if (model_error > min_motion_th_) {
        model_error_sse2_ += model_error * model_error;
        num_samples_++;
    }

    if (num_samples_ < 1) {
        return initial_threshold_;
    }
    return std::sqrt(model_error_sse2_ / num_samples_); // Keep averaging model error (m)
}

}