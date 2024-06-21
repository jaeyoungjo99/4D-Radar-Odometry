#include "algorithm/ego_motion_compensation.hpp"

// Variables and functions only used in this cpp
namespace{

}

namespace radar_odometry {

std::vector<RadarPoint> VelFiltering(const std::vector<RadarPoint> cloud, 
                                                const Eigen::Matrix4d &predicted_vel,
                                                float margin)
{
    std::vector<RadarPoint> inliers;
    std::vector<int> current_inliers;

    Eigen::Vector3d linear_vel = predicted_vel.block<3,1>(0, 3);
    Eigen::Matrix3d angular_vel = predicted_vel.block<3,3>(0, 0);

    double radar_v_x = linear_vel(0);
    double radar_v_y = linear_vel(1);
    double radar_v_z = linear_vel(2);

    std::cout<<"Predicted radar_v_x: "<<radar_v_x <<" radar_v_y: "<<radar_v_y<<std::endl;


    for (size_t i = 0; i < cloud.size(); ++i) {

        float point_azim_rad = cloud[i].azi_angle * M_PI / 180.0f;
        float point_vel = - cloud[i].vel;
        // double est_radial_vel = radar_v_x * cos(point_azim_rad) + radar_v_y * sin(point_azim_rad)
        //                         + radar_v_x * sin(point_azim_rad) - radar_v_y * cos(point_azim_rad);
        double est_radial_vel = radar_v_x * cos(point_azim_rad);

        // float predicted_y = coeffs[0] * cos(x) + coeffs[1] * sin(x);

        if (std::abs(est_radial_vel - point_vel) <= margin) {
            current_inliers.push_back(i);
        }
    }

    for (int idx : current_inliers) {
        inliers.push_back(cloud[idx]);
    }

    return inliers;
}

Eigen::Vector2d FitSine(const std::vector<RadarPoint> cloud)
{
    Eigen::MatrixXd A(cloud.size(), 2);
    Eigen::VectorXd b(cloud.size());

    for (size_t i = 0; i < cloud.size(); ++i) {
        float x = cloud[i].azi_angle * M_PI/180.0f;
        float y = - cloud[i].vel;

        A(i, 0) = cos(x);
        A(i, 1) = sin(x);
        b(i) = y;
    }

    // Solve the normal equation A^T * A * coeffs = A^T * b
    Eigen::Vector2d coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    
    return coeffs;  // [vx, vy]
}

Eigen::Vector2d FitSine(const std::vector<RadarPoint> cloud, const std::vector<int>& indices)
{
    Eigen::MatrixXd A(indices.size(), 2);
    Eigen::VectorXd b(indices.size());

    for (size_t i = 0; i < indices.size(); ++i) {
        float x = cloud[indices[i]].azi_angle * M_PI / 180.0f;
        float y = -cloud[indices[i]].vel;

        A(i, 0) = cos(x);
        A(i, 1) = sin(x);
        b(i) = y;
    }

    // Solve the normal equation A^T * A * coeffs = A^T * b
    Eigen::Vector2d coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    return coeffs;  // [vx, vy]
}

std::vector<RadarPoint> RansacFit(const std::vector<RadarPoint> cloud, float margin, int max_iterations)
{
    std::vector<RadarPoint> inliers;
    std::vector<int> best_inliers;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, cloud.size() - 1);

    int best_inliers_count = 0;
    Eigen::Vector2d best_coeffs;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Randomly select 2 points
        std::vector<int> sample_indices = {dis(gen), dis(gen)};

        // Fit model to the sample points
        Eigen::Vector2d coeffs = FitSine(cloud, sample_indices);

        // Count inliers
        std::vector<int> current_inliers;
        for (size_t i = 0; i < cloud.size(); ++i) {
            float x = cloud[i].azi_angle * M_PI / 180.0f;
            float y = - cloud[i].vel;
            float predicted_y = coeffs[0] * cos(x) + coeffs[1] * sin(x);

            if (std::abs(predicted_y - y) <= margin) {
                current_inliers.push_back(i);
            }
        }

        // Update the best model if the current one is better
        if (current_inliers.size() > best_inliers_count) {
            best_inliers_count = current_inliers.size();
            best_coeffs = coeffs;
            best_inliers = current_inliers;
        }
    }
    
    
    for (int idx : best_inliers) {
        inliers.push_back(cloud[idx]);
    }

    return inliers;
}

}