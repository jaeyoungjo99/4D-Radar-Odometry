#include "algorithm/ego_motion_compensation.hpp"

// Variables and functions only used in this cpp
namespace{

}

namespace radar_odometry {

std::vector<RadarPoint> VelFiltering(const std::vector<RadarPoint> cloud, 
                                    const Velocity &predicted_vel,
                                    float margin)
{
    std::vector<RadarPoint> inliers;
    std::vector<int> vec_inliers;

    Eigen::Vector3d linear_vel = predicted_vel.linear;
    Eigen::Vector3d angular_vel = predicted_vel.angular;

    Eigen::Vector3d ego_to_sensor_translation(3.5, 0.0, 0.5);

    for (size_t i = 0; i < cloud.size(); ++i){
        const double p_azim_rad = cloud[i].azi_angle * M_PI / 180.0f; // 정면 0 반시계
        const double p_ele_rad = cloud[i].ele_angle * M_PI / 180.0f; // 정면 0 윗방향

        Eigen::Vector3d point_direction_vector(cos(p_ele_rad) * cos(p_azim_rad),
                                         cos(p_ele_rad) * sin(p_azim_rad), 
                                         sin(p_ele_rad));
        
        double vel_predict = - point_direction_vector.dot(linear_vel + angular_vel.cross(ego_to_sensor_translation));

        if (std::abs(vel_predict - cloud[i].vel) <= margin) {
            vec_inliers.push_back(i);
        }
    }

    for (int idx : vec_inliers) {
        inliers.push_back(cloud[idx]);
    }

    return inliers;
}

// v = v_x * cos(azim) + v_y * sin(azim)
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
        std::vector<int> vec_inliers;
        for (size_t i = 0; i < cloud.size(); ++i) {
            float x = cloud[i].azi_angle * M_PI / 180.0f;
            float y = - cloud[i].vel;
            float predicted_y = coeffs[0] * cos(x) + coeffs[1] * sin(x);

            if (std::abs(predicted_y - y) <= margin) {
                vec_inliers.push_back(i);
            }
        }

        // Update the best model if the current one is better
        if (vec_inliers.size() > best_inliers_count) {
            best_inliers_count = vec_inliers.size();
            best_coeffs = coeffs;
            best_inliers = vec_inliers;
        }
    }
    
    
    for (int idx : best_inliers) {
        inliers.push_back(cloud[idx]);
    }

    return inliers;
}

Velocity EgoMotionEstimation(const std::vector<RadarPoint> cloud, const double ego_to_radar_x_m, const double ego_to_radar_yaw_rad)
{
    // v = v_x * cos(azim) + v_y * sin(azim)
    Eigen::Vector2d est_coeff = FitSine(cloud);
    double d_est_azim = atan2(est_coeff[1], est_coeff[0]);
    double d_est_vel = sqrt(est_coeff[0]*est_coeff[0] + est_coeff[1]*est_coeff[1]);
    
    Eigen::Vector2d est_azim_rad_vec(cos(d_est_azim), sin(d_est_azim));
    Eigen::Vector2d est_vel_vec( est_coeff[0],  est_coeff[1]);

    //
    // v = (cos(alpha + beta) = b/l sin(alpha + beta)) v_s
    double alpha = atan2(est_coeff[1], est_coeff[0]);
    double v_s = sqrt(est_coeff[0]*est_coeff[0] + est_coeff[1]*est_coeff[1]);
    double v_yaw = sin(alpha + ego_to_radar_yaw_rad) * d_est_vel / ego_to_radar_x_m;

    Velocity estimated_velocity;
    estimated_velocity.linear.x() = est_coeff[0];
    estimated_velocity.linear.y() = est_coeff[1];

    estimated_velocity.angular.z() = v_yaw;

    return estimated_velocity;
}


bool CheckVelValidation(const Eigen::Matrix4d & est_motion)
{

    Eigen::Vector3d linear_vel = est_motion.block<3,1>(0, 3);
    Eigen::Matrix3d omega = est_motion.block<3,3>(0, 0);

    double v_x = linear_vel(0);
    double v_y = linear_vel(1);


    Eigen::AngleAxisd angle_axis(omega);
    Eigen::Vector3d angular_vel = angle_axis.angle() * angle_axis.axis();
    // double yaw_vel = angular_vel.z();
    double yaw_vel = std::atan2(omega(1, 0), omega(0, 0));

    std::cout<<"ICP vx: "<<v_x<<" vy: "<<v_y<< " vyaw_vel deg: "<<yaw_vel * 180 / M_PI <<std::endl;

    if( fabs(yaw_vel) > 20.0 * M_PI/180.0 ||
       fabs(v_x) > 50 || fabs(v_y) > 50){
        return false;
    } 

    return true;
}

bool CheckVelValidation(const Velocity & est_motion)
{

    std::cout<<"ICP vx: "<<est_motion.linear.x()<<" vy: "<<est_motion.linear.y()<< " vyaw_vel deg: "<<est_motion.angular.z() * 180 / M_PI <<std::endl;

    if( fabs(est_motion.angular.z()) > 20.0 * M_PI/180.0 ||
       fabs(est_motion.linear.x()) > 50 || fabs(est_motion.linear.y()) > 50){
        return false;
    } 

    return true;
}


}