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
    std::vector<int> vec_inliers;

    Eigen::Vector3d linear_vel = predicted_vel.block<3,1>(0, 3);
    Eigen::Matrix3d angular_vel = predicted_vel.block<3,3>(0, 0);

    double est_radar_v_x = linear_vel(0);
    double est_radar_v_y = linear_vel(1);
    double est_radar_v_z = linear_vel(2);

    Eigen::Vector2d est_radar_vel_vec(est_radar_v_x, est_radar_v_y);

    std::cout<<"Predicted est_radar_v_x: "<<est_radar_v_x <<" est_radar_v_y: "<<est_radar_v_y<<std::endl;

    for (size_t i = 0; i < cloud.size(); ++i) {

        float point_azim_rad = cloud[i].azi_angle * M_PI / 180.0f;
        float point_vel = cloud[i].vel; // 전진시 대체로 음수
        
        Eigen::Vector2d point_azim_rad_vec(cos(point_azim_rad), sin(point_azim_rad));
        double est_point_vel = - point_azim_rad_vec.dot(est_radar_vel_vec);

        if (std::abs(est_point_vel - point_vel) <= margin) {
            vec_inliers.push_back(i);
        }
    }

    for (int idx : vec_inliers) {
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


Eigen::Matrix4d CalculateVelocity(const Eigen::Matrix4d& transform, double delta_t_sec) {
    // 회전 행렬 R
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    // 평행 이동 벡터 t
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
    // 선형 속도 v (t 변화를 시간으로 나눔)
    Eigen::Vector3d linear_vel = translation / delta_t_sec;
    // 각속도 행렬 omega 계산 (logarithm map 사용)
    // Eigen::Matrix3d omega = rotation.log() / delta_t_sec;
    Eigen::Matrix3d omega = (rotation - Eigen::Matrix3d::Identity()) / delta_t_sec;

    Eigen::AngleAxisd angle_axis(rotation);
    Eigen::Vector3d angular_vel = angle_axis.angle() * angle_axis.axis() / delta_t_sec;
    double yaw_vel = angular_vel.z();
    std::cout<<"YAW VEL DEG: "<<yaw_vel*180/M_PI<<std::endl;

    // 속도 행렬 V를 생성
    Eigen::Matrix4d velocity = Eigen::Matrix4d::Zero();
    velocity.block<3, 3>(0, 0) = omega;
    velocity.block<3, 1>(0, 3) = linear_vel;

    return velocity;
}

bool CheckMotionEstimation(const Eigen::Matrix4d & cv_prediction, const Eigen::Matrix4d & lsq_prediction, double delta_radar_time_sec)
{

    Eigen::Vector3d lsq_linear = lsq_prediction.block<3,1>(0, 3);
    Eigen::Matrix3d lsq_angular = lsq_prediction.block<3,3>(0, 0);

    double lsq_v_x = lsq_linear(0) / delta_radar_time_sec;
    double lsq_v_y = lsq_linear(1) / delta_radar_time_sec;


    double lsq_dyaw = std::atan2(lsq_angular(1, 0), lsq_angular(0, 0));

    if( fabs(lsq_dyaw / delta_radar_time_sec) > 20.0 * M_PI/180.0 ||
       fabs(lsq_v_x) > 50 || fabs(lsq_v_y) > 50){
        return false;
    } 

    return true;
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

}