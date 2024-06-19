#include "radar_odometry.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <chrono>

#include "algorithm/adaptive_threshold.hpp"
#include "algorithm/ego_motion_compensation.hpp"
#include "algorithm/registration.hpp"

namespace radar_odometry::pipeline {

bool RadarOdometry::RegisterPoints(const pcl::PointCloud<PointXYZPRVAE>::Ptr i_radar_points, const double i_radar_timestamp_sec)
{
    // 0. Delta time calculation
    double d_delta_radar_time_sec;

    if(poses_.size() == 0){
        d_delta_radar_time_sec = 0.0;
    }
    else{
        d_delta_radar_time_sec = i_radar_timestamp_sec - d_last_radar_time_sec_;
    }
    d_last_radar_time_sec_ = i_radar_timestamp_sec;

    std::cout<<"d_delta_radar_time_sec: "<<d_delta_radar_time_sec<<std::endl;

    Eigen::Matrix4f new_pose = Eigen::Matrix4f::Identity();
    

    // Eigen::Matrix4f prediction = GetPredictionModel(); // if poses < 2, return identity
    Eigen::Matrix4f prediction = GetPredictionModel(i_radar_timestamp_sec); // if poses < 2, return identity
    Eigen::Matrix4f last_pose = !poses_.empty() ? poses_.back() : Eigen::Matrix4f::Identity();

    Eigen::Matrix4f initial_guess = last_pose * prediction;

    if(config_.odometry_type == OdometryType::EGOMOTION){

        std::cout<<"Origin Point Num: "<<i_radar_points->points.size()<<std::endl;
        
        // 1. Points Preprocessing based on CV model
        pcl::PointCloud<PointXYZPRVAE>::Ptr vel_filtered_radar_ptr(new pcl::PointCloud<PointXYZPRVAE>);
        Eigen::Matrix4f predicted_vel;// = !poses_.empty() ? prediction / (i_radar_timestamp_sec - times_.back()) : Eigen::Matrix4f::Identity(); 

        if(poses_.size() < 2){ // Prediction is not completed
            predicted_vel = Eigen::Matrix4f::Identity();
            *vel_filtered_radar_ptr = *i_radar_points;
        }
        else{
            predicted_vel = prediction / (i_radar_timestamp_sec - times_.back());
            vel_filtered_radar_ptr = VelFiltering(i_radar_points, predicted_vel, 5);
        }

        std::cout<<"Vel Filtered Point Num: "<<vel_filtered_radar_ptr->points.size()<<std::endl;

        // 2. RANSAC Outlier Removal
        pcl::PointCloud<PointXYZPRVAE>::Ptr ransac_radar_ptr(new pcl::PointCloud<PointXYZPRVAE>);
        ransac_radar_ptr = RansacFit(vel_filtered_radar_ptr, 1, 100);

        std::cout<<"Ransac Filtered Point Num: "<<ransac_radar_ptr->points.size()<<std::endl;

        // 3. LSQ Fitting
        Eigen::Vector2f est_vel = FitSine(ransac_radar_ptr);

        double d_est_azim = atan2(est_vel[1], est_vel[0]);
        double d_est_vel = sqrt(est_vel[0]*est_vel[0] + est_vel[1]*est_vel[1]);

        // Radar pose DR with Radar
        double radar_dx = d_delta_radar_time_sec * d_est_vel * cos(d_est_azim + config_.ego_to_radar_yaw_deg * M_PI/180.0);
        double radar_dy = d_delta_radar_time_sec * d_est_vel * sin(d_est_azim + config_.ego_to_radar_yaw_deg * M_PI/180.0);
        double radar_dyaw = d_delta_radar_time_sec * d_est_vel * sin(d_est_azim + config_.ego_to_radar_yaw_deg * M_PI/180.0) / config_.ego_to_radar_x_m;

        Eigen::Matrix4f delta_pose = Eigen::Matrix4f::Identity();

        delta_pose(0, 0) = std::cos(radar_dyaw);
        delta_pose(0, 1) = -std::sin(radar_dyaw);
        delta_pose(1, 0) = std::sin(radar_dyaw);
        delta_pose(1, 1) = std::cos(radar_dyaw);

        // Translation part
        delta_pose(0, 3) = radar_dx;
        delta_pose(1, 3) = radar_dy;

        new_pose = last_pose * delta_pose;

    }
    else if(config_.odometry_type == OdometryType::ICP){
        // 4. Registration
        const double sigma = GetAdaptiveThreshold(); // Keep estimated model error

        // new_pose = ???

        Eigen::Matrix4f model_deviation = initial_guess.inverse() * new_pose;
        adaptive_threshold_.UpdateModelDeviation(model_deviation);
    }

    // End
    poses_.push_back(new_pose);
    times_.push_back(i_radar_timestamp_sec);
    

    return true;
}

// Regard dt is same all time
Eigen::Matrix4f RadarOdometry::GetPredictionModel() const {
    Eigen::Matrix4f pred = Eigen::Matrix4f::Identity();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}

// Consider various dt
Eigen::Matrix4f RadarOdometry::GetPredictionModel(double cur_timestamp) const {
    Eigen::Matrix4f pred = Eigen::Matrix4f::Identity();
    const size_t N = poses_.size();
    if (N < 2) return pred;

    double last_dt = times_[N - 1] - times_[N - 2];
    double cur_dt = cur_timestamp - times_[N - 1];

    Eigen::Matrix4f delta_pose = poses_[N - 2].inverse() * poses_[N - 1];
    
    Eigen::Matrix4f velocity = delta_pose / last_dt;
    Eigen::Matrix4f displacement = velocity * cur_dt;

    return displacement;
}

double RadarOdometry::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_threshold;
    }
    return adaptive_threshold_.ComputeThreshold();
}


bool RadarOdometry::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).block<3,1>(0,3).norm();
    return motion > 5.0 * config_.min_motion_th;
}

}