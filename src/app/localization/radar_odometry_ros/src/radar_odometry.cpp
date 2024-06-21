#include "radar_odometry.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <chrono>

#include "algorithm/adaptive_threshold.hpp"
#include "algorithm/ego_motion_compensation.hpp"
#include "algorithm/preprocessing.hpp"
#include "algorithm/voxel_hash_map.hpp"
#include "algorithm/registration.hpp"

namespace radar_odometry::pipeline {

RadarOdometry::RadarPointVectorTuple RadarOdometry::RegisterPoints(const std::vector<RadarPoint> i_radar_points, const double i_radar_timestamp_sec)
{
    RadarPointVector ransac_radar_points;
    RadarPointVector vel_filtered_radar_points;

    const auto &cropped_frame = Preprocess(i_radar_points, config_.max_range, config_.min_range);

    // 0. Delta time calculation    
    double d_delta_radar_time_sec = !times_.empty() ? i_radar_timestamp_sec - times_.back() : 0.0;
    
    Eigen::Matrix4d new_pose = Eigen::Matrix4d::Identity();
    

    // Eigen::Matrix4d prediction = GetPredictionModel(); // if poses < 2, return identity
    Eigen::Matrix4d prediction = GetPredictionModel(i_radar_timestamp_sec); // if poses < 2, return identity
    Eigen::Matrix4d last_pose = !poses_.empty() ? poses_.back() : Eigen::Matrix4d::Identity();

    Eigen::Matrix4d initial_guess = last_pose * prediction; // CV 모델 예측 결과

    std::cout<<"Prediction Norm: "<< prediction.block<3,1>(0,3).norm() <<std::endl;

    if(config_.odometry_type == OdometryType::EGOMOTION || config_.odometry_type == OdometryType::EGOMOTIONICP){
        std::chrono::system_clock::time_point ego_motion_estimation_start_time_sec = std::chrono::system_clock::now();

        std::cout<<"Origin Point Num: "<<cropped_frame.size()<<std::endl;

        // 1. Points Preprocessing based on CV model

        Eigen::Matrix4d predicted_vel;

        if(poses_.size() < 2){ // Prediction is not completed
            predicted_vel = Eigen::Matrix4d::Identity();
            vel_filtered_radar_points = cropped_frame;
        }
        else{
            predicted_vel = prediction / (i_radar_timestamp_sec - times_.back());
            vel_filtered_radar_points = VelFiltering(cropped_frame, predicted_vel, config_.doppler_vel_margin);

            if(vel_filtered_radar_points.size() < 30){
                std::cout<<"Vel Filtering Fail! use origin"<<std::endl;
                vel_filtered_radar_points = cropped_frame;
            }
        }

        std::cout<<"Vel Filtered Point Num: "<<vel_filtered_radar_points.size()<<std::endl;

        // 2. RANSAC Outlier Removal
        
        ransac_radar_points = RansacFit(vel_filtered_radar_points, 1, 100); // margin, max_iter

        std::cout<<"Ransac Filtered Point Num: "<<ransac_radar_points.size()<<std::endl;
        

        // 3. LSQ Fitting
        Eigen::Vector2d est_coeff;
        double d_est_azim, d_est_vel;
        double radar_dx, radar_dy, radar_dyaw;
        if(ransac_radar_points.size() > 10){
            est_coeff = FitSine(ransac_radar_points);
            d_est_azim = atan2(est_coeff[1], est_coeff[0]);
            d_est_vel = sqrt(est_coeff[0]*est_coeff[0] + est_coeff[1]*est_coeff[1]);

            // Radar pose DR with Radar
            radar_dx = d_delta_radar_time_sec * d_est_vel * cos(d_est_azim + config_.ego_to_radar_yaw_deg * M_PI/180.0);
            radar_dy = d_delta_radar_time_sec * d_est_vel * sin(d_est_azim + config_.ego_to_radar_yaw_deg * M_PI/180.0);
            radar_dyaw = d_delta_radar_time_sec * d_est_vel * sin(d_est_azim + config_.ego_to_radar_yaw_deg * M_PI/180.0) / config_.ego_to_radar_x_m;

        }
        else{
            Eigen::Vector3d linear = prediction.block<3,1>(0, 3);
            Eigen::Matrix3d angular = prediction.block<3,3>(0, 0);

            radar_dx = linear(0);
            radar_dy = linear(1);
            radar_dyaw = std::atan2(angular(1, 0), angular(0, 0));
        }


        Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();

        delta_pose(0, 0) = std::cos(radar_dyaw);
        delta_pose(0, 1) = -std::sin(radar_dyaw);
        delta_pose(1, 0) = std::sin(radar_dyaw);
        delta_pose(1, 1) = std::cos(radar_dyaw);

        // Translation part
        delta_pose(0, 3) = radar_dx;
        delta_pose(1, 3) = radar_dy;

        new_pose = last_pose * delta_pose;
        
        std::cout<<"Est radar_dx: "<<radar_dx <<" dy: "<<radar_dy<< " dyaw deg: "<<radar_dyaw*180/M_PI<<std::endl;

        std::chrono::duration<double> ego_motion_estimation_time_sec = std::chrono::system_clock::now() - ego_motion_estimation_start_time_sec;
        std::cout<<"Ego motion Time sec: " << ego_motion_estimation_time_sec.count() << std::endl;

        if(!poses_.empty() && config_.odometry_type == OdometryType::EGOMOTIONICP){
            std::chrono::system_clock::time_point registration_start_time_sec = std::chrono::system_clock::now();

            // const double sigma = GetAdaptiveThreshold(); // Keep estimated model error
            double sigma = GetAdaptiveThreshold(); // Keep estimated model error

            ROS_WARN_STREAM("Sigma: "<<sigma);
            
            std::cout<<"Delta pose Norm: "<< delta_pose.block<3,1>(0,3).norm() <<std::endl;


            if(sigma > 1.0) sigma = 1.0;

            // new_pose = RegisterScan2Scan3DoF2(ransac_radar_points, last_radar_ptr, 
            //                                 last_pose * delta_pose, last_pose, 
            //                                 3.0 * sigma, sigma / 3.0);    

            new_pose = RegisterScan2Map(cropped_frame, 
                                        local_map_,
                                        last_pose * delta_pose, 
                                        3.0 * sigma, sigma / 3.0); 

            std::chrono::duration<double>registration_time_sec = std::chrono::system_clock::now() - registration_start_time_sec;
            std::cout<<"Registration Time sec: " << registration_time_sec.count() << std::endl;
        }
        last_radar_ptr = ransac_radar_points;

    }
    else if(config_.odometry_type == OdometryType::ICP){
        // 4. Registration
        const double sigma = GetAdaptiveThreshold(); // Keep estimated model error

    }

    // End
    poses_.push_back(new_pose);
    local_map_.Update(cropped_frame, new_pose);
    times_.push_back(i_radar_timestamp_sec);

    Eigen::Matrix4d model_deviation = initial_guess.inverse() * new_pose;
    adaptive_threshold_.UpdateModelDeviation(model_deviation);

    return {ransac_radar_points, ransac_radar_points};
}

// Regard dt is same all time
Eigen::Matrix4d RadarOdometry::GetPredictionModel() const {
    Eigen::Matrix4d pred = Eigen::Matrix4d::Identity();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}

// Consider various dt
Eigen::Matrix4d RadarOdometry::GetPredictionModel(double cur_timestamp) const {
    Eigen::Matrix4d pred = Eigen::Matrix4d::Identity();
    const size_t N = poses_.size();
    if (N < 2) return pred;

    double last_dt = times_[N - 1] - times_[N - 2];
    double cur_dt = cur_timestamp - times_[N - 1];

    Eigen::Matrix4d delta_pose = poses_[N - 2].inverse() * poses_[N - 1];
    
    Eigen::Matrix4d velocity = delta_pose / last_dt;
    Eigen::Matrix4d displacement = velocity * cur_dt;

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