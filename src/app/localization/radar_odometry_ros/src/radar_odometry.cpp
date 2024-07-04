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
    bool b_is_fitting_failed = false;

    const auto &cropped_frame = Preprocess(i_radar_points, config_.max_range, config_.min_range);

    // 0. Delta time calculation    
    double d_delta_radar_time_sec = !times_.empty() ? i_radar_timestamp_sec - times_.back() : 0.0;

    std::cout<<"Time diff: "<<d_delta_radar_time_sec<<std::endl;
    
    Eigen::Matrix4d new_pose = Eigen::Matrix4d::Identity();

    // cv_prediction 는 지난 pose 기준 상대좌표
    Eigen::Matrix4d cv_prediction = GetPredictionModel(i_radar_timestamp_sec); // if poses < 2, return identity
    Eigen::Matrix4d last_pose = !poses_.empty() ? poses_.back() : Eigen::Matrix4d::Identity();
    Eigen::Matrix4d initial_guess = last_pose * cv_prediction; // CV 모델 예측 결과 global 좌표


    std::cout<<"CV  Prediction Norm: "<< cv_prediction.block<3,1>(0,3).norm() <<std::endl;

    if(config_.odometry_type == OdometryType::EGOMOTION || config_.odometry_type == OdometryType::EGOMOTIONICP){
        std::chrono::system_clock::time_point ego_motion_estimation_start_time_sec = std::chrono::system_clock::now();

        std::cout<<"Origin Point Num: "<<cropped_frame.size()<<std::endl;

        // 1. Points Preprocessing based on CV model
        if(poses_.size() < 2){ // Prediction is not completed
            vel_filtered_radar_points = cropped_frame;
        }
        else{
            
            Velocity predicted_vel = CalculateVelocity(cv_prediction, i_radar_timestamp_sec - times_.back());
            vel_filtered_radar_points = VelFiltering(cropped_frame, predicted_vel, config_.doppler_vel_margin);

            if(vel_filtered_radar_points.size() < config_.icp_min_point_num){
                ROS_WARN_STREAM("VEL FILTERING FAIL! " << vel_filtered_radar_points.size() << " points");

                // 정적 포인트가 많은데 prediction이 잘못되어 vel filtering이 실패하는지 확인 위해 ransac 실행
                // ransac 인라이어 비중이 높으면 vel fiting 성공으로 다시 간주
                vel_filtered_radar_points = RansacFit(cropped_frame, 1, 100); // margin, max_iter
                if( (float) vel_filtered_radar_points.size() / (float) cropped_frame.size() > 0.8 ){
                    // 80% 이상이 인라이어
                    ROS_INFO_STREAM("RANSAC INLIER Percent " << (float) vel_filtered_radar_points.size() / (float) cropped_frame.size() * 100.0 << " % ");
                }
                else{
                    ROS_WARN_STREAM("RANSAC INLIER Percent " << (float) vel_filtered_radar_points.size() / (float) cropped_frame.size() * 100.0 << " % ");
                    // b_is_fitting_failed = true;
                    vel_filtered_radar_points = cropped_frame;
                }

            }
            else if( fabs(i_radar_timestamp_sec - times_.back()) < 0.01){
                ROS_WARN_STREAM("TIME DIFF ERROR: "<< i_radar_timestamp_sec - times_.back() <<" SEC");
                b_is_fitting_failed = true;
                vel_filtered_radar_points = cropped_frame;
            }
        }

        std::cout<<"Vel Filtered Point Num: "<<vel_filtered_radar_points.size()<<std::endl;

        // 2. RANSAC Outlier Removal
        if(b_is_fitting_failed == false){
            ransac_radar_points = RansacFit(vel_filtered_radar_points, 1, 100); // margin, max_iter
            std::cout<<"Ransac Filtered Point Num: "<<ransac_radar_points.size()<<std::endl;
        }
        else{
            ransac_radar_points = vel_filtered_radar_points;
        }

        
        // 3. LSQ Fitting
        Eigen::Matrix4d lsq_prediction = Eigen::Matrix4d::Identity(); // 지난 pose 기준 상대좌표

        // Ego Motion Estimation
        if(ransac_radar_points.size() > config_.icp_min_point_num && b_is_fitting_failed == false){

            Velocity estimated_velocity = EgoMotionEstimation(ransac_radar_points, config_.ego_to_radar_x_m, config_.ego_to_radar_yaw_deg * M_PI/180.0);
            
            Eigen::Vector3d translation = estimated_velocity.linear * d_delta_radar_time_sec;
            Eigen::Matrix3d rotation;
            rotation = Eigen::AngleAxisd(estimated_velocity.angular.z()  * d_delta_radar_time_sec, Eigen::Vector3d::UnitZ());

            lsq_prediction.block<3, 3>(0, 0) = rotation;
            lsq_prediction.block<3, 1>(0, 3) = translation;

            // std::cout<<"LSQ radar_dx: "<<radar_dx <<" dy: "<<radar_dy<< " dyaw deg: "<<radar_dyaw*180/M_PI<<std::endl;

            if(CheckVelValidation( CalculateVelocity(lsq_prediction , d_delta_radar_time_sec)) == false){
                ROS_WARN_STREAM("LSQ Motion Deviate!!!! USE PREDICTION");
                lsq_prediction = cv_prediction;        
            }
        }
        else{
            ROS_WARN_STREAM("VEL FILTERING FAIL! USE PREDICTION");
            lsq_prediction = cv_prediction;
        }

        new_pose = last_pose * lsq_prediction;
        
        std::chrono::duration<double> ego_motion_estimation_time_sec = std::chrono::system_clock::now() - ego_motion_estimation_start_time_sec;
        std::cout<<"Ego motion Time sec: " << ego_motion_estimation_time_sec.count() << std::endl;

        if(!poses_.empty() && ransac_radar_points.size() > config_.icp_min_point_num && 
                config_.odometry_type == OdometryType::EGOMOTIONICP ){
            std::chrono::system_clock::time_point registration_start_time_sec = std::chrono::system_clock::now();

            double trans_sigma = GetTransAdaptiveThreshold(); // Keep estimated model error
            double vel_sigma = GetVelAdaptiveThreshold(); // Keep estimated model error

            ROS_WARN_STREAM("Trans Sigma: "<< trans_sigma);
            ROS_WARN_STREAM("Vel   Sigma: "<< vel_sigma);
            
            std::cout<<"LSQ Prediction Norm: "<< lsq_prediction.block<3,1>(0,3).norm() <<std::endl;

            // if(trans_sigma > 1.0) trans_sigma = 1.0;

            new_pose = registration_.RunRegister(ransac_radar_points, local_map_, last_pose * lsq_prediction, last_pose,
                                            d_delta_radar_time_sec,
                                            3.0 * trans_sigma, trans_sigma / 3.0);

            std::cout<<"ICP Prediction Norm: "<< (last_pose.inverse() * new_pose).block<3,1>(0,3).norm() <<std::endl;

            if(CheckVelValidation( CalculateVelocity(last_pose.inverse() * new_pose, d_delta_radar_time_sec)) == false){
                ROS_WARN_STREAM("ICP Motion Deviate!!!! USE LSQ PREDICTION");
                new_pose = last_pose * lsq_prediction;
            }

            std::chrono::duration<double>registration_time_sec = std::chrono::system_clock::now() - registration_start_time_sec;
            std::cout<<"Registration Time sec: " << registration_time_sec.count() << std::endl;
        }

        // End
        Eigen::Matrix4d model_deviation = (last_pose * lsq_prediction).inverse() * new_pose;
        adaptive_threshold_.UpdateModelDeviation(model_deviation);

        // Vel fitting 실패시, 동적 포인트가 맵에 누적되는것 방지
        if(b_is_fitting_failed == false)
            local_map_.Update(ransac_radar_points, new_pose);

        poses_.push_back(new_pose);
        times_.push_back(i_radar_timestamp_sec);

    }
    else if(config_.odometry_type == OdometryType::ICP){
        // // 4. Registration


        std::chrono::system_clock::time_point registration_start_time_sec = std::chrono::system_clock::now();

        double trans_sigma = GetTransAdaptiveThreshold(); // Keep estimated model error
        double vel_sigma = GetVelAdaptiveThreshold(); // Keep estimated model error

        ROS_WARN_STREAM("Trans Sigma: "<< trans_sigma);
        ROS_WARN_STREAM("Vel   Sigma: "<< vel_sigma);
        
        // if(trans_sigma > 1.0) trans_sigma = 1.0;

        new_pose = registration_.RunRegister(cropped_frame, local_map_, initial_guess, last_pose,
                                d_delta_radar_time_sec,
                                3.0 * trans_sigma, trans_sigma / 3.0);

        std::cout<<"ICP Prediction Norm: "<< (last_pose.inverse() * new_pose).block<3,1>(0,3).norm() <<std::endl;

        if(CheckVelValidation( CalculateVelocity(last_pose.inverse() * new_pose, d_delta_radar_time_sec)) == false){
            ROS_WARN_STREAM("ICP Motion Deviate!!!! USE LSQ PREDICTION");
            new_pose = initial_guess;
        }

        std::chrono::duration<double>registration_time_sec = std::chrono::system_clock::now() - registration_start_time_sec;
        std::cout<<"Registration Time sec: " << registration_time_sec.count() << std::endl;

        // End
        Eigen::Matrix4d model_deviation = initial_guess.inverse() * new_pose;
        adaptive_threshold_.UpdateModelDeviation(model_deviation);

        // Vel fitting 실패시, 동적 포인트가 맵에 누적되는것 방지
        if(b_is_fitting_failed == false)
            local_map_.Update(cropped_frame, new_pose);

        poses_.push_back(new_pose);
        times_.push_back(i_radar_timestamp_sec);
    }

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

    // Calculate the delta pose between T-2 and T-1
    Eigen::Matrix4d delta_pose = poses_[N - 2].inverse() * poses_[N - 1];

    // Extract linear and angular components from delta_pose
    Eigen::Vector3d delta_translation = delta_pose.block<3, 1>(0, 3);
    Eigen::Matrix3d delta_rotation = delta_pose.block<3, 3>(0, 0);

    // Calculate linear and angular velocities
    Eigen::Vector3d linear_velocity = delta_translation / last_dt;
    Eigen::AngleAxisd delta_angle_axis(delta_rotation);
    Eigen::Vector3d angular_velocity = delta_angle_axis.angle() * delta_angle_axis.axis() / last_dt;

    // Predict the translation component at the current timestamp
    Eigen::Vector3d predicted_translation = linear_velocity * cur_dt;

    // Predict the rotation component at the current timestamp
    Eigen::AngleAxisd predicted_angle_axis(angular_velocity.norm() * cur_dt, angular_velocity.normalized());
    Eigen::Matrix3d predicted_rotation = predicted_angle_axis.toRotationMatrix();

    // Form the predicted transform
    pred.block<3, 3>(0, 0) = predicted_rotation;
    pred.block<3, 1>(0, 3) = predicted_translation;

    return pred;
}

double RadarOdometry::GetTransAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_trans_threshold;
    }
    return adaptive_threshold_.ComputeTransThreshold();
}

double RadarOdometry::GetVelAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_vel_threshold;
    }
    return adaptive_threshold_.ComputeTransThreshold();
}


bool RadarOdometry::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).block<3,1>(0,3).norm();
    return motion > 5.0 * config_.min_motion_th;
}

}