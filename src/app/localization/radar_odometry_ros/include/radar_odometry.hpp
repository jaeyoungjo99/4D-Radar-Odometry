/****************************************************************************/
// Module:      radar_odometry.hpp
// Description: radar_odometry main pipeline
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      June 19, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX - 
/****************************************************************************/

#ifndef __RADAR_ODOMETRY_HPP__
#define __RADAR_ODOMETRY_HPP__
#pragma once

#include <Eigen/Core>
#include <utility>
#include <vector>

// PCL
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include "types/point_type.hpp"

#include "algorithm/adaptive_threshold.hpp"
#include "algorithm/ego_motion_compensation.hpp"
#include "algorithm/registration.hpp"

namespace radar_odometry::pipeline {

typedef enum {
	EGOMOTION,
    ICP,
    EGOMOTIONICP
} OdometryType;

struct RadarOdometryConfig{
    double ego_to_radar_x_m = 1.0;
    double ego_to_radar_yaw_deg = 0.0;
    OdometryType odometry_type = OdometryType::EGOMOTION;

    // map params
    double voxel_size = 1.0;
    double max_range = 100.0;
    double min_range = 5.0;
    int max_points_per_voxel = 20;

    double min_motion_th = 0.1;
    double initial_threshold = 2.0; // p2p search distance

    double doppler_vel_margin = 3.0;
};

class RadarOdometry{
public:
    using PointXYZPRVAETuple = std::tuple<pcl::PointCloud<PointXYZPRVAE>, pcl::PointCloud<PointXYZPRVAE>>;
public:
    explicit RadarOdometry(const RadarOdometryConfig &config)
        : config_(config),
        adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range)
        {last_radar_ptr.reset(new pcl::PointCloud<PointXYZPRVAE>());}

    RadarOdometry() : RadarOdometry(RadarOdometryConfig{}) {}

public:
    // std::tuple<pcl::PointCloud<PointXYZPRVAE>::Ptr, Vector3dVector>
    PointXYZPRVAETuple RegisterPoints(const pcl::PointCloud<PointXYZPRVAE>::Ptr i_radar_points, const double i_radar_timestamp_sec);
    Eigen::Matrix4d GetPredictionModel() const;
    Eigen::Matrix4d GetPredictionModel(double cur_timestamp) const;
    
    double GetAdaptiveThreshold();
    bool HasMoved();

public:
    std::vector<Eigen::Matrix4d> poses() const {return poses_;};

private:
    std::vector<Eigen::Matrix4d> poses_;
    std::vector<double> times_;
    RadarOdometryConfig config_;
    AdaptiveThreshold adaptive_threshold_;

    double d_last_radar_time_sec_;
    pcl::PointCloud<PointXYZPRVAE>::Ptr last_radar_ptr;
};

}

#endif  // __RADAR_ODOMETRY_HPP__