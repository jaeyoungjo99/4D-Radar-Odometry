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
#include <chrono>

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
#include "algorithm/preprocessing.hpp"
#include "algorithm/voxel_hash_map.hpp"
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
    IcpType icp_type = IcpType::P2P;

    // map params
    double voxel_size = 1.0;
    double max_range = 100.0;
    double min_range = 5.0;
    int max_points_per_voxel = 20;

    double local_map_time_th = 1.0;

    double min_motion_th = 0.1;
    double initial_trans_threshold = 2.0; // p2p search distance
    double initial_vel_threshold = 30.0; // vel search distance

    double doppler_vel_margin = 3.0;

    bool icp_3dof = true;
    bool icp_doppler = true;
    double doppler_gm_th = 0.5;
    double doppler_trans_lambda = 0.5;
    int icp_min_point_num = 10;

    double lm_lambda = 0.0;

    double range_variance_m = 0.1;
    double azimuth_variance_deg = 2.0;
    double elevation_variance_deg = 5.0;

    int gicp_max_point = 20;
};

class RadarOdometry{
public:
    using RadarPointVector = std::vector<RadarPoint>;
    using RadarPointVectorTuple = std::tuple<std::vector<RadarPoint>, std::vector<RadarPoint>>;
public:
    explicit RadarOdometry(const RadarOdometryConfig &config)
        : config_(config),
        local_map_(config.voxel_size, config.max_range, config.max_points_per_voxel, config.local_map_time_th),
        adaptive_threshold_(config.initial_trans_threshold, config.initial_vel_threshold,
            config.min_motion_th, config.max_range),
        registration_(config.icp_type, config.icp_3dof, config.lm_lambda,  config.icp_doppler,
                    config.doppler_gm_th, config.doppler_trans_lambda, config.range_variance_m,
                    config.azimuth_variance_deg, config.elevation_variance_deg, config.gicp_max_point,
                    config.ego_to_radar_x_m)
        {}

    RadarOdometry() : RadarOdometry(RadarOdometryConfig{}) {}

public:
    RadarPointVectorTuple RegisterPoints(const std::vector<RadarPoint> i_radar_points, const double i_radar_timestamp_sec);
    Eigen::Matrix4d GetPredictionModel() const;
    Eigen::Matrix4d GetPredictionModel(double cur_timestamp) const;
    
    double GetTransAdaptiveThreshold();
    double GetVelAdaptiveThreshold();
    bool HasMoved();

public:
    std::vector<RadarPoint> LocalMap() const { return local_map_.Pointcloud(); };
    std::vector<Eigen::Matrix4d> poses() const {return poses_;};

private:
    std::vector<Eigen::Matrix4d> poses_;
    std::vector<double> times_;
    RadarOdometryConfig config_;
    VoxelHashMap local_map_;
    AdaptiveThreshold adaptive_threshold_;
    Registration registration_;

    double d_last_radar_time_sec_;
    std::vector<RadarPoint> last_radar_ptr;
};

}

#endif  // __RADAR_ODOMETRY_HPP__