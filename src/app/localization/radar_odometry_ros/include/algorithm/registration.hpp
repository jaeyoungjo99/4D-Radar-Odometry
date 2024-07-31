/****************************************************************************/
// Module:      registration.hpp
// Description: registration algorithm
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      June 19, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX - 
/****************************************************************************/

#ifndef __REGISTRATION_HPP__
#define __REGISTRATION_HPP__
#pragma once

#include <Eigen/Core>
#include <utility>
#include <vector>
#include <algorithm>

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
#include "algorithm/voxel_hash_map.hpp"

namespace radar_odometry {

typedef enum {
	P2P,
    P2PCOV,
    PCOV2PCOV,
    GICP,
    APDGICP
} IcpType;

struct Registration{
    explicit Registration(IcpType icp_type, bool icp_3dof, double lm_lambda, 
            bool icp_doppler, double doppler_gm_th, double doppler_trans_lambda,
            double range_variance_m, double azimuth_variance_deg, double elevation_variance_deg, double gicp_max_point,
            double ego_to_radar_x_m, double ego_to_radar_y_m, double ego_to_radar_z_m, 
            double ego_to_radar_roll_deg, double ego_to_radar_pitch_deg, double ego_to_radar_yaw_deg)
            :icp_type_(icp_type),  icp_3dof_(icp_3dof), lm_lambda_(lm_lambda),
            icp_doppler_(icp_doppler), doppler_gm_th_(doppler_gm_th), doppler_trans_lambda_(doppler_trans_lambda),
            range_variance_m_(range_variance_m), azimuth_variance_deg_(azimuth_variance_deg), 
            elevation_variance_deg_(elevation_variance_deg), gicp_max_point_(gicp_max_point),
            ego_to_radar_x_m_(ego_to_radar_x_m), ego_to_radar_y_m_(ego_to_radar_y_m), ego_to_radar_z_m_(ego_to_radar_z_m),
            ego_to_radar_roll_deg_(ego_to_radar_roll_deg), ego_to_radar_pitch_deg_(ego_to_radar_pitch_deg), ego_to_radar_yaw_deg_(ego_to_radar_yaw_deg) {}


    Eigen::Matrix4d AlignCloudsLocalDoppler3DoF(std::vector<SRadarPoint> &source,
                            const std::vector<SRadarPoint> &target,
                            Eigen::Matrix4d &last_icp_pose,
                            const Velocity &sensor_velocity,
                            double trans_th,
                            double vel_th);

    Eigen::Matrix4d AlignCloudsLocalDoppler6DoF(std::vector<SRadarPoint> &source,
                            const std::vector<SRadarPoint> &target,
                            Eigen::Matrix4d &last_icp_pose,
                            const Velocity &sensor_velocity,
                            double trans_th,
                            double vel_th);

    Eigen::Matrix4d AlignCloudsLocalDoppler6DoFAll(std::vector<int> source_indices,
                            std::vector<SRadarPoint> &source_all,
                            const std::vector<SRadarPoint> &target,
                            Eigen::Matrix4d &last_icp_pose,
                            const Velocity &sensor_velocity,
                            double trans_th,
                            double vel_th);

    Eigen::Matrix4d AlignCloudsLocalDoppler6DoFEgo(std::vector<SRadarPoint> &source,
                            const std::vector<SRadarPoint> &target,
                            Eigen::Matrix4d &last_icp_pose,
                            const Velocity &sensor_velocity,
                            double trans_th,
                            double vel_th);

    // Master
    Eigen::Matrix4d RunRegister(const std::vector<SRadarPoint> &frame,
                            std::vector<SRadarPoint> &frame_global,
                            const VoxelHashMap &voxel_map,
                            const Eigen::Matrix4d &initial_guess,
                            const Eigen::Matrix4d &last_pose,
                            const double dt,
                            double trans_sigma,
                            double vel_sigma);


    // Parameters
    IcpType icp_type_;
    bool icp_3dof_;
    double lm_lambda_;
    bool icp_doppler_;
    double doppler_gm_th_;
    double doppler_trans_lambda_;
    double range_variance_m_;
    double azimuth_variance_deg_;
    double elevation_variance_deg_;
    int gicp_max_point_;
    double ego_to_radar_x_m_;
    double ego_to_radar_y_m_;
    double ego_to_radar_z_m_;
    double ego_to_radar_roll_deg_;
    double ego_to_radar_pitch_deg_;
    double ego_to_radar_yaw_deg_;
};
}

#endif  // __REGISTRATION_HPP__