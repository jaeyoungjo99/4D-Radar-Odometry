/****************************************************************************/
// Module:      voxel_hash_map.hpp
// Description: voxel_hash_map algorithm
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      June 19, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX - 
/****************************************************************************/

#ifndef __VOXEL_HASH_MAP_HPP__
#define __VOXEL_HASH_MAP_HPP__
#pragma once

#include <Eigen/Core>
#include <utility>
#include <tuple>
#include <vector>
#include <unordered_map>
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

namespace radar_odometry {
struct VoxelHashMap{

    using RadarPointVector = std::vector<SRadarPoint>;
    using RadarPointVectorTuple = std::tuple<RadarPointVector, RadarPointVector>;
    using Voxel = Eigen::Vector3i;

    struct VoxelBlock {
        // buffer of points with a max limit of n_points
        std::vector<SRadarPoint> points;
        int num_points_;
        inline void AddPoint(const SRadarPoint &point) {
            if (points.size() < static_cast<size_t>(num_points_)) points.push_back(point);
        }
    };
    struct VoxelHash {
        size_t operator()(const Voxel &voxel) const {
            const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
            return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
        }
    };

    explicit VoxelHashMap(double voxel_size, double max_distance, int max_points_per_voxel, int local_map_time_th)
        : voxel_size_(voxel_size),
          max_distance_(max_distance),
          max_points_per_voxel_(max_points_per_voxel),
          local_map_time_th_(local_map_time_th) {}


    RadarPointVectorTuple GetCorrespondences(const RadarPointVector &points,
                                             double max_correspondance_distance) const;

    std::tuple<std::vector<int>, std::vector<SRadarPoint>, std::vector<SRadarPoint>> 
                            GetCorrespondencesWithIdx(const RadarPointVector &points,
                                             double max_correspondance_distance) const;

    RadarPointVectorTuple GetCorrespondencesCov(const RadarPointVector &points,
                                             double max_correspondance_distance,
                                             int max_cov_point) const;

    inline void Clear() { map_.clear(); }
    inline bool Empty() const { return map_.empty(); }
    void Update(const RadarPointVector &points, const Eigen::Vector3d &origin);
    void Update(const RadarPointVector &points, const Eigen::Matrix4d &pose);
    void UpdateGlobal(const RadarPointVector &points, const Eigen::Matrix4d &pose);
    void AddPoints(const RadarPointVector &points);
    void RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    void RemovePointsFarfromTime(const double cur_timestamp);
    std::vector<SRadarPoint> Pointcloud() const;
    std::vector<SRadarPoint> StaticPointcloud() const;


    double voxel_size_;
    double max_distance_;
    int max_points_per_voxel_;
    double local_map_time_th_;
    std::unordered_map<Voxel, VoxelBlock, VoxelHash> map_; 

};
}

#endif  // __VOXEL_HASH_MAP_HPP__