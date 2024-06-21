#include "algorithm/voxel_hash_map.hpp"

// Variables and functions only used in this cpp
namespace{
struct ResultTuple {
    ResultTuple(std::size_t n) {
        source.reserve(n);
        target.reserve(n);
    }
    std::vector<RadarPoint> source;
    std::vector<RadarPoint> target;
};
}

namespace radar_odometry {


VoxelHashMap::RadarPointVectorTuple VoxelHashMap::GetCorrespondences(
    const RadarPointVector &points, double max_correspondance_distance) const {
    auto GetClosestNeighboor = [&](const RadarPoint &point) {
        auto kx = static_cast<int>(point.pose[0] / voxel_size_);
        auto ky = static_cast<int>(point.pose[1] / voxel_size_);
        auto kz = static_cast<int>(point.pose[2] / voxel_size_);
        std::vector<Voxel> voxels;
        voxels.reserve(27);
        for (int i = kx - 1; i < kx + 1 + 1; ++i) {
            for (int j = ky - 1; j < ky + 1 + 1; ++j) {
                for (int k = kz - 1; k < kz + 1 + 1; ++k) {
                    voxels.emplace_back(i, j, k);
                }
            }
        }

        using RadarPointVector = std::vector<RadarPoint>;
        RadarPointVector neighboors;
        neighboors.reserve(27 * max_points_per_voxel_);
        for (const auto &voxel : voxels) {
            auto search = map_.find(voxel);
            if (search != map_.end()) {
                const auto &points = search->second.points;
                if (!points.empty()) {
                    for (const auto &point : points) {
                        neighboors.emplace_back(point);
                    }
                }
            }
        }

        RadarPoint closest_neighbor;
        double closest_distance2 = std::numeric_limits<double>::max();
        for (const auto &neighbor : neighboors) {
            double distance = (neighbor.pose - point.pose).squaredNorm();
            if (distance < closest_distance2) {
                closest_neighbor = neighbor;
                closest_distance2 = distance;
            }
        }

        return closest_neighbor;
    };

    RadarPointVector source, target;
    for (const auto &point : points) {
        RadarPoint closest_neighboors = GetClosestNeighboor(point);
        if ((closest_neighboors.pose - point.pose).norm() < max_correspondance_distance) {
            source.emplace_back(point);
            target.emplace_back(closest_neighboors);
        }
    }

    return std::make_tuple(source, target);
}

std::vector<RadarPoint> VoxelHashMap::Pointcloud() const {
    std::vector<RadarPoint> points;
    points.reserve(max_points_per_voxel_ * map_.size()); // 복셀내 최대 포인트 수 * 전체 복셀수  할당
    for (const auto &[voxel, voxel_block] : map_) { // 각 복셀 순회
        (void)voxel;
        for (const auto &point : voxel_block.points) { // 복셀 내 포인트 순회
            points.push_back(point);
        }
    }
    return points;
}

// Point 단위 Update
void VoxelHashMap::Update(const RadarPointVector &points, const Eigen::Vector3d &origin) {
    AddPoints(points);
    RemovePointsFarFromLocation(origin);
    RemovePointsFarfromTime(points.back().timestamp);
}

void VoxelHashMap::Update(const RadarPointVector &points, const Eigen::Matrix4d &pose) {
    RadarPointVector points_transformed(points.size());
    Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
    Eigen::Vector3d translation = pose.block<3, 1>(0, 3);
    
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const RadarPoint &point) {
                       RadarPoint transformed_point = point;
                       transformed_point.pose = rotation * point.pose + translation;
                       return transformed_point;
                   });

    Update(points_transformed, translation);
}


void VoxelHashMap::AddPoints(const RadarPointVector &points) {
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) { // point = RadarPoint
        // auto voxel = (point.pose / voxel_size_).cast<int>();
        auto voxel = Voxel((point.pose / voxel_size_).template cast<int>());
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto &voxel_block = search->second;
            voxel_block.AddPoint(point);
        } else {
            map_.insert({voxel, VoxelBlock{{point}, max_points_per_voxel_}});
        }
    });
}

void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    for (auto it = map_.begin(); it != map_.end();) {
        const auto &pt = it->second.points.front().pose; // second = VoxelBlock. 해당 voxel의 첫 포인트 탐색
        const auto max_distance2 = max_distance_ * max_distance_;
        if ((pt - origin).squaredNorm() > (max_distance2)) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
}

void VoxelHashMap::RemovePointsFarfromTime(const double cur_timestamp) {
    for (auto it = map_.begin(); it != map_.end();) {
        const auto &pt = it->second.points.front().timestamp; // second = VoxelBlock. 해당 voxel의 첫 포인트 탐색
        if (fabs(pt - cur_timestamp) > local_map_time_th_) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
}

}