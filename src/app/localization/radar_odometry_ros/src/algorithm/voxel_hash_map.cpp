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

        // 최근접 탐색
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

    // 최대 탐색 거리 제한
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

VoxelHashMap::RadarPointVectorTuple VoxelHashMap::GetCorrespondencesCov(
    const RadarPointVector &points, double max_correspondance_distance, int max_cov_point) const {

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

        // 비교 함수: a가 b보다 멀리 있으면 true를 반환
        auto compare = [&](const std::pair<double, RadarPoint> &a, const std::pair<double, RadarPoint> &b) {
            return a.first > b.first;
        };

        // 우선순위 큐 선언
        std::priority_queue<std::pair<double, RadarPoint>, std::vector<std::pair<double, RadarPoint>>, decltype(compare)> pq(compare);

        // 모든 이웃 포인트에 대해
        for (const auto &neighbor : neighboors) {
            double distance = (neighbor.pose - point.pose).norm();

            if(distance < max_correspondance_distance){
                if (pq.size() < max_cov_point) {
                    pq.push(std::make_pair(distance, neighbor));
                } else if (distance < pq.top().first) {
                    pq.pop();
                    pq.push(std::make_pair(distance, neighbor));
                }
            }
        }

        // 우선순위 큐에서 최근접 포인트들을 추출
        RadarPointVector closest_neighboors;
        while (!pq.empty()) {
            closest_neighboors.push_back(pq.top().second);
            pq.pop();
        }

        RadarPoint closest_neighbor = GetCov(closest_neighboors);

        return closest_neighbor;
    };

    // 최대 탐색 거리 제한
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

std::vector<RadarPoint> VoxelHashMap::StaticPointcloud() const {
    std::vector<RadarPoint> points;
    points.reserve(max_points_per_voxel_ * map_.size()); // 복셀내 최대 포인트 수 * 전체 복셀수  할당
    for (const auto &[voxel, voxel_block] : map_) { // 각 복셀 순회
        (void)voxel;
        for (const auto &point : voxel_block.points) { // 복셀 내 포인트 순회
            if(point.range < 30){
                points.push_back(point);
            }
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
                       RadarPoint transformed_point = point; // 속성 복사
                       transformed_point.sensor_pose = pose; // 센서 위치 저장
                       transformed_point.pose = rotation * point.pose + translation; // 위치만 변경
                       return transformed_point;
                   });

    Update(points_transformed, translation);
}


void VoxelHashMap::AddPoints(const RadarPointVector &points) {
    if(points.size() == 0)
        return;

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