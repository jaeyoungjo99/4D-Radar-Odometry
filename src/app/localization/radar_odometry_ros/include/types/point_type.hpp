#ifndef __POINTTYPE_HPP__
#define __POINTTYPE_HPP__
#pragma once

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <regex>
#include <string>
#include <vector>

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

/* Types */
struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    float    intensity;
    uint16_t ring;
    float    time;      // point time after scan start
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // make sure our new allocators are aligned
} EIGEN_ALIGN16;    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZIRT,
                    ( float, x, x )
                    ( float, y, y )
                    ( float, z, z )
                    ( float, intensity, intensity )
                    ( uint16_t, ring, ring )
				    ( float, time, time )
)

/* Types */
struct PointXYZPRVAE
{
    PCL_ADD_POINT4D;
    float       power;
    float       range;
    float       vel;
    float       azi_angle;
    float       ele_angle;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // make sure our new allocators are aligned
} EIGEN_ALIGN16;    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZPRVAE,
                    ( float, x, x )
                    ( float, y, y )
                    ( float, z, z )
                    ( float, power, power )
                    ( float, range, range )
				    ( float, vel, vel )
                    ( float, azi_angle, azi_angle )
                    ( float, ele_angle, ele_angle )
)

struct VodRadarPointType {
    float x, y, z;       // 위치
    float RCS;           // 레이다 반사율
    float v_r;           // 속도
    float v_r_compensated; // 보상된 속도
    float time;          // 측정 시간
};

struct RadarPoint
{
    PCL_ADD_POINT4D;      // position in [m]
    float power;         // CFAR cell to side noise ratio in [dB]
    float vel;  // Doppler velocity in [m/s]
    float range;          // range in [m]
    float RCS;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

struct SRadarPoint {
    Eigen::Vector3d pose;
    Eigen::Matrix4d cov;

    Eigen::Vector3d local;
    Eigen::Matrix4d sensor_pose;


    double power; // TODO: Power residual? (07/15).
    double range; // m
    double vel; // mps
    double azi_angle; // deg Right to Left
    double ele_angle; // deg. Down to Up

    int frame_idx;
    double timestamp;

    bool is_static;

    // 생성자
    SRadarPoint()
        : pose(Eigen::Vector3d::Zero()), cov(Eigen::Matrix4d::Identity()),
          local(Eigen::Vector3d::Zero()),
          sensor_pose(Eigen::Matrix4d::Identity()), power(0.0),
          range(0.0), vel(0.0), azi_angle(0.0), ele_angle(0.0),
          frame_idx(0), timestamp(0.0), is_static(false){}

    ~SRadarPoint(){
    };

    // reset 함수
    void reset() {
        pose.setZero();
        cov.setZero();
        local.setZero();
        sensor_pose.setZero();
        power = 0.0;
        range = 0.0;
        vel = 0.0;
        azi_angle = 0.0;
        ele_angle = 0.0;
        frame_idx = 0;
        timestamp = 0.0;
        is_static = false;
    }
};

struct Velocity {
    Eigen::Vector3d linear;  // 선형 속도
    Eigen::Vector3d angular; // 각속도

    double time_diff_sec;

    // 기본 생성자Q - 멤버 변수를 0으로 초기화
    Velocity()
        : linear(Eigen::Vector3d::Zero()), angular(Eigen::Vector3d::Zero()), time_diff_sec(0.1) {}

    // 매개변수를 받아서 초기화하는 생성자
    Velocity(const Eigen::Vector3d& lin, const Eigen::Vector3d& ang)
        : linear(lin), angular(ang) {}
};

inline Velocity CalculateVelocity(const Eigen::Matrix4d& transform, double delta_t_sec) {
    // 회전 행렬 R
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    // 평행 이동 벡터 t
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
    
    // 선형 속도 v (t 변화를 시간으로 나눔)
    Eigen::Vector3d linear_vel = translation / delta_t_sec;
    
    // 각속도 행렬 omega 계산 (logarithm map 사용)
    Eigen::AngleAxisd angle_axis(rotation);
    Eigen::Vector3d angular_vel = angle_axis.angle() * angle_axis.axis() / delta_t_sec;
    
    // Velocity 구조체 생성 및 반환
    Velocity velocity;
    velocity.linear = linear_vel;
    velocity.angular = angular_vel;
    velocity.time_diff_sec = delta_t_sec;
    
    return velocity;
}

inline Eigen::Matrix4d CalculateTransform(const Velocity& velocity, double delta_t_sec) {
    // 선형 변환 계산
    Eigen::Vector3d translation = velocity.linear * delta_t_sec;

    // 각속도에서 회전 변환 계산
    double angle = velocity.angular.norm() * delta_t_sec;
    Eigen::Vector3d axis = velocity.angular.normalized();
    Eigen::AngleAxisd rotation(angle, axis);
    Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();

    // 변환 행렬 생성
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation_matrix;
    transform.block<3, 1>(0, 3) = translation;

    return transform;
}

inline SRadarPoint GetCov(const std::vector<SRadarPoint> &points){

    SRadarPoint cov_point;

    int num_points = points.size();
    
    if(num_points == 0){
        return cov_point;
    }

    // 행: 4(x,y,z,0), 열: 포인트 개수  
    Eigen::Matrix<double, 4, -1> neighbors(4, num_points); 
    for (int j = 0; j < num_points; j++) {
      neighbors.col(j) = points[j].pose.homogeneous();
    }

    // 행 평균
    Eigen::Vector4d mean = neighbors.rowwise().mean();
    neighbors.colwise() -= mean.eval();
    cov_point.cov = neighbors * neighbors.transpose() / num_points;
    cov_point.pose = mean.head<3>();

    return cov_point;
}

inline SRadarPoint CalPointCov(const SRadarPoint point, double range_var_m, double azim_var_deg, double ele_var_deg){
    SRadarPoint cov_point = point;
    double dist = cov_point.range;
    // double s_x = std::max(dist * range_var_m, 0.1);
    double s_x = range_var_m;
    double s_y = std::max(0.1, dist * sin(azim_var_deg / 180 * M_PI)); // 0.00873
    double s_z = std::max(0.1, dist * sin(ele_var_deg / 180 * M_PI)); // 0.01745
    double elevation = cov_point.ele_angle / 180 * M_PI;
    double azimuth = cov_point.azi_angle / 180 * M_PI;
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(elevation, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d R; // Rotation matrix
    R = yawAngle * pitchAngle;

    Eigen::Matrix3d S; // Scaling matix
    S << s_x, 0.0, 0.0,
         0.0, s_y, 0.0,
         0.0, 0.0, s_z;

    Eigen::Matrix3d cov = R * S;
    cov_point.cov = Eigen::Matrix4d::Zero();
    cov_point.cov.block<3, 3>(0, 0) = cov;

    return cov_point;
}

inline SRadarPoint CalPointCov2d(const SRadarPoint point, double range_var_m, double azim_var_deg){
    SRadarPoint cov_point = point;
    double dist = sqrt(cov_point.pose.x()*cov_point.pose.x() + cov_point.pose.y()*cov_point.pose.y());
    // double s_x = std::max(dist * range_var_m, 0.1);
    double s_x = range_var_m;
    double s_y = std::max(0.1, dist * sin(azim_var_deg / 180 * M_PI)); // 0.00873
    double azimuth = cov_point.azi_angle / 180 * M_PI;
    Eigen::Matrix2d R;
    R << cos(azimuth), sin(azimuth),
        -sin(azimuth), cos(azimuth);

    Eigen::Matrix2d S; // Scaling matix
    S << s_x, 0.0,
         0.0, s_y;

    Eigen::Matrix2d cov = R * S;
    cov_point.cov = Eigen::Matrix4d::Zero();
    cov_point.cov.block<2, 2>(0, 0) = cov;
    cov_point.cov(2, 2) = 1.0;

    return cov_point;
}

inline void CalFramePointCov(std::vector<SRadarPoint> &points, double range_var_m, double azim_var_deg, double ele_var_deg){
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) {
                        SRadarPoint cov_point = CalPointCov(point, range_var_m, azim_var_deg, ele_var_deg);
                        return cov_point;
                   });
}

inline void CalFramePointCov2d(std::vector<SRadarPoint> &points, double range_var_m, double azim_var_deg){
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) {
                        SRadarPoint cov_point = CalPointCov2d(point, range_var_m, azim_var_deg);
                        return cov_point;
                   });
}

inline std::string FixFrameId(const std::string &frame_id) {
    return std::regex_replace(frame_id, std::regex("^/"), "");
}

inline std::unique_ptr<sensor_msgs::PointCloud2> CreatePointCloud2Msg(const size_t n_points,
                                                         const std_msgs::Header &header,
                                                         bool timestamp = false) {
    auto cloud_msg = std::make_unique<sensor_msgs::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    cloud_msg->header = header;
    cloud_msg->header.frame_id = FixFrameId(cloud_msg->header.frame_id);
    cloud_msg->fields.clear();
    int offset = 0;
    offset = addPointField(*cloud_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "power", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "range", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "vel", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "azi_angle", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "ele_angle", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset += sizeOfPointField(sensor_msgs::PointField::FLOAT32);
    if (timestamp) {
        // assuming timestamp on a velodyne fashion for now (between 0.0 and 1.0)
        offset = addPointField(*cloud_msg, "time", 1, sensor_msgs::PointField::FLOAT64, offset);
        offset += sizeOfPointField(sensor_msgs::PointField::FLOAT64);
    }

    // Resize the point cloud accordingly
    cloud_msg->point_step = offset;
    cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
    cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);
    modifier.resize(n_points);
    return cloud_msg;
}

inline void FillPointCloud2XYZ(const std::vector<SRadarPoint> &points, sensor_msgs::PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");

    sensor_msgs::PointCloud2Iterator<float> msg_power(msg, "power");
    sensor_msgs::PointCloud2Iterator<float> msg_range(msg, "range");
    sensor_msgs::PointCloud2Iterator<float> msg_vel(msg, "vel");
    sensor_msgs::PointCloud2Iterator<float> msg_azi_angle(msg, "azi_angle");
    sensor_msgs::PointCloud2Iterator<float> msg_ele_angle(msg, "ele_angle");
    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z, 
            ++msg_power, ++msg_range, ++msg_vel, ++msg_azi_angle, ++msg_ele_angle) {
        const SRadarPoint &point = points[i];
        *msg_x = point.pose.x();
        *msg_y = point.pose.y();
        *msg_z = point.pose.z();

        *msg_power = point.power;
        *msg_range = point.range;
        *msg_vel = point.vel;
        *msg_azi_angle = point.azi_angle;
        *msg_ele_angle = point.ele_angle;
    }
}

inline void FillVodPointCloud2XYZ(const std::vector<RadarPoint> &points, sensor_msgs::PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");

    sensor_msgs::PointCloud2Iterator<float> msg_rcs(msg, "RCS");
    sensor_msgs::PointCloud2Iterator<float> msg_vel(msg, "vel");
    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z, 
            ++msg_rcs, ++msg_vel) {
        const RadarPoint &point = points[i];
        *msg_x = point.x;
        *msg_y = point.y;
        *msg_z = point.z;

        *msg_rcs = point.power;
        *msg_vel = point.vel;
    }
}

inline std::vector<SRadarPoint> AfiPointCloud2ToRadarPoints(const sensor_msgs::PointCloud2::ConstPtr msg) {
    std::vector<SRadarPoint> points;
    points.reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");

    sensor_msgs::PointCloud2ConstIterator<float> msg_power(*msg, "power");
    sensor_msgs::PointCloud2ConstIterator<float> msg_range(*msg, "range");
    sensor_msgs::PointCloud2ConstIterator<float> msg_vel(*msg, "vel");
    sensor_msgs::PointCloud2ConstIterator<float> msg_azi_angle(*msg, "azi_angle");
    sensor_msgs::PointCloud2ConstIterator<float> msg_ele_angle(*msg, "ele_angle");

    double time_stamp = msg->header.stamp.toSec();
    for (size_t i = 0; i < msg->height * msg->width; ++i, ++msg_x, ++msg_y, ++msg_z, 
            ++msg_power, ++msg_range, ++msg_vel, ++msg_azi_angle, ++msg_ele_angle) {

        SRadarPoint iter_point;

        iter_point.pose <<*msg_x, *msg_y, *msg_z; // Canbe Transformed
        iter_point.local <<*msg_x, *msg_y, *msg_z; // Fixed Local frame. DO NOT CHANGE AFTER IT TODO: Make it const
        iter_point.power = *msg_power;
        iter_point.range = *msg_range;
        iter_point.vel = *msg_vel;
        iter_point.azi_angle = *msg_azi_angle;
        iter_point.ele_angle = *msg_ele_angle;
        iter_point.timestamp = time_stamp;
        
        points.emplace_back(iter_point);
    }
    return points;
}

inline std::unique_ptr<sensor_msgs::PointCloud2> SRadarPointToPointCloud2(const std::vector<SRadarPoint> &points,
                                                       const std_msgs::Header &header) {
    auto msg = CreatePointCloud2Msg(points.size(), header);
    FillPointCloud2XYZ(points, *msg);
    return msg;
}

inline std::unique_ptr<sensor_msgs::PointCloud2> VodToPointCloud2(const std::vector<RadarPoint> &points,
                                                       const std_msgs::Header &header) {
    auto msg = CreatePointCloud2Msg(points.size(), header);
    FillVodPointCloud2XYZ(points, *msg);
    return msg;
}


#endif
