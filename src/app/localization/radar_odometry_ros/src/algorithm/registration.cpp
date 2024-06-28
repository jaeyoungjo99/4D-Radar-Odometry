#include "algorithm/registration.hpp"


namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

using Matrix3_3d = Eigen::Matrix<double, 3, 3>;
using Matrix2_3d = Eigen::Matrix<double, 2, 3>;
using Matrix1_3d = Eigen::Matrix<double, 1, 3>;
using Matrix1_4d = Eigen::Matrix<double, 1, 4>;
using Matrix1_6d = Eigen::Matrix<double, 1, 6>;
using Matrix4_6d = Eigen::Matrix<double, 4, 6>;

// using Matrix3d = Eigen::Matrix<double, 3, 1>;
}  // namespace Eigen


// Variables and functions only used in this cpp
namespace{

inline double square(double x) { return x * x; }

constexpr int MAX_NUM_ITERATIONS_ = 50;
constexpr double ESTIMATION_THRESHOLD_ = 0.01;

void TransformPoints(const Eigen::Matrix4d &T, std::vector<RadarPoint> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) {
                       Eigen::Vector4d p(point.pose.x(), point.pose.y(), point.pose.z(), 1.0);
                       Eigen::Vector4d p_transformed = T * p;
                       RadarPoint transformed_point = point;
                       transformed_point.pose = p_transformed.head<3>();
                       return transformed_point;
                   });
}

void TransformPoints(const Eigen::Matrix4d &transform, const std::vector<RadarPoint> i_points, std::vector<RadarPoint>& o_points) {
    o_points.clear();
    o_points = i_points;
    
    // 변환 행렬의 상위 3x3 부분을 회전 행렬로 사용
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);

    // 변환 행렬의 상위 3x1 부분을 평행 이동 벡터로 사용
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);

    for (size_t i = 0; i < i_points.size(); ++i) {
        // 입력 포인트를 Eigen::Vector4d 동차 좌표로 변환
        Eigen::Vector4d input_point(i_points[i].pose.x(), i_points[i].pose.y(), i_points[i].pose.z(), 1.0);

        // 변환 행렬을 사용하여 출력 포인트를 계산
        Eigen::Vector4d output_point = transform * input_point;

        // 변환된 좌표를 출력 포인트로 저장
        o_points[i].pose.x() = output_point.x();
        o_points[i].pose.y() = output_point.y();
        o_points[i].pose.z() = output_point.z();
    }
}

Eigen::Matrix<double, 6, 1> se3_log(const Eigen::Matrix4d &transform) {
    Eigen::Matrix<double, 6, 1> se3_log;
    Eigen::Matrix3d rotation = transform.block<3,3>(0,0);
    Eigen::Vector3d translation = transform.block<3,1>(0,3);
    Eigen::Vector3d eulerAngles = rotation.eulerAngles(2, 1, 0); // ZYX order: yaw, pitch, roll

    // 리 대수 요소에 저장
    se3_log.head<3>() = translation;
    se3_log.tail<3>() = eulerAngles;

    return se3_log;
}

Eigen::Matrix3d vectorToSkewSymmetricMatrix(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d skew_symmetric;
    skew_symmetric << 0, -vec.z(), vec.y(),
                      vec.z(), 0, -vec.x(),
                      -vec.y(), vec.x(), 0;
    return skew_symmetric;
}


Eigen::Matrix4d AlignClouds(const std::vector<RadarPoint> &source,
                            const std::vector<RadarPoint> &target,
                            double th) {
    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero();

    for (size_t i = 0; i < source.size(); ++i) {
        const Eigen::Vector3d residual = source[i].pose - target[i].pose;
        Eigen::Matrix3_6d J_r;

        // [ I(3x3), -(T p_k)^ ]
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * vectorToSkewSymmetricMatrix(source[i].pose);

        double weight = square(th) / square(th + residual.squaredNorm());
        JTJ.noalias() += J_r.transpose() * weight * J_r;
        JTr.noalias() += J_r.transpose() * weight * residual;
    }

    const Eigen::Vector6d x = JTJ.ldlt().solve(-JTr);

    Eigen::Vector3d rotation_vector = x.tail<3>(); // rpy
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())).toRotationMatrix(); // rotation
    transformation.block<3, 1>(0, 3) = x.head<3>(); // transform xyz

    return transformation;
}

Eigen::Matrix4d AlignCloudsDoppler(const std::vector<RadarPoint> &source,
                            const std::vector<RadarPoint> &target,
                            const Velocity &sensor_velocity,
                            double th,
                            double doppler_gm_th,
                            double doppler_trans_lambda) {
    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero();

    Eigen::Matrix6d JvTJv = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JvTrv = Eigen::Vector6d::Zero();

    Eigen::Matrix6d JTJ_tot = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JTr_tot = Eigen::Vector6d::Zero();

    Eigen::Vector3d est_sensor_vel_vehicle_coord = sensor_velocity.linear;
    // Eigen::Vector3d est_sensor_vel_vehicle_coord(sensor_velocity.linear.x(), sensor_velocity.linear.y(), 0.0);
    Eigen::Vector3d ego_to_sensor_translation(3.5, 0.0, 0.5);

    double vel_diff_sum = 0.0;

    for (size_t i = 0; i < source.size(); ++i) {
        const Eigen::Vector3d residual = target[i].pose - source[i].pose;
        Eigen::Matrix3_6d J_r;

        // 1. Translation
        // [ I(3x3), -(T p_k)^ ]
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * vectorToSkewSymmetricMatrix(source[i].pose);

        double weight_t = square(th) / square(th + residual.squaredNorm());

        JTJ.noalias() += J_r.transpose() * weight_t * J_r;
        JTr.noalias() += J_r.transpose() * weight_t * residual;

        // 2. Velocity
        // V_D_k - V_est(T)
        const double p_azim_rad = source[i].azi_angle * M_PI / 180.0f; // 정면 0 반시계
        const double p_ele_rad = source[i].ele_angle * M_PI / 180.0f; // 정면 0 윗방향

        Eigen::Vector3d point_direction_vector(cos(p_ele_rad) * cos(p_azim_rad),
                                         cos(p_ele_rad) * sin(p_azim_rad), 
                                         sin(p_ele_rad));
        

        // Eigen::Vector3d point_direction_vector(cos(p_azim_rad),sin(p_azim_rad), 0.0);


        // 차량의 속도 벡터와 관측 각도를 통해 도플러 속도 예상. 전진시 음수
        double est_point_vel = - point_direction_vector.dot(est_sensor_vel_vehicle_coord);
        double vel_residual = source[i].vel - est_point_vel;

        vel_diff_sum += fabs(vel_residual);

        Eigen::Matrix1_6d J_v;

        // [ - d_k / dt , -d_k x t_s / dt ]
        J_v.block<1,3>(0,0) = - point_direction_vector.transpose() / 0.1;
        J_v.block<1,3>(0,3) = - (point_direction_vector.cross(ego_to_sensor_translation)).transpose() / 0.1;

        double weight_v = square(doppler_gm_th) / square(doppler_gm_th + square(vel_residual));

        JvTJv.noalias() += J_v.transpose() * weight_v * J_v; // 6x1 1x6
        JvTrv.noalias() += J_v.transpose() * weight_v * vel_residual; // 6x1 1x1


        Eigen::Matrix4_6d J_tot;
        Eigen::Matrix1_4d R_tot;
    
        J_tot.block<3,6>(0,0) = J_r * sqrt(weight_t * (1.0 - doppler_trans_lambda));
        J_tot.block<1,6>(2,0) = J_v * sqrt(weight_v * doppler_trans_lambda);

        R_tot.block<1,3>(0,0) << residual.x() * sqrt(weight_t * (1.0 - doppler_trans_lambda)), 
                                 residual.y() * sqrt(weight_t * (1.0 - doppler_trans_lambda)),
                                 residual.z() * sqrt(weight_t * (1.0 - doppler_trans_lambda));
        R_tot(0,3) = vel_residual * sqrt(weight_v * doppler_trans_lambda);

        JTJ_tot.noalias() += J_tot.transpose() * J_tot; // 6x4 4x6
        JTr_tot.noalias() += J_tot.transpose() * R_tot.transpose(); // 6x4 4x1
    }

    std::cout<<"Vel diff mean: "<<vel_diff_sum / source.size()<<std::endl;

    const Eigen::Vector6d x_t = JTJ.ldlt().solve(JTr);
    const Eigen::Vector6d x_v = JvTJv.ldlt().solve(JvTrv);
    const Eigen::Vector6d x_tot = JTJ_tot.ldlt().solve(JTr_tot);

    // std::cout<<"x_t"<<std::endl;
    // std::cout<<x_t.transpose()<<std::endl;

    // std::cout<<"x_v"<<std::endl;
    // std::cout<<x_v.transpose()<<std::endl;

    // std::cout<<"x_tot"<<std::endl;
    // std::cout<<x_tot.transpose()<<std::endl;


    Eigen::Vector3d rotation_vector = x_tot.tail<3>(); // rpy
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())).toRotationMatrix(); // rotation
    transformation.block<3, 1>(0, 3) = x_tot.head<3>(); // transform xyz

    return transformation;
}

Eigen::Matrix4d AlignClouds3DoF(const std::vector<RadarPoint> &source,
                                const std::vector<RadarPoint> &target,
                                double th) {
    Eigen::Matrix3d JTJ = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < source.size(); ++i) {
        const Eigen::Vector2d residual(source[i].pose.x() - target[i].pose.x(), source[i].pose.y() - target[i].pose.y());
        Eigen::Matrix2_3d J_r;

        if(fabs(target[i].pose.z() - source[i].pose.z()) > 1.0) continue;

        // [ I(2x2), -(T p_k)^ ]
        J_r.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
        J_r.block<2, 1>(0, 2) << -source[i].pose(1), source[i].pose(0);

        double weight = square(th) / square(th + residual.squaredNorm());

        JTJ.noalias() += J_r.transpose() * weight * J_r;
        JTr.noalias() += J_r.transpose() * weight * residual;
    }

    const Eigen::Vector3d x = JTJ.ldlt().solve(-JTr);

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<2, 2>(0, 0) = Eigen::Rotation2Dd(x(2)).toRotationMatrix();
    transformation(0, 3) = x(0);
    transformation(1, 3) = x(1);

    return transformation;
}

Eigen::Matrix4d AlignCloudsDoppler3DoF(const std::vector<RadarPoint> &source,
                            const std::vector<RadarPoint> &target,
                            const Velocity &sensor_velocity,
                            double th,
                           double doppler_gm_th,
                           double doppler_trans_lambda) {
    Eigen::Matrix3d JTJ = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr = Eigen::Vector3d::Zero();

    Eigen::Matrix3d JvTJv = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JvTrv = Eigen::Vector3d::Zero();

    Eigen::Matrix3d JTJ_tot = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr_tot = Eigen::Vector3d::Zero();

    Eigen::Vector2d est_sensor_vel_vehicle_coord(sensor_velocity.linear.x(), sensor_velocity.linear.y()); // TODO
    Eigen::Vector2d ego_to_sensor_translation(3.5, 0.0);

    double vel_diff_sum = 0.0;


    for (size_t i = 0; i < source.size(); ++i) {
        const Eigen::Vector2d residual(target[i].pose.x() - source[i].pose.x(), target[i].pose.y() - source[i].pose.y());
        Eigen::Matrix2_3d J_r;

        if(fabs(target[i].pose.z() - source[i].pose.z()) > 1.0) continue;
        
        // 1. Translation
        // [ I(3x3), -(T p_k)^ ]
        J_r.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
        J_r.block<2, 1>(0, 2) << -source[i].pose(1), source[i].pose(0);

        double weight_t = square(th) / square(th + residual.squaredNorm());

        JTJ.noalias() += J_r.transpose() * weight_t * J_r;
        JTr.noalias() += J_r.transpose() * weight_t * residual;

        // 2. Velocity
        // V_D_k - V_est(T)
        const double p_azim_rad = source[i].azi_angle * M_PI / 180.0f; // 정면 0 반시계
        const double p_ele_rad = source[i].ele_angle * M_PI / 180.0f; // 정면 0 윗방향

        Eigen::Vector2d point_direction_vector(cos(p_azim_rad), sin(p_azim_rad));



        // 차량의 속도 벡터와 관측 각도를 통해 도플러 속도 예상. 전진시 음수
        double est_point_vel = - point_direction_vector.dot(est_sensor_vel_vehicle_coord);
        double vel_residual = source[i].vel - est_point_vel;

        vel_diff_sum += fabs(vel_residual);

        Eigen::Matrix1_3d J_v;

        // [ - d_k / dt , -d_k x t_s / dt ]
        J_v.block<1,2>(0,0) = - point_direction_vector.transpose() / 0.1;
        J_v(0,2) = - (point_direction_vector.x() * ego_to_sensor_translation.y() - point_direction_vector.y() * ego_to_sensor_translation.x()) / 0.1;

        double weight_v = square(doppler_gm_th) / square(doppler_gm_th + square(vel_residual));

        JvTJv.noalias() += J_v.transpose() * weight_v * J_v; // 3x1 1x3
        JvTrv.noalias() += J_v.transpose() * weight_v * vel_residual; // 3x1 1x1

        //

        Eigen::Matrix3d J_tot;
        Eigen::Matrix1_3d R_tot;
    
        J_tot.block<2,3>(0,0) = J_r * sqrt(weight_t * (1.0 - doppler_trans_lambda));
        J_tot.block<1,3>(2,0) = J_v * sqrt(weight_v * doppler_trans_lambda);

        R_tot.block<1,2>(0,0) << residual.x() * sqrt(weight_t * (1.0 - doppler_trans_lambda)),
                                 residual.y() * sqrt(weight_t * (1.0 - doppler_trans_lambda));
        R_tot(0,2) = vel_residual * sqrt(weight_v * doppler_trans_lambda);

        JTJ_tot.noalias() += J_tot.transpose() * J_tot; // 3x3 3x3
        JTr_tot.noalias() += J_tot.transpose() * R_tot.transpose(); // 3x3 3x1

    }

    // std::cout<<"Vel diff mean: "<<vel_diff_sum / source.size()<<std::endl;

    Eigen::Matrix3d JTJ_tot_diag = JTJ_tot.diagonal().asDiagonal();

    const Eigen::Vector3d x_t = JTJ.ldlt().solve(JTr);
    const Eigen::Vector3d x_v = JvTJv.ldlt().solve(JvTrv);
    // const Eigen::Vector3d x_tot = JTJ_tot.ldlt().solve(JTr_tot);
    const Eigen::Vector3d x_tot = (JTJ_tot + 0.0 * JTJ_tot_diag).ldlt().solve(JTr_tot);

    // std::cout<<"x_t"<<std::endl;
    // std::cout<<x_t.transpose()<<std::endl;

    // std::cout<<"x_v"<<std::endl;
    // std::cout<<x_v.transpose()<<std::endl;

    // std::cout<<"x_tot"<<std::endl;
    // std::cout<<x_tot.transpose()<<std::endl;

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    transformation.block<2, 2>(0, 0) = Eigen::Rotation2Dd(x_tot(2)).toRotationMatrix();
    transformation(0, 3) = x_tot(0);
    transformation(1, 3) = x_tot(1);


    return transformation;
}


}

namespace radar_odometry {


Eigen::Matrix4d RegisterFrame(const std::vector<RadarPoint> &frame,
                              const VoxelHashMap &voxel_map,
                              const Eigen::Matrix4d &initial_guess,
                              double max_correspondence_distance,
                              double kernel) {
    if (voxel_map.Empty()) return initial_guess;

    std::vector<RadarPoint> source = frame;
    TransformPoints(initial_guess, source);

    Eigen::Matrix4d T_icp = Eigen::Matrix4d::Identity();
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        const auto &[src, tgt] = voxel_map.GetCorrespondences(source, max_correspondence_distance);
        Eigen::Matrix4d estimation = AlignClouds(src, tgt, kernel);
        TransformPoints(estimation, source);
        T_icp = estimation * T_icp;
        if ((estimation.block<3, 3>(0, 0).eulerAngles(0, 1, 2).norm() + estimation.block<3, 1>(0, 3).norm()) < ESTIMATION_THRESHOLD_) break;
    }
    return T_icp * initial_guess;
}

Eigen::Matrix4d RegisterFrameDoppler(const std::vector<RadarPoint> &frame,
                              const VoxelHashMap &voxel_map,
                              const Eigen::Matrix4d &initial_guess,
                              const Eigen::Matrix4d &last_pose,
                              double max_correspondence_distance,
                              double kernel,
                              double doppler_gm_th,
                              double doppler_trans_lambda) {

    doppler_gm_th = std::max(doppler_gm_th, DBL_MIN);
    doppler_trans_lambda = std::min(std::max(doppler_trans_lambda, 0.0), 1.0);

    if (voxel_map.Empty()) return initial_guess;

    std::vector<RadarPoint> source = frame;
    TransformPoints(initial_guess, source);

    Velocity iter_velocity;
    Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d last_icp_pose = initial_guess;

    Eigen::Matrix4d T_icp = Eigen::Matrix4d::Identity();
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        const auto &[src, tgt] = voxel_map.GetCorrespondences(source, max_correspondence_distance);

        delta_pose = last_pose.inverse() * last_icp_pose;
        iter_velocity = CalculateVelocity(delta_pose, 0.1);

        Eigen::Matrix4d estimation = AlignCloudsDoppler(src, tgt, iter_velocity, kernel, doppler_gm_th, doppler_trans_lambda);
        TransformPoints(estimation, source);
        T_icp = estimation * T_icp;

        last_icp_pose = T_icp * initial_guess;

        if ((estimation.block<3, 3>(0, 0).eulerAngles(0, 1, 2).norm() + estimation.block<3, 1>(0, 3).norm()) < ESTIMATION_THRESHOLD_) break;
    }
    return T_icp * initial_guess;
}

Eigen::Matrix4d RegisterFrame3DoF(const std::vector<RadarPoint> &frame,
                                  const VoxelHashMap &voxel_map,
                                  const Eigen::Matrix4d &initial_guess,
                                  double max_correspondence_distance,
                                  double kernel) {
    if (voxel_map.Empty()) return initial_guess;

    std::vector<RadarPoint> source = frame;
    TransformPoints(initial_guess, source);

    Eigen::Matrix4d T_icp = Eigen::Matrix4d::Identity();
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        const auto &[src, tgt] = voxel_map.GetCorrespondences(source, max_correspondence_distance);
        Eigen::Matrix4d estimation = AlignClouds3DoF(src, tgt, kernel);
        TransformPoints(estimation, source);
        T_icp = estimation * T_icp;

        double yaw = std::atan2(estimation(1, 0), estimation(0, 0));
        if ((std::abs(yaw) + estimation.block<2, 1>(0, 3).norm()) < ESTIMATION_THRESHOLD_) break;

    }
    return T_icp * initial_guess;
}

Eigen::Matrix4d RegisterFrameDoppler3DoF(const std::vector<RadarPoint> &frame,
                           const VoxelHashMap &voxel_map,
                           const Eigen::Matrix4d &initial_guess,
                           const Eigen::Matrix4d &last_pose,
                           double max_correspondence_distance,
                           double kernel,
                           double doppler_gm_th,
                           double doppler_trans_lambda) {

    
    doppler_gm_th = std::max(doppler_gm_th, DBL_MIN);
    doppler_trans_lambda = std::min(std::max(doppler_trans_lambda, 0.0), 1.0);

    if (voxel_map.Empty()) return initial_guess;

    std::vector<RadarPoint> source = frame;
    TransformPoints(initial_guess, source);

    Velocity iter_velocity;
    Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d last_icp_pose = initial_guess;

    Eigen::Matrix4d T_icp = Eigen::Matrix4d::Identity(); // Global
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        const auto &[src, tgt] = voxel_map.GetCorrespondences(source, max_correspondence_distance);

        delta_pose = last_pose.inverse() * last_icp_pose;
        iter_velocity = CalculateVelocity(delta_pose, 0.1);

        Eigen::Matrix4d estimation = AlignCloudsDoppler3DoF(src, tgt, iter_velocity, kernel, doppler_gm_th, doppler_trans_lambda);
        TransformPoints(estimation, source);
        T_icp = estimation * T_icp;

        last_icp_pose = T_icp * initial_guess;

        double yaw = std::atan2(estimation(1, 0), estimation(0, 0));
        if ((std::abs(yaw) + estimation.block<2, 1>(0, 3).norm()) < ESTIMATION_THRESHOLD_) break;

    }
    return T_icp * initial_guess;
}


}