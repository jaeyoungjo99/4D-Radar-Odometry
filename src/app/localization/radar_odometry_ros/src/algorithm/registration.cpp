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
namespace {

inline double square(double x) { return x * x; }

constexpr int MAX_NUM_ITERATIONS_ = 50;
constexpr double ESTIMATION_THRESHOLD_ = 0.005;

void TransformPoints(const Eigen::Matrix4d &T, std::vector<RadarPoint> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) {
                       Eigen::Vector4d p(point.pose.x(), point.pose.y(), point.pose.z(), 1.0);
                       Eigen::Vector4d p_transformed = T * p;
                       RadarPoint transformed_point = point; // 모든 속성 복사
                       transformed_point.pose = p_transformed.head<3>();
                       return transformed_point;
                   });
}

void TransformPoints(const Eigen::Matrix4d &T, const std::vector<RadarPoint> &points, std::vector<RadarPoint> &o_points) {
    assert(points.size() == o_points.size()); // points와 o_points가 같은 크기를 가지는지 확인

    std::transform(points.cbegin(), points.cend(), o_points.begin(),
                   [&](const auto &point) {
                       Eigen::Vector4d p(point.pose.x(), point.pose.y(), point.pose.z(), 1.0);
                       Eigen::Vector4d p_transformed = T * p;
                       RadarPoint transformed_point = point; // 모든 속성 복사
                       transformed_point.pose = p_transformed.head<3>();
                       return transformed_point;
                   });
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


}

namespace radar_odometry {


Eigen::Matrix4d Registration::AlignCloudsLocal3DoF(std::vector<RadarPoint> &source,
                                const std::vector<RadarPoint> &target,
                                Eigen::Matrix4d &iter_pose,
                                double th) {

    Eigen::Matrix3d JTJ = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < source.size(); ++i) {
        const Eigen::Vector2d residual_global(target[i].pose.x() - source[i].pose.x(), target[i].pose.y() - source[i].pose.y());

        Eigen::Vector4d hom_point(target[i].pose.x(), target[i].pose.y(), target[i].pose.z(), 1.0);
        Eigen::Vector4d transformed_hom_point = iter_pose.inverse() * hom_point;
        
        const Eigen::Vector2d target_local = transformed_hom_point.head<2>();
        const Eigen::Vector2d residual_local = target_local - source[i].local.head<2>();

        Eigen::Matrix2_3d J_r;

        if(fabs(target[i].pose.z() - source[i].pose.z()) > 1.0) continue;

        double range_weight = 1.0;

        if(icp_type_ == IcpType::P2PCOV){
            
            /*  test_iter_pose
                [r00 r01 x
                 r10 r11 y
                 0   0   1]
            */

            Eigen::Matrix3d test_iter_pose = Eigen::Matrix3d::Identity();
            test_iter_pose.block<2,2>(0,0) = iter_pose.block<2,2>(0,0);

            Eigen::Matrix3d RCR;
            RCR = test_iter_pose.matrix() * source[i].cov.block<3,3>(0,0) * test_iter_pose.matrix().transpose(); // Only Source Point Cov
            RCR(2, 2) = 1.0;

            Eigen::Matrix3d mahalanobis = RCR.inverse(); // (CiB + T*CiA*T^(-1))^(−1)
            mahalanobis(2, 2) = 0.0f;

            Eigen::Vector3d error = Eigen::Vector3d::Zero();
            error.head<2>() = (target[i].pose.head<2>() - source[i].pose.head<2>());

            range_weight = (double)(error.transpose() * mahalanobis * error) / error.squaredNorm();

        }
        else if (icp_type_ == IcpType::PCOV2PCOV){
            Eigen::Matrix3d test_iter_pose = Eigen::Matrix3d::Identity();
            test_iter_pose.block<2,2>(0,0) = iter_pose.block<2,2>(0,0);

            Eigen::Matrix3d test_target_pose = Eigen::Matrix3d::Identity();
            test_target_pose.block<2,2>(0,0) = target[i].sensor_pose.block<2,2>(0,0);

            Eigen::Matrix3d RCR;
            RCR = test_target_pose.matrix() * target[i].cov.block<3,3>(0,0) * test_target_pose.matrix().transpose() + 
                  test_iter_pose.matrix()   * source[i].cov.block<3,3>(0,0) * test_iter_pose.matrix().transpose();
            RCR(2, 2) = 1.0;

            Eigen::Matrix3d mahalanobis = RCR.inverse(); // (CiB + T*CiA*T^(-1))^(−1)
            mahalanobis(2, 2) = 0.0f;

            Eigen::Vector3d error = Eigen::Vector3d::Zero();
            error.head<2>() = (target[i].pose.head<2>() - source[i].pose.head<2>());

            range_weight = (double)(error.transpose() * mahalanobis * error) / error.squaredNorm();

        }

        // [ I(2x2), -(T p_k)^ ]
        J_r.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
        J_r.block<2, 1>(0, 2) << -source[i].local(1), source[i].local(0); // 센서좌표계 SkewSymmetric

        // GM Kernel
        double weight = square(th) / square(th + residual_local.squaredNorm()) * range_weight;

        JTJ.noalias() += J_r.transpose() * weight * J_r;
        JTr.noalias() += J_r.transpose() * weight * residual_local;
    }

    // LM Optimization
    Eigen::Matrix3d JTJ_diag = JTJ.diagonal().asDiagonal();
    const Eigen::Vector3d x = (JTJ + lm_lambda_ * JTJ_diag).ldlt().solve(JTr);

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<2, 2>(0, 0) = Eigen::Rotation2Dd(x(2)).toRotationMatrix();
    transformation(0, 3) = x(0);
    transformation(1, 3) = x(1);

    // Source Point 의 센서 좌표 기준 미소 transformation
    return transformation;
}

Eigen::Matrix4d Registration::AlignCloudsLocalDoppler3DoF(std::vector<RadarPoint> &source,
                            const std::vector<RadarPoint> &target,
                            Eigen::Matrix4d &iter_pose,
                            const Velocity &sensor_velocity,
                            double trans_th, double vel_th) {

    Eigen::Matrix3d JTJ_tot = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr_tot = Eigen::Vector3d::Zero();

    // yaw 각도를 라디안으로 변환
    double ego_to_radar_yaw_rad = ego_to_radar_yaw_deg_ * M_PI / 180.0;

    // 회전 행렬 생성 (2D 회전)
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(ego_to_radar_yaw_rad), -sin(ego_to_radar_yaw_rad),
                       sin(ego_to_radar_yaw_rad),  cos(ego_to_radar_yaw_rad);


    Eigen::Vector2d est_sensor_vel_sensor_coord(sensor_velocity.linear.x(), sensor_velocity.linear.y()); // TODO add calib
    // Eigen::Vector2d est_sensor_vel_vehicle_coord = rotation_matrix * est_sensor_vel_sensor_coord;

    Eigen::Vector2d est_sensor_vel_vehicle_coord(sensor_velocity.linear.x(), sensor_velocity.linear.y()); // TODO add calib

    
    Eigen::Vector2d ego_to_sensor_translation(ego_to_radar_x_m_, 0.0);

    for (size_t i = 0; i < source.size(); ++i) {
        Eigen::Vector4d hom_point(target[i].pose.x(), target[i].pose.y(), target[i].pose.z(), 1.0);
        Eigen::Vector4d transformed_hom_point = iter_pose.inverse() * hom_point;
        
        const Eigen::Vector2d target_local = transformed_hom_point.head<2>();
        const Eigen::Vector2d residual_local = target_local - source[i].local.head<2>();

        Eigen::Matrix2_3d J_r;

        if(fabs(target[i].pose.z() - source[i].pose.z()) > 1.0) continue;

        double range_weight = 1.0;

        double target_static_weight = 1.0;
        
        if(target[i].is_static == false){
            target_static_weight = 0.01;
        }

        if(icp_type_ == IcpType::P2PCOV){
            
            /*  test_iter_pose
                [r00 r01 x
                 r10 r11 y
                 0   0   1]
            */

            Eigen::Matrix3d test_iter_pose = Eigen::Matrix3d::Identity();
            test_iter_pose.block<2,2>(0,0) = iter_pose.block<2,2>(0,0);

            Eigen::Matrix3d RCR;
            RCR = test_iter_pose.matrix() * source[i].cov.block<3,3>(0,0) * test_iter_pose.matrix().transpose(); // Only Source Point Cov
            RCR(2, 2) = 1.0;

            Eigen::Matrix3d mahalanobis = RCR.inverse(); // (CiB + T*CiA*T^(-1))^(−1)
            mahalanobis(2, 2) = 0.0f;

            Eigen::Vector3d error = Eigen::Vector3d::Zero();
            error.head<2>() = (target[i].pose.head<2>() - source[i].pose.head<2>());

            range_weight = (double)(error.transpose() * mahalanobis * error) / error.squaredNorm();

        }
        else if (icp_type_ == IcpType::PCOV2PCOV){
            Eigen::Matrix3d test_iter_pose = Eigen::Matrix3d::Identity();
            test_iter_pose.block<2,2>(0,0) = iter_pose.block<2,2>(0,0);

            Eigen::Matrix3d test_target_pose = Eigen::Matrix3d::Identity();
            test_target_pose.block<2,2>(0,0) = target[i].sensor_pose.block<2,2>(0,0);

            Eigen::Matrix3d RCR;
            RCR = test_target_pose.matrix() * target[i].cov.block<3,3>(0,0) * test_target_pose.matrix().transpose() + 
                  test_iter_pose.matrix()   * source[i].cov.block<3,3>(0,0) * test_iter_pose.matrix().transpose();
            RCR(2, 2) = 1.0;

            Eigen::Matrix3d mahalanobis = RCR.inverse(); // (CiB + T*CiA*T^(-1))^(−1)
            mahalanobis(2, 2) = 0.0f;

            Eigen::Vector3d error = Eigen::Vector3d::Zero(); // (dx, dy, 0)
            error.head<2>() = (target[i].pose.head<2>() - source[i].pose.head<2>());

            range_weight = (double)(error.transpose() * mahalanobis * error) / error.squaredNorm();

        }

        // 1. Translation
        // [ I(3x3), -(T p_k)^ ]
        J_r.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
        J_r.block<2, 1>(0, 2) << -source[i].local(1), source[i].local(0); // 센서좌표계 SkewSymmetric

        double weight_t = square(trans_th) / square(trans_th + residual_local.squaredNorm()) * range_weight * target_static_weight;

        // 2. Velocity
        // V_D_k - V_est(T)
        const double p_azim_rad = source[i].azi_angle * M_PI / 180.0f; // 정면 0 반시계
        const double p_ele_rad = source[i].ele_angle * M_PI / 180.0f; // 정면 0 윗방향

        Eigen::Vector2d point_direction_vector(cos(p_azim_rad), sin(p_azim_rad));
        Eigen::Vector2d point_direction_vector_ego = rotation_matrix * point_direction_vector;



        // 차량의 속도 벡터와 관측 각도를 통해 도플러 속도 예상. 전진시 음수
        double est_point_vel = - point_direction_vector.dot(est_sensor_vel_sensor_coord);
        double vel_residual = source[i].vel - est_point_vel;

        Eigen::Matrix1_3d J_v;

        // [ - d_k / dt , -d_k x t_s / dt ]
        J_v.block<1,2>(0,0) = - point_direction_vector.transpose() / sensor_velocity.time_diff_sec;
        J_v(0,2) = - (point_direction_vector_ego.x() * ego_to_sensor_translation.y() - 
                      point_direction_vector_ego.y() * ego_to_sensor_translation.x()) / sensor_velocity.time_diff_sec;



        double weight_v = square(vel_th) / square(vel_th + square(vel_residual)) * range_weight;

        // static 속성 부여
        if(fabs(vel_residual) < vel_th * 3.0 && residual_local.norm() < 0.6){
            source[i].is_static = true;
        }

        Eigen::Matrix3d J_tot;
        Eigen::Matrix1_3d R_tot;

        double sqrt_trans_weight = sqrt(weight_t * (1.0 - doppler_trans_lambda_));
        double sqrt_vel_weight = sqrt(weight_v * doppler_trans_lambda_);
    
        J_tot.block<2,3>(0,0) = J_r * sqrt_trans_weight;
        J_tot.block<1,3>(2,0) = J_v * sqrt_vel_weight;

        R_tot.block<1,2>(0,0) << residual_local.x() * sqrt_trans_weight,
                                 residual_local.y() * sqrt_trans_weight;
        R_tot(0,2) = vel_residual * sqrt_vel_weight;

        JTJ_tot.noalias() += J_tot.transpose() * J_tot; // 3x3 3x3
        JTr_tot.noalias() += J_tot.transpose() * R_tot.transpose(); // 3x3 3x1

    }

    // LM Optimization
    Eigen::Matrix3d JTJ_tot_diag = JTJ_tot.diagonal().asDiagonal();
    const Eigen::Vector3d x_tot = (JTJ_tot + lm_lambda_ * JTJ_tot_diag).ldlt().solve(JTr_tot);

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    transformation.block<2, 2>(0, 0) = Eigen::Rotation2Dd(x_tot(2)).toRotationMatrix();
    transformation(0, 3) = x_tot(0);
    transformation(1, 3) = x_tot(1);


    return transformation;
}

Eigen::Matrix4d Registration::AlignCloudsLocal6DoF(std::vector<RadarPoint> &source,
                                const std::vector<RadarPoint> &target,
                                Eigen::Matrix4d &iter_pose,
                                double th) {

    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero();

    for (size_t i = 0; i < source.size(); ++i) {
        const Eigen::Vector3d residual = target[i].pose - source[i].pose;

        Eigen::Vector4d hom_point(target[i].pose.x(), target[i].pose.y(), target[i].pose.z(), 1.0);
        Eigen::Vector4d transformed_hom_point = iter_pose.inverse() * hom_point;
        
        const Eigen::Vector3d target_local = transformed_hom_point.head<3>();
        const Eigen::Vector3d residual_local = target_local - source[i].local.head<3>();

        Eigen::Matrix3_6d J_r;

        double range_weight = 1.0;


        if(icp_type_ == IcpType::P2PCOV){

            Eigen::Matrix4d RCR;
            RCR = iter_pose.matrix() * source[i].cov * iter_pose.matrix().transpose(); // Only Source Point Cov
            RCR(3, 3) = 1.0;

            Eigen::Matrix4d mahalanobis = RCR.inverse(); // (CiB + T*CiA*T^(-1))^(−1)
            mahalanobis(3, 3) = 0.0f;

            Eigen::Vector4d error = Eigen::Vector4d::Zero();
            error.head<3>() = (target[i].pose - source[i].pose);

            range_weight = (double)(error.transpose() * mahalanobis * error) / error.squaredNorm();


        }
        else if (icp_type_ == IcpType::PCOV2PCOV){

            Eigen::Matrix4d RCR;
            RCR = target[i].sensor_pose.matrix() * target[i].cov * target[i].sensor_pose.matrix().transpose() + 
                              iter_pose.matrix() * source[i].cov * iter_pose.matrix().transpose();
            RCR(3, 3) = 1.0;

            Eigen::Matrix4d mahalanobis = RCR.inverse(); // (CiB + T*CiA*T^(-1))^(−1)
            mahalanobis(3, 3) = 0.0f;

            Eigen::Vector4d error = Eigen::Vector4d::Zero();
            error.head<3>() = (target[i].pose - source[i].pose);

            range_weight = (double)(error.transpose() * mahalanobis * error) / error.squaredNorm();
        }

        // [ I(3x3), -(T p_k)^ ]
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * vectorToSkewSymmetricMatrix(source[i].local);

        double weight = square(th) / square(th + residual_local.squaredNorm()) * range_weight;
        JTJ.noalias() += J_r.transpose() * weight * J_r;
        JTr.noalias() += J_r.transpose() * weight * residual_local;

    }

    Eigen::Matrix6d JTJ_diag = JTJ.diagonal().asDiagonal();
    const Eigen::Vector6d x = (JTJ + lm_lambda_ * JTJ_diag).ldlt().solve(JTr);

    Eigen::Vector3d rotation_vector = x.tail<3>(); // rpy
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())).toRotationMatrix(); // rotation
    transformation.block<3, 1>(0, 3) = x.head<3>(); // transform xyz

    // Source Point 의 센서 좌표 기준 미소 transformation
    return transformation;
}

Eigen::Matrix4d Registration::AlignCloudsLocalDoppler6DoF(std::vector<RadarPoint> &source,
                            const std::vector<RadarPoint> &target,
                            Eigen::Matrix4d &iter_pose,
                            const Velocity &sensor_velocity,
                            double trans_th, double vel_th) {

    Eigen::Matrix6d JTJ_tot = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JTr_tot = Eigen::Vector6d::Zero();

    // yaw 각도를 라디안으로 변환
    double ego_to_radar_yaw_rad = ego_to_radar_yaw_deg_ * M_PI / 180.0;

    // 회전 행렬 생성 (2D 회전)
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << cos(ego_to_radar_yaw_rad), -sin(ego_to_radar_yaw_rad), 0,
                    sin(ego_to_radar_yaw_rad),  cos(ego_to_radar_yaw_rad), 0,
                    0, 0, 1;

    Eigen::Vector3d est_sensor_vel_vehicle_coord = sensor_velocity.linear;
    // Eigen::Vector3d est_sensor_vel_vehicle_coord(sensor_velocity.linear.x(), sensor_velocity.linear.y(), 0.0);
    // Eigen::Vector3d ego_to_sensor_translation(ego_to_radar_x_m_, 0.0, 0.5);
    Eigen::Vector3d ego_to_sensor_translation(ego_to_radar_x_m_, 0.0, 0.0);

    for (size_t i = 0; i < source.size(); ++i) {
        Eigen::Vector4d hom_point(target[i].pose.x(), target[i].pose.y(), target[i].pose.z(), 1.0);
        Eigen::Vector4d transformed_hom_point = iter_pose.inverse() * hom_point;
        
        const Eigen::Vector3d target_local = transformed_hom_point.head<3>();
        const Eigen::Vector3d residual_local = target_local - source[i].local.head<3>();

        Eigen::Matrix3_6d J_r;

        double range_weight = 1.0;

        double target_static_weight = 1.0;
        
        if(target[i].is_static == false){
            target_static_weight = 0.01;
        }

        if(icp_type_ == IcpType::P2PCOV){

            Eigen::Matrix4d RCR;
            RCR = iter_pose.matrix() * source[i].cov * iter_pose.matrix().transpose(); // Only Source Point Cov
            RCR(3, 3) = 1.0;

            Eigen::Matrix4d mahalanobis = RCR.inverse(); // (CiB + T*CiA*T^(-1))^(−1)
            mahalanobis(3, 3) = 0.0f;

            Eigen::Vector4d error = Eigen::Vector4d::Zero();
            error.head<3>() = (target[i].pose - source[i].pose);

            range_weight = (double)(error.transpose() * mahalanobis * error) / error.squaredNorm();


        }
        else if (icp_type_ == IcpType::PCOV2PCOV){

            Eigen::Matrix4d RCR;
            RCR = target[i].sensor_pose.matrix() * target[i].cov * target[i].sensor_pose.matrix().transpose() + 
                              iter_pose.matrix() * source[i].cov * iter_pose.matrix().transpose();
            RCR(3, 3) = 1.0;

            Eigen::Matrix4d mahalanobis = RCR.inverse(); // (CiB + T*CiA*T^(-1))^(−1)
            mahalanobis(3, 3) = 0.0f;

            Eigen::Vector4d error = Eigen::Vector4d::Zero();
            error.head<3>() = (target[i].pose - source[i].pose);

            range_weight = (double)(error.transpose() * mahalanobis * error) / error.squaredNorm();
        }

        // [ I(3x3), -(T p_k)^ ]
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r(2,2) = 0.0; // FIXME: Not optimizing Z for translation
        J_r.block<3, 3>(0, 3) = -1.0 * vectorToSkewSymmetricMatrix(source[i].local);
        // J_r.block<3, 1>(0, 4) << 0.0, 0.0, 0.0; // FIXME: Not optimizing Pitch for translation
        // J_r.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(); // FIXME: Not optimizing Rotation for translation

        double weight_t = square(trans_th) / square(trans_th + residual_local.squaredNorm()) * range_weight * target_static_weight;

        // 2. Velocity
        // V_D_k - V_est(T)
        const double p_azim_rad = source[i].azi_angle * M_PI / 180.0f; // 정면 0 반시계
        const double p_ele_rad = source[i].ele_angle * M_PI / 180.0f; // 정면 0 윗방향

        Eigen::Vector3d point_direction_vector(cos(p_ele_rad) * cos(p_azim_rad),
                                               cos(p_ele_rad) * sin(p_azim_rad), 
                                               sin(p_ele_rad));
        Eigen::Vector3d point_direction_vector_ego = rotation_matrix * point_direction_vector;

        // Eigen::Vector3d point_direction_vector(cos(p_azim_rad),sin(p_azim_rad), 0.0);


        // 차량의 속도 벡터와 관측 각도를 통해 도플러 속도 예상. 전진시 음수
        double est_point_vel = - point_direction_vector.dot(est_sensor_vel_vehicle_coord);
        double vel_residual = source[i].vel - est_point_vel;

        Eigen::Matrix1_6d J_v;

        // [ - d_k / dt , -d_k x t_s / dt ]
        J_v.block<1,3>(0,0) = - point_direction_vector.transpose() / sensor_velocity.time_diff_sec;
        J_v.block<1,3>(0,3) = - (point_direction_vector_ego.cross(ego_to_sensor_translation)).transpose() / sensor_velocity.time_diff_sec;
        // J_v.block<1,3>(0,3) << 0.0, 0.0, 0.0; // FIXME: Not Optimizing Rotation for Doppler
        J_v(0,2) = 0.0; // FIXME: Not optimizing z for Doppler

        double weight_v = square(vel_th) / square(vel_th + square(vel_residual)) * range_weight;

        // static 속성 부여
        if(fabs(vel_residual) < vel_th * 3.0 && residual_local.norm() < 0.6){
            source[i].is_static = true;
        }


        Eigen::Matrix4_6d J_tot;
        Eigen::Matrix1_4d R_tot;
    
        double sqrt_trans_weight = sqrt(weight_t * (1.0 - doppler_trans_lambda_));
        double sqrt_vel_weight = sqrt(weight_v * doppler_trans_lambda_);

        J_tot.block<3,6>(0,0) = J_r * sqrt_trans_weight;
        J_tot.block<1,6>(3,0) = J_v * sqrt_vel_weight;

        R_tot.block<1,3>(0,0) << residual_local.x() * sqrt_trans_weight, 
                                 residual_local.y() * sqrt_trans_weight,
                                 residual_local.z() * sqrt_trans_weight;
        R_tot(0,3) = vel_residual * sqrt_vel_weight;

        JTJ_tot.noalias() += J_tot.transpose() * J_tot; // 6x4 4x6
        JTr_tot.noalias() += J_tot.transpose() * R_tot.transpose(); // 6x4 4x1

    }

    Eigen::Matrix6d JTJ_tot_diag = JTJ_tot.diagonal().asDiagonal();
    // const Eigen::Vector6d x_tot = (JTJ_tot + lm_lambda_ * JTJ_tot_diag).ldlt().solve(JTr_tot);
    const Eigen::Vector6d x_tot = (JTJ_tot + lm_lambda_ * JTJ_tot_diag).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(JTr_tot);



    Eigen::Vector3d rotation_vector = x_tot.tail<3>(); // rpy
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())).toRotationMatrix(); // rotation
    transformation.block<3, 1>(0, 3) = x_tot.head<3>(); // transform xyz

    // Source Point 의 센서 좌표 기준 미소 transformation
    return transformation;
}


Eigen::Matrix4d Registration::RunRegister(const std::vector<RadarPoint> &frame,
                        std::vector<RadarPoint> &frame_global,
                        const VoxelHashMap &voxel_map,
                        const Eigen::Matrix4d &initial_guess,
                        const Eigen::Matrix4d &last_pose,
                        const double dt,
                        double trans_sigma,
                        double vel_sigma)
{

    doppler_gm_th_ = std::max(doppler_gm_th_, DBL_MIN);
    doppler_trans_lambda_ = std::min(std::max(doppler_trans_lambda_, 0.0), 1.0);
    std::vector<int> source_indices;
    std::vector<RadarPoint> source_c_global, target_c_global;

    std::vector<RadarPoint> source_local = frame;  

    // 1. Global 좌표계의 source 생성 및 point uncertaintly 계산
    if(icp_3dof_ == true){
        CalFramePointCov2d(source_local, range_variance_m_, azimuth_variance_deg_);
    }
    else{
        CalFramePointCov(source_local, range_variance_m_, azimuth_variance_deg_, elevation_variance_deg_);
    }

    std::vector<RadarPoint> source_global = source_local;
    TransformPoints(initial_guess, source_local, source_global);
    frame_global = source_global;

    if (voxel_map.Empty()){
        return initial_guess;
    } 

    // 2. ICP 수행
    Eigen::Matrix4d last_icp_pose = initial_guess;
    
    int i_iter_num = 0;
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        i_iter_num++;

        // Get Correspodence in global frame
        
        if (icp_type_ == IcpType::P2P || icp_type_ == IcpType::P2PCOV || icp_type_ == IcpType::PCOV2PCOV) {
            // std::tie(source_c_global, target_c_global) = voxel_map.GetCorrespondences(source_global, trans_sigma * 3);
            std::tie(source_indices, source_c_global, target_c_global) = voxel_map.GetCorrespondencesWithIdx(source_global, trans_sigma * 3);
        } else {
            std::tie(source_c_global, target_c_global) = voxel_map.GetCorrespondencesCov(source_global, trans_sigma * 3, gicp_max_point_);
        }

        Eigen::Matrix4d estimation_local = Eigen::Matrix4d::Identity(); // 센서 좌표계 상에서의 ICP 미소 변화량

        if(icp_3dof_ == true){
            if(icp_doppler_ == true){
                Velocity iter_velocity = CalculateVelocity(last_pose.inverse() * last_icp_pose, dt); // last pose 기준 상대속도
                estimation_local = AlignCloudsLocalDoppler3DoF(source_c_global, target_c_global, last_icp_pose, iter_velocity, 
                                                                trans_sigma / 3.0, vel_sigma / 3.0);
            }
            else{
                estimation_local = AlignCloudsLocal3DoF(source_c_global, target_c_global, last_icp_pose, trans_sigma / 3.0);
            }
        }
        else{
            if(icp_doppler_ == true){
                Velocity iter_velocity = CalculateVelocity(last_pose.inverse() * last_icp_pose, dt); // last pose 기준 상대속도
                estimation_local = AlignCloudsLocalDoppler6DoF(source_c_global, target_c_global, last_icp_pose, iter_velocity, 
                                                                trans_sigma / 3.0, vel_sigma / 3.0);
            }
            else{
                estimation_local = AlignCloudsLocal6DoF(source_c_global, target_c_global, last_icp_pose, trans_sigma / 3.0);
            }
        }

        // 전역 좌표계상의 추정된 포즈 계산
        last_icp_pose = last_icp_pose * estimation_local;

        // 전역 좌표계 변화량으로 source global 미소이동
        TransformPoints(last_icp_pose, source_local, source_global);

        // ICP 종료 조건 확인
        // double rot_norm = estimation_local.block<3, 3>(0, 0).eulerAngles(0, 1, 2).norm();

        Eigen::AngleAxisd angleAxis(estimation_local.block<3, 3>(0, 0));
        double rot_norm = angleAxis.angle(); 

        double transform_norm = rot_norm + estimation_local.block<3, 1>(0, 3).norm();
        std::cout<<"Norm "<<transform_norm<<"\t r_norm: "<<rot_norm<<"\t s_norm: "<<estimation_local.block<3, 1>(0, 3).norm()<<std::endl;
        if (transform_norm < ESTIMATION_THRESHOLD_) break;

    }

    // Association 된 point 에 static 부여
    for(int t_idx = 0; t_idx < source_indices.size(); t_idx++){
        
        if(source_c_global[t_idx].is_static == true){
            frame_global[source_indices[t_idx]].is_static = true;
        }

    }

    std::cout<<"RegisterFrameLocal Iter: "<<i_iter_num<<std::endl;

    // (전역 ICP 누적 변화량)
    return last_icp_pose;
}
}