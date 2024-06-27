#include "algorithm/registration.hpp"


namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

using Matrix3_3d = Eigen::Matrix<double, 3, 3>;
using Matrix2_3d = Eigen::Matrix<double, 2, 3>;
using Matrix1_3d = Eigen::Matrix<double, 1, 3>;
using Matrix1_6d = Eigen::Matrix<double, 1, 6>;
using Matrix4_6d = Eigen::Matrix<double, 4, 6>;

// using Matrix3d = Eigen::Matrix<double, 3, 1>;
}  // namespace Eigen


// Variables and functions only used in this cpp
namespace{

inline double square(double x) { return x * x; }

constexpr int MAX_NUM_ITERATIONS_ = 50;
constexpr double ESTIMATION_THRESHOLD_ = 0.001;


pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree_;

Eigen::Affine3f incrementalOdometryAffineBack;
Eigen::Affine3f incrementalOdometryAffineFront;
Eigen::Affine3f m_init_tf;

float transformTobeMapped[6] = {0};

// Ailab
void DecomposeTransform(const Eigen::Matrix4d& transform, float transformTobeMapped[6]) {
    // 평행 이동 벡터 추출
    transformTobeMapped[3] = static_cast<float>(transform(0, 3)); // x
    transformTobeMapped[4] = static_cast<float>(transform(1, 3)); // y
    transformTobeMapped[5] = static_cast<float>(transform(2, 3)); // z

    // 회전 행렬 추출
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);

    // 회전 행렬을 roll, pitch, yaw로 변환
    transformTobeMapped[0] = static_cast<float>(std::atan2(rotation(2, 1), rotation(2, 2))); // roll
    transformTobeMapped[1] = static_cast<float>(std::atan2(-rotation(2, 0), std::sqrt(rotation(2, 1) * rotation(2, 1) + rotation(2, 2) * rotation(2, 2)))); // pitch
    transformTobeMapped[2] = static_cast<float>(std::atan2(rotation(1, 0), rotation(0, 0))); // yaw
}

// Ailab
void ComposeTransform(const float transformTobeMapped[6], Eigen::Matrix4d& transform) {
    // 회전 행렬 생성
    Eigen::Matrix3d rotation;
    double roll = static_cast<double>(transformTobeMapped[0]);
    double pitch = static_cast<double>(transformTobeMapped[1]);
    double yaw = static_cast<double>(transformTobeMapped[2]);

    rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    // 변환 행렬 초기화
    transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;

    // 평행 이동 벡터 설정
    transform(0, 3) = static_cast<double>(transformTobeMapped[3]); // x
    transform(1, 3) = static_cast<double>(transformTobeMapped[4]); // y
    transform(2, 3) = static_cast<double>(transformTobeMapped[5]); // z
}


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


std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> GetCorrespondences(const std::vector<RadarPoint> source,
                                                                                       double max_correspondance_distance)
{
    std::vector<Eigen::Vector3d> source_points;
    std::vector<Eigen::Vector3d> target_points;

    int i_source_num = source.size();

    for (int i = 0; i < i_source_num; ++i) {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        pcl::PointXYZINormal search_point;
        search_point.x = source[i].pose.x();
        search_point.y = source[i].pose.y();
        search_point.z = source[i].pose.z();

        if (kdtree_.nearestKSearch(search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] < max_correspondance_distance) {
                Eigen::Vector3d src_point(search_point.x, search_point.y, search_point.z);
                Eigen::Vector3d tgt_point(kdtree_.getInputCloud()->points[pointIdxNKNSearch[0]].x,
                                          kdtree_.getInputCloud()->points[pointIdxNKNSearch[0]].y,
                                          kdtree_.getInputCloud()->points[pointIdxNKNSearch[0]].z);
                
                source_points.push_back(src_point);
                target_points.push_back(tgt_point);
            }
        }
    }

    return std::make_tuple(source_points, target_points);
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> 
                                    GetCorrespondencesOrigin(const std::vector<RadarPoint> source_origin,
                                    const std::vector<RadarPoint> source_glob,
                                    double max_correspondance_distance)
{
    std::vector<Eigen::Vector3d> source_origin_points;
    std::vector<Eigen::Vector3d> source_glob_points;
    std::vector<Eigen::Vector3d> target_points;

    int i_source_num = source_glob.size();

    for (int i = 0; i < i_source_num; ++i) {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        pcl::PointXYZINormal glob_point, origin_point;
        origin_point.x = source_origin[i].pose.x();
        origin_point.y = source_origin[i].pose.y();
        origin_point.z = source_origin[i].pose.z();

        glob_point.x = source_glob[i].pose.x();
        glob_point.y = source_glob[i].pose.y();
        glob_point.z = source_glob[i].pose.z();

        if (kdtree_.nearestKSearch(glob_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] < max_correspondance_distance) {

                Eigen::Vector3d src_origin_point(origin_point.x, origin_point.y, origin_point.z);
                Eigen::Vector3d src_glob_point(glob_point.x, glob_point.y, glob_point.z);
                Eigen::Vector3d tgt_point(kdtree_.getInputCloud()->points[pointIdxNKNSearch[0]].x,
                                          kdtree_.getInputCloud()->points[pointIdxNKNSearch[0]].y,
                                          kdtree_.getInputCloud()->points[pointIdxNKNSearch[0]].z);
                
                source_origin_points.push_back(src_origin_point);
                source_glob_points.push_back(src_glob_point);
                target_points.push_back(tgt_point);
            }
        }
    }

    return std::make_tuple(source_origin_points, source_glob_points, target_points);
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
                            double th) {
    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero();

    Eigen::Matrix6d JvTJv = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JvTrv = Eigen::Vector6d::Zero();

    Eigen::Vector3d est_sensor_vel_vehicle_coord = sensor_velocity.linear; // TODO

    double vel_diff_sum = 0.0;

    for (size_t i = 0; i < source.size(); ++i) {
        const Eigen::Vector3d residual = target[i].pose - source[i].pose;
        Eigen::Matrix3_6d J_r;

        // 1. Translation
        // [ I(3x3), -(T p_k)^ ]
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * vectorToSkewSymmetricMatrix(source[i].pose);

        double weight = square(th) / square(th + residual.squaredNorm());

        JTJ.noalias() += J_r.transpose() * weight * J_r;
        JTr.noalias() += J_r.transpose() * weight * residual;

        // 2. Velocity
        // V_D_k - V_est(T)
        const double p_azim_rad = source[i].azi_angle * M_PI / 180.0f; // 정면 0 반시계
        const double p_ele_rad = source[i].azi_angle * M_PI / 180.0f; // 정면 0 윗방향

        Eigen::Vector3d point_direction_vector(cos(p_ele_rad) * cos(p_azim_rad),
                                         cos(p_ele_rad) * sin(p_azim_rad), 
                                         sin(p_ele_rad));
        
        Eigen::Vector3d ego_to_sensor_translation(3.5, 0.0, 0.0);

        // 차량의 속도 벡터와 관측 각도를 통해 도플러 속도 예상. 전진시 음수
        double est_point_vel = -point_direction_vector.dot(est_sensor_vel_vehicle_coord);
        double vel_residual = source[i].vel - est_point_vel;

        vel_diff_sum += fabs(vel_residual);

        Eigen::Matrix1_6d J_v;

        // [ - d_k / dt , -d_k x t_s / dt ]
        J_v.block<1,3>(0,0) = - point_direction_vector.transpose() / 0.1;
        J_v.block<1,3>(0,3) = - (point_direction_vector.cross(ego_to_sensor_translation)).transpose() / 0.1;

        JvTJv.noalias() += J_v.transpose() * J_v; // 6x1 1x6
        JvTrv.noalias() += J_v.transpose() * vel_residual; // 6x1 1x1
    }

    std::cout<<"Vel diff mean: "<<vel_diff_sum / source.size()<<std::endl;

    const Eigen::Vector6d x = JTJ.ldlt().solve(JTr);

    const Eigen::Vector6d x_v = JvTJv.ldlt().solve(JvTrv);

    std::cout<<"x_v"<<std::endl;
    std::cout<<x_v<<std::endl;


    Eigen::Vector3d rotation_vector = x.tail<3>(); // rpy
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())).toRotationMatrix(); // rotation
    transformation.block<3, 1>(0, 3) = x.head<3>(); // transform xyz

    return transformation;
}

Eigen::Matrix4d AlignClouds3DoF(const std::vector<RadarPoint> &source,
                                const std::vector<RadarPoint> &target,
                                double th) {
    Eigen::Matrix3d JTJ = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr = Eigen::Vector3d::Zero();

    Eigen::Matrix3d JTJDiag = Eigen::Matrix3d::Zero();

    for (size_t i = 0; i < source.size(); ++i) {
        const Eigen::Vector2d residual(source[i].pose.x() - target[i].pose.x(), source[i].pose.y() - target[i].pose.y());
        Eigen::Matrix2_3d J_r;

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
                            double th) {
    Eigen::Matrix3d JTJ = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr = Eigen::Vector3d::Zero();

    Eigen::Matrix3d JvTJv = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JvTrv = Eigen::Vector3d::Zero();

    Eigen::Matrix3d JTJ_tot = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr_tot = Eigen::Vector3d::Zero();

    Eigen::Vector2d est_sensor_vel_vehicle_coord(sensor_velocity.linear.x(), sensor_velocity.linear.y()); // TODO

    double vel_diff_sum = 0.0;

    for (size_t i = 0; i < source.size(); ++i) {
        const Eigen::Vector2d residual(target[i].pose.x() - source[i].pose.x(), target[i].pose.y() - source[i].pose.y());
        Eigen::Matrix2_3d J_r;

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
        const double p_ele_rad = source[i].azi_angle * M_PI / 180.0f; // 정면 0 윗방향

        Eigen::Vector2d point_direction_vector(cos(p_azim_rad), sin(p_azim_rad));

        Eigen::Vector2d ego_to_sensor_translation(3.5, 0.0);

        // 차량의 속도 벡터와 관측 각도를 통해 도플러 속도 예상. 전진시 음수
        double est_point_vel = -point_direction_vector.dot(est_sensor_vel_vehicle_coord);
        double vel_residual = source[i].vel - est_point_vel;

        vel_diff_sum += fabs(vel_residual);

        Eigen::Matrix1_3d J_v;

        // [ - d_k / dt , -d_k x t_s / dt ]
        J_v.block<1,2>(0,0) = - point_direction_vector.transpose() / 0.1;
        J_v(0,2) = - (point_direction_vector.x() * ego_to_sensor_translation.y() - point_direction_vector.y() * ego_to_sensor_translation.x()) / 0.1;

        double weight_v = square(th) / square(th + vel_residual);

        JvTJv.noalias() += J_v.transpose() * weight_v * J_v; // 3x1 1x3
        JvTrv.noalias() += J_v.transpose() * weight_v * vel_residual; // 3x1 1x1

        //

        Eigen::Matrix3d J_tot;

        Eigen::Matrix1_3d R_tot(residual.x(), residual.y(), vel_residual);
    
        J_tot.block<2,3>(0,0) = J_r;
        J_tot.block<1,3>(2,0) = J_v;

        JTJ_tot.noalias() += J_tot.transpose() * weight_t * J_tot; // 3x4 4x3
        JTr_tot.noalias() += J_tot.transpose() * weight_t * R_tot.transpose(); // 3x3 3x1

    }

    std::cout<<"Vel diff mean: "<<vel_diff_sum / source.size()<<std::endl;

    const Eigen::Vector3d x = JTJ.ldlt().solve(JTr);

    const Eigen::Vector3d x_v = JvTJv.ldlt().solve(JvTrv);

    const Eigen::Vector3d x_tot = JTJ_tot.ldlt().solve(JTr_tot);

    std::cout<<"x_v"<<std::endl;
    std::cout<<x_v<<std::endl;

    std::cout<<"x_tot"<<std::endl;
    std::cout<<x_tot<<std::endl;

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    transformation.block<2, 2>(0, 0) = Eigen::Rotation2Dd(x_tot(2)).toRotationMatrix();
    transformation(0, 3) = x_tot(0);
    transformation(1, 3) = x_tot(1);


    return transformation;
}


bool LMOptimization(const std::vector<Eigen::Vector3d> &source_origin,
                    const std::vector<Eigen::Vector3d> &source_glob,
                    const std::vector<Eigen::Vector3d> &target_glob,
                    double th) 
{
    // Eigen::Matrix3d rotation_matrix = cur_transform.block<3, 3>(0, 0);

    // // yaw 각도 계산
    // double cur_yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    double sin_yaw = sin(transformTobeMapped[2]);
    double cos_yaw = cos(transformTobeMapped[2]);

    int laserCloudSelNum = source_origin.size();

    Eigen::MatrixXf matA(laserCloudSelNum, 3);
    Eigen::VectorXf matB(laserCloudSelNum);
    Eigen::MatrixXf matAtA(3, 3);
    Eigen::MatrixXf matAtB(3, 1);
    Eigen::VectorXf matX(3);
    
    for (int i = 0; i < laserCloudSelNum; i++) {

        Eigen::Vector2d source_2d(source_glob[i].x(), source_glob[i].y());
        Eigen::Vector2d target_2d(target_glob[i].x(), target_glob[i].y());
        Eigen::Vector2d residual = source_2d - target_2d; // A
        Eigen::Vector2d norm_residual = residual / residual.norm();

        float yaw_grad = (cos_yaw * source_origin[i].x() + sin_yaw * source_origin[i].y()) * norm_residual.x()
                       + (sin_yaw * source_origin[i].x() - cos_yaw * source_origin[i].y()) * norm_residual.y();

        double score = square(th) / square(th + residual.squaredNorm());

        matA(i, 0) = score * yaw_grad ; // yaw gradient
        matA(i, 1) = score * norm_residual.x(); // x gradient
        matA(i, 2) = score * norm_residual.y(); // y gradient
        matB(i)    = - score * residual.norm(); // error

    }

    Eigen::MatrixXf matAt = matA.transpose();
    Eigen::MatrixXf matAtAdiag = (matAt * matA).diagonal().asDiagonal();

    matAtA = matAt * matA + 1.0 * matAtAdiag;
    matAtB = matAt * matB;

    matX = matAtA.colPivHouseholderQr().solve(matAtB);

    double delta_yaw = matX(0);
    double delta_x = matX(1);
    double delta_y = matX(2);

    transformTobeMapped[2] += delta_yaw;
    transformTobeMapped[3] += delta_x;
    transformTobeMapped[4] += delta_y;

    float deltaR = fabs(delta_yaw); // deg
    float deltaT = sqrt(pow(delta_x, 2) +
                        pow(delta_y, 2)); // m
 
    if (deltaR < ESTIMATION_THRESHOLD_ && deltaT < ESTIMATION_THRESHOLD_) {
        return true; // converged
    }
    return false;
}
}

namespace radar_odometry {


Eigen::Matrix4d RegisterScan2Map(const std::vector<RadarPoint> &frame,
                                    const VoxelHashMap &voxel_map,
                                    const  Eigen::Matrix4d &initial_guess,
                                    double max_correspondence_distance,
                                    double kernel) 
{   
    if (voxel_map.Empty()) return initial_guess;

    std::vector<RadarPoint> cur_origin_points = frame;
    std::vector<RadarPoint> cur_moved_points;
    std::vector<RadarPoint> last_glob_points = voxel_map.Pointcloud();

    
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_xyzin_ptr(new pcl::PointCloud<pcl::PointXYZINormal>);

    int i_source_point_num = frame.size();
    int i_target_point_num = last_glob_points.size();

    std::cout<<"i_source_point_num: "<<i_source_point_num<<std::endl;
    std::cout<<"i_target_point_num: "<<i_target_point_num<<std::endl;


    for(int i = 0; i < i_target_point_num; ++i){
        pcl::PointXYZINormal point;
        point.x = last_glob_points[i].pose.x();
        point.y = last_glob_points[i].pose.y();
        point.z = last_glob_points[i].pose.z();

        target_xyzin_ptr->points.push_back(point);

    }

    kdtree_.setInputCloud(target_xyzin_ptr);

    // 초기 위치 지정
    DecomposeTransform(initial_guess, transformTobeMapped);

    float init_transformTobeMapped[6] = {0};
    for(int i = 0; i < 6; i++){
        init_transformTobeMapped[i] = transformTobeMapped[i];
    }

    std::cout<<"Init x: "<< init_transformTobeMapped[3] <<" y: "<<init_transformTobeMapped[4] <<" yaw deg: "<<init_transformTobeMapped[2]*180/M_PI<<std::endl;

    Eigen::Matrix4d T_icp = initial_guess;

    int iter_count = 0;
    for(int j = 0; j < MAX_NUM_ITERATIONS_; ++j){
        iter_count++;
        // cur_origin_points to Source_glob transform
        TransformPoints(T_icp, cur_origin_points, cur_moved_points);

        // Correspondence 형성
        auto [source_origin, source_glob, target_glob] = GetCorrespondencesOrigin(cur_origin_points, cur_moved_points, max_correspondence_distance);

        if(source_origin.size() < 10 ){
            std::cout<<"Too Small # association: "<< source_origin.size() << std::endl;
            break;
        }

        // LM Optimization 수행
        if(LMOptimization(source_origin, source_glob, target_glob, kernel) == true){
            ComposeTransform(transformTobeMapped, T_icp);
            break;
        }

        // std::cout<<" After Ticp x: "<< transformTobeMapped[3] <<" y: "<<transformTobeMapped[4] <<" yaw: "<<transformTobeMapped[2]<<std::endl;
        
        //
        ComposeTransform(transformTobeMapped, T_icp);
        // T_icp =  estimation * T_icp;
    }
    

    std::cout<<"Del  x: "<< transformTobeMapped[3] - init_transformTobeMapped[3]
             <<" y: "<<transformTobeMapped[4] - init_transformTobeMapped[4]
             <<" yaw deg: "<<(transformTobeMapped[2] - init_transformTobeMapped[2]) *180/M_PI <<" iter: "<< iter_count <<std::endl;

    


    return T_icp;
}

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
                              const Velocity &sensor_velocity,
                              double max_correspondence_distance,
                              double kernel) {
    if (voxel_map.Empty()) return initial_guess;

    std::vector<RadarPoint> source = frame;
    TransformPoints(initial_guess, source);

    Eigen::Matrix4d T_icp = Eigen::Matrix4d::Identity();
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        const auto &[src, tgt] = voxel_map.GetCorrespondences(source, max_correspondence_distance);
        Eigen::Matrix4d estimation = AlignCloudsDoppler(src, tgt, sensor_velocity, kernel);
        TransformPoints(estimation, source);
        T_icp = estimation * T_icp;
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
                           double kernel) {
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

        Eigen::Matrix4d estimation = AlignCloudsDoppler3DoF(src, tgt, iter_velocity, kernel);
        TransformPoints(estimation, source);
        T_icp = estimation * T_icp;

        last_icp_pose = T_icp * initial_guess;

        double yaw = std::atan2(estimation(1, 0), estimation(0, 0));
        if ((std::abs(yaw) + estimation.block<2, 1>(0, 3).norm()) < ESTIMATION_THRESHOLD_) break;

    }
    return T_icp * initial_guess;
}

}