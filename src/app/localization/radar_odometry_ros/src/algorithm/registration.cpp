#include "algorithm/registration.hpp"


namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen


// Variables and functions only used in this cpp
namespace{

inline double square(double x) { return x * x; }

struct ResultTuple {
    ResultTuple() {
        JTJ.setZero();
        JTr.setZero();
    }

    ResultTuple operator+(const ResultTuple &other) {
        this->JTJ += other.JTJ;
        this->JTr += other.JTr;
        return *this;
    }

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
};

constexpr int MAX_NUM_ITERATIONS_ = 20;
constexpr double ESTIMATION_THRESHOLD_ = 0.001;


pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree_;

Eigen::Affine3f incrementalOdometryAffineBack;
Eigen::Affine3f incrementalOdometryAffineFront;
Eigen::Affine3f m_init_tf;

float transformTobeMapped[6] = {0};


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


void TransformPoints(const Eigen::Matrix4d &transform, pcl::PointCloud<PointXYZPRVAE> &i_points) {

    for (size_t i = 0; i < i_points.points.size(); ++i) {
        // 입력 포인트를 Eigen::Vector4d 동차 좌표로 변환
        Eigen::Vector4d input_point(i_points.points[i].x, i_points.points[i].y, i_points.points[i].z, 1.0);

        // 변환 행렬을 사용하여 출력 포인트를 계산
        Eigen::Vector4d output_point = transform * input_point;

        // 변환된 좌표를 출력 포인트로 저장
        i_points.points[i].x = output_point.x();
        i_points.points[i].y = output_point.y();
        i_points.points[i].z = output_point.z();
    }
}

void TransformPoints(const Eigen::Matrix4d &transform, const pcl::PointCloud<PointXYZPRVAE> i_points, pcl::PointCloud<PointXYZPRVAE>& o_points) {
    o_points.clear();
    o_points = i_points;
    
    // 변환 행렬의 상위 3x3 부분을 회전 행렬로 사용
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);

    // 변환 행렬의 상위 3x1 부분을 평행 이동 벡터로 사용
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);

    for (size_t i = 0; i < i_points.points.size(); ++i) {
        // 입력 포인트를 Eigen::Vector4d 동차 좌표로 변환
        Eigen::Vector4d input_point(i_points.points[i].x, i_points.points[i].y, i_points.points[i].z, 1.0);

        // 변환 행렬을 사용하여 출력 포인트를 계산
        Eigen::Vector4d output_point = transform * input_point;

        // 변환된 좌표를 출력 포인트로 저장
        o_points.points[i].x = output_point.x();
        o_points.points[i].y = output_point.y();
        o_points.points[i].z = output_point.z();
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

Eigen::Matrix3d vectorToSkewSymmetricMatrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d v_hat;
    v_hat <<    0, -v.z(),  v.y(),
             v.z(),     0, -v.x(),
            -v.y(),  v.x(),     0;
    return v_hat;
}

Eigen::Matrix2d vector2dToSkewSymmetricMatrix(const Eigen::Vector2d &v) {
    Eigen::Matrix2d skew_symmetric;
    skew_symmetric << 0, -v.y(),
                      v.y(), 0;
    return skew_symmetric;
}


std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> GetCorrespondences(const pcl::PointCloud<PointXYZPRVAE> source,
                                                                                       double max_correspondance_distance)
{
    std::vector<Eigen::Vector3d> source_points;
    std::vector<Eigen::Vector3d> target_points;

    int i_source_num = source.points.size();

    for (int i = 0; i < i_source_num; ++i) {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        pcl::PointXYZINormal search_point;
        search_point.x = source.points[i].x;
        search_point.y = source.points[i].y;
        search_point.z = source.points[i].z;

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
                                    GetCorrespondencesOrigin(const pcl::PointCloud<PointXYZPRVAE> source_origin,
                                    const pcl::PointCloud<PointXYZPRVAE> source_glob,
                                    double max_correspondance_distance)
{
    std::vector<Eigen::Vector3d> source_origin_points;
    std::vector<Eigen::Vector3d> source_glob_points;
    std::vector<Eigen::Vector3d> target_points;

    int i_source_num = source_glob.points.size();

    for (int i = 0; i < i_source_num; ++i) {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        pcl::PointXYZINormal glob_point, origin_point;
        origin_point.x = source_origin.points[i].x;
        origin_point.y = source_origin.points[i].y;
        origin_point.z = source_origin.points[i].z;

        glob_point.x = source_glob.points[i].x;
        glob_point.y = source_glob.points[i].y;
        glob_point.z = source_glob.points[i].z;

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



Eigen::Matrix4d AlignClouds(const std::vector<Eigen::Vector3d> &source,
                            const std::vector<Eigen::Vector3d> &target,
                            double th) {
    auto compute_jacobian_and_residual = [&](size_t i) {
        const Eigen::Vector3d residual = source[i] - target[i];
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * vectorToSkewSymmetricMatrix(source[i]);
        return std::make_tuple(J_r, residual);
    };

    ResultTuple result;
    auto Weight = [&](double residual2) { return square(th) / square(th + residual2); };

    for (size_t i = 0; i < source.size(); ++i) {
        const auto &[J_r, residual] = compute_jacobian_and_residual(i);
        const double w = Weight(residual.squaredNorm());
        result.JTJ.noalias() += J_r.transpose() * w * J_r;
        result.JTr.noalias() += J_r.transpose() * w * residual;
    }

    const Eigen::Vector6d x = result.JTJ.ldlt().solve(-result.JTr);

    // 변환 벡터를 SE(3) 변환 행렬로 변환
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisd(x.tail<3>().norm(), x.tail<3>().normalized())).toRotationMatrix();
    transformation.block<3, 1>(0, 3) = x.head<3>();

    return transformation;
}


Eigen::Matrix4d AlignClouds3DoF(const std::vector<Eigen::Vector3d> &source,
                            const std::vector<Eigen::Vector3d> &target,
                            double th) 
{
    int i_pair_num = source.size();

    Eigen::Matrix3d JTJ = Eigen::Matrix3d::Zero();
    Eigen::Vector3d JTr = Eigen::Vector3d::Zero();

    double sqnorm_sum = 0.0;

    for(int i = 0; i < i_pair_num; ++i){
        Eigen::Vector2d source_2d(source[i].x(), source[i].y());
        Eigen::Vector2d target_2d(target[i].x(), target[i].y());
        Eigen::Vector2d residual = source_2d - target_2d; // A
        Eigen::Matrix<double, 2, 3> J_r;
        J_r.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
        J_r.block<2, 1>(0, 2) = vector2dToSkewSymmetricMatrix(source_2d).block<2, 1>(0, 0);

        double weight = square(th) / square(th + residual.squaredNorm());
        JTJ.noalias() += J_r.transpose() * weight * J_r;
        JTr.noalias() += J_r.transpose() * weight * residual;

        sqnorm_sum += residual.squaredNorm();
    }
    
    Eigen::Vector3d x = JTJ.ldlt().solve(-JTr);
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    double yaw = x[2]; 
    transformation.block<3, 3>(0, 0) = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    
    // 2D 변환을 3D 변환으로 변환 (x와 y만 적용)
    transformation(0, 3) = x[0];
    transformation(1, 3) = x[1];
    transformation(2, 3) = 0; // z 변환은 0으로 유지

    std::cout<<"Total: "<< i_pair_num <<" sqnorm mean: "<< sqnorm_sum / i_pair_num <<std::endl;
    std::cout << "dx: " << transformation(0, 3) << " dy: " << transformation(1, 3) << " yaw: " << yaw << std::endl;

    return transformation;
}

Eigen::Matrix4d LMOptimization3DoF(const std::vector<Eigen::Vector3d> &source_origin,
                                    const std::vector<Eigen::Vector3d> &source_glob,
                                    const std::vector<Eigen::Vector3d> &target_glob,
                                    const Eigen::Matrix4d cur_transform,
                                    double th) 
{
    Eigen::Matrix3d rotation_matrix = cur_transform.block<3, 3>(0, 0);

    // yaw 각도 계산
    double cur_yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    double sin_yaw = sin(cur_yaw);
    double cos_yaw = cos(cur_yaw);

    int laserCloudSelNum = source_origin.size();

    Eigen::MatrixXf matA(laserCloudSelNum, 3);
    Eigen::VectorXf matB(laserCloudSelNum);
    Eigen::MatrixXf matAtA(3, 3);
    Eigen::MatrixXf matAtB(3, 1);
    Eigen::VectorXf matX(3);
    
    for (int i = 0; i < laserCloudSelNum; i++) {

        Eigen::Vector2d source_2d(source_glob[i].x(), source_glob[i].y());
        Eigen::Vector2d target_2d(target_glob[i].x(), target_glob[i].y());
        Eigen::Vector2d residual = source_2d - target_2d;
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

    matAtA = matAt * matA + 5.0 * matAtAdiag;
    matAtB = matAt * matB;

    matX = matAtA.colPivHouseholderQr().solve(matAtB);

    // 변화량을 적용할 변환 행렬 계산
    Eigen::Matrix4d delta_transform = Eigen::Matrix4d::Identity();
    double delta_yaw = matX(0);
    double delta_x = matX(1);
    double delta_y = matX(2);


    delta_transform.block<3, 3>(0, 0) = Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    
    Eigen::Vector4d translation(delta_x, delta_y, 0, 1);

    Eigen::Vector4d global_translation = cur_transform * translation;
    delta_transform(0, 3) = global_translation(0);
    delta_transform(1, 3) = global_translation(1);
    delta_transform(2, 3) = global_translation(2);

    std::cout<<"Num: "<<laserCloudSelNum <<" dx: "<< delta_x <<" dy: "<< delta_y << " dyaw: "<<delta_yaw << std::endl;
    
    return delta_transform;
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

    matAtA = matAt * matA + 10.0 * matAtAdiag;
    matAtB = matAt * matB;

    matX = matAtA.colPivHouseholderQr().solve(matAtB);

    double delta_yaw = matX(0);
    double delta_x = matX(1);
    double delta_y = matX(2);

    transformTobeMapped[2] += delta_yaw;
    transformTobeMapped[3] += delta_x;
    transformTobeMapped[4] += delta_y;

    // std::cout<<"Num: "<<laserCloudSelNum <<" dx: "<< delta_x <<" dy: "<< delta_y << " dyaw: "<<delta_yaw << std::endl;
    

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

Eigen::Matrix4d RegisterScan2Scan3DoF(const pcl::PointCloud<PointXYZPRVAE> i_cur_points,
                                      const pcl::PointCloud<PointXYZPRVAE> i_last_points,
                                      const Eigen::Matrix4d &initial_guess,
                                      const Eigen::Matrix4d &last_pose,
                                      double max_correspondence_distance,
                                      double kernel)
{   
    pcl::PointCloud<PointXYZPRVAE> cur_origin_points = i_cur_points;
    pcl::PointCloud<PointXYZPRVAE> cur_moved_points = i_cur_points;
    pcl::PointCloud<PointXYZPRVAE> last_glob_points;

    // 과거 포인트를 global로 이동. 필수적
    TransformPoints(last_pose, i_last_points, last_glob_points);

    TransformPoints(initial_guess, cur_moved_points);
    
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_xyzin_ptr(new pcl::PointCloud<pcl::PointXYZINormal>);

    int i_source_point_num = i_cur_points.points.size();
    int i_target_point_num = last_glob_points.points.size();

    std::cout<<"Targel Local Num: "<<last_glob_points.points.size()<<std::endl;
    std::cout<<"Target Glob Num: "<<i_target_point_num<<std::endl;


    for(int i = 0; i < i_target_point_num; ++i){
        pcl::PointXYZINormal point;
        point.x = last_glob_points.points[i].x;
        point.y = last_glob_points.points[i].y;
        point.z = last_glob_points.points[i].z;
        // TODO

        target_xyzin_ptr->points.push_back(point);

    }


    kdtree_.setInputCloud(target_xyzin_ptr);

    // m_init_tf = init_tf;
    

    Eigen::Matrix4d T_icp = Eigen::Matrix4d::Identity();

    int iter_count = 0;
    for(int j = 0; j < MAX_NUM_ITERATIONS_; ++j){
        iter_count++;
        // Get association
        // auto [source_points, target_points] = GetCorrespondences(i_cur_points, max_correspondence_distance);
        // Eigen::Matrix4d estimation = AlignClouds(source_points, target_points, kernel);

        // TransformPoints(initial_guess * T_icp, cur_origin_points, cur_moved_points);
        // auto [source_origin, source_glob, target_glob] = GetCorrespondencesOrigin(cur_origin_points, cur_moved_points, max_correspondence_distance);

        auto [source_glob, target_glob] = GetCorrespondences(cur_moved_points, max_correspondence_distance);

        if(source_glob.size() < 10 ){
            std::cout<<"Too Small # association: "<< source_glob.size() << std::endl;
            break;
        }

        // 여기서 estimation은 global 기준
        Eigen::Matrix4d estimation = AlignClouds3DoF(source_glob, target_glob, kernel);

        // 
        TransformPoints(estimation, cur_moved_points);

        T_icp =  estimation * T_icp;

        // Terminate
        if(se3_log(estimation).norm() < ESTIMATION_THRESHOLD_) break;
    }

    std::cout<<"Iter Count: "<<iter_count<<std::endl;

    // T_icp 는 global 좌표계 기준
    return T_icp * initial_guess;
    // return initial_guess * T_icp;
}

Eigen::Matrix4d RegisterScan2Scan3DoF2(const pcl::PointCloud<PointXYZPRVAE> i_cur_points,
                                      const pcl::PointCloud<PointXYZPRVAE> i_last_points,
                                      const Eigen::Matrix4d &initial_guess,
                                      const Eigen::Matrix4d &last_pose,
                                      double max_correspondence_distance,
                                      double kernel)
{   
    pcl::PointCloud<PointXYZPRVAE> cur_origin_points = i_cur_points;
    pcl::PointCloud<PointXYZPRVAE> cur_moved_points;
    pcl::PointCloud<PointXYZPRVAE> last_glob_points;

    TransformPoints(last_pose, i_last_points, last_glob_points);

    
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_xyzin_ptr(new pcl::PointCloud<pcl::PointXYZINormal>);

    int i_source_point_num = i_cur_points.points.size();
    int i_target_point_num = last_glob_points.points.size();

    std::cout<<"i_source_point_num: "<<i_source_point_num<<std::endl;
    std::cout<<"i_target_point_num: "<<i_target_point_num<<std::endl;


    for(int i = 0; i < i_target_point_num; ++i){
        pcl::PointXYZINormal point;
        point.x = last_glob_points.points[i].x;
        point.y = last_glob_points.points[i].y;
        point.z = last_glob_points.points[i].z;

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

    for(int j = 0; j < MAX_NUM_ITERATIONS_; ++j){
        
        // transformTobeMapped 는 전역 좌표계 source 위치
        // std::cout<<" "<<std::endl;
        
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
             <<" yaw deg: "<<(transformTobeMapped[2] - init_transformTobeMapped[2]) *180/M_PI <<std::endl;

    


    return T_icp;
}

}