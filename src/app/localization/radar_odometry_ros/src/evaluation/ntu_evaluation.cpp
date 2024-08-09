#include <evaluation/ntu_evaluation.hpp>


NtuEvaluation::NtuEvaluation(){}
NtuEvaluation::~NtuEvaluation(){}

void NtuEvaluation::Evaluation(std::vector<std::pair<double, Eigen::Matrix4d>> vec_time_radar_pose_mat, 
                    std::string result_folder_path, std::string calib_file_path)
{
    std::cout<<"[NtuEvaluation] Start Evaluation"<<std::endl;

    // 파일에서 ntu_radar2imu_mat_ 행렬 읽어오기
    std::ifstream calib_file(calib_file_path);
    if (calib_file.is_open()) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                calib_file >> ntu_radar2imu_mat_(i, j);
            }
        }
        calib_file.close();
    } else {
        std::cerr << "[NtuEvaluation] Error: Could not open calibration file: " << calib_file_path << std::endl;
        return;
    }

    int i_est_radar_pose_num = vec_time_radar_pose_mat.size();

    std::cout<<"[NtuEvaluation] Total est pose num: "<< i_est_radar_pose_num <<std::endl;

    std::ofstream result_file(result_folder_path + "/radar_poses_imu_frame.txt");
    if (!result_file.is_open()) {
        std::cerr << "[NtuEvaluation] Error: Could not open result file for writing." << std::endl;
        return;
    }

    // 소수점 자릿수 지정 (예: 10자리)
    result_file << std::fixed << std::setprecision(10);

    ntu_radar2imu_mat_ = Eigen::Matrix4d::Identity();

    for (const auto& time_pose_pair : vec_time_radar_pose_mat) {
        double timestamp = time_pose_pair.first;
        Eigen::Matrix4d radar_pose = time_pose_pair.second;
        
        // Radar 좌표계를 IMU 좌표계로 변환
        // Eigen::Matrix4d imu_pose = ntu_radar2imu_mat_ * radar_pose * ntu_radar2imu_mat_.inverse();
        Eigen::Matrix4d imu_pose = radar_pose * ntu_radar2imu_mat_;

        // 변환된 pose에서 rotation (3x3)과 translation (3x1) 추출
        Eigen::Matrix3d rotation = imu_pose.block<3,3>(0,0);
        Eigen::Vector3d translation = imu_pose.block<3,1>(0,3);

        // Rotation을 quaternion으로 변환
        Eigen::Quaterniond quaternion(rotation);

        // 결과를 파일에 저장
        result_file << timestamp << " " 
                    << translation(0) << " " << translation(1) << " " << translation(2) << " "
                    << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w()
                    << std::endl;
    }

    result_file.close();
    std::cout<<"[NtuEvaluation] Evaluation completed and saved to: "<< result_folder_path << "/radar_poses_imu_frame.txt" << std::endl;
}