#include <evaluation/vod_evaluation.hpp>



VodEvaluation::VodEvaluation(){}
VodEvaluation::~VodEvaluation(){}

void VodEvaluation::Init()
{
    NodeHandle nh;
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/perception.ini");
    v_ini_parser_.Init((dir + ini_dir).c_str());

    ProcessINI();

    config_.odometry_type = radar_odometry::pipeline::OdometryType(cfg_i_odometry_type_);
    config_.icp_type = radar_odometry::IcpType(cfg_i_icp_type_);

    odometry_ = radar_odometry::pipeline::RadarOdometry(config_);   
}

void VodEvaluation::ProcessINI()
{
    if (v_ini_parser_.IsFileUpdated()){
        NodeHandle nh;

        if ( v_ini_parser_.ParseConfig("radar_odometry", "odometry_type", cfg_i_odometry_type_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/odometry_type");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "icp_type", cfg_i_icp_type_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/icp_type");
        }

        if ( v_ini_parser_.ParseConfig("radar_odometry", "ego_to_radar_x_m", config_.ego_to_radar_x_m) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/ego_to_radar_x_m");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "ego_to_radar_y_m", config_.ego_to_radar_y_m) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/ego_to_radar_y_m");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "ego_to_radar_z_m", config_.ego_to_radar_z_m) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/ego_to_radar_z_m");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "ego_to_radar_roll_deg", config_.ego_to_radar_roll_deg) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/ego_to_radar_roll_deg");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "ego_to_radar_pitch_deg", config_.ego_to_radar_pitch_deg) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/ego_to_radar_pitch_deg");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "ego_to_radar_yaw_deg", config_.ego_to_radar_yaw_deg) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/ego_to_radar_yaw_deg");
        }

        if ( v_ini_parser_.ParseConfig("radar_odometry", "voxel_size", config_.voxel_size) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/voxel_size");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "max_range", config_.max_range) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/max_range");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "min_range", config_.min_range) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/min_range");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "max_points_per_voxel", config_.max_points_per_voxel) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/max_points_per_voxel");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "local_map_time_th", config_.local_map_time_th) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/local_map_time_th");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "min_motion_th", config_.min_motion_th) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/min_motion_th");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "initial_trans_threshold", config_.initial_trans_threshold) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/initial_trans_threshold");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "initial_vel_threshold", config_.initial_vel_threshold) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/initial_vel_threshold");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "doppler_vel_margin", config_.doppler_vel_margin) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/doppler_vel_margin");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "icp_3dof", config_.icp_3dof) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/icp_3dof");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "icp_doppler", config_.icp_doppler) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/icp_doppler");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "doppler_gm_th", config_.doppler_gm_th) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/doppler_gm_th");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "use_rcs_weight", config_.use_rcs_weight) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/use_rcs_weight");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "doppler_trans_lambda", config_.doppler_trans_lambda) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/doppler_trans_lambda");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "icp_min_point_num", config_.icp_min_point_num) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/icp_min_point_num");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "lm_lambda", config_.lm_lambda) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/lm_lambda");
        }

        if ( v_ini_parser_.ParseConfig("radar_odometry", "range_variance_m", config_.range_variance_m) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/range_variance_m");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "azimuth_variance_deg", config_.azimuth_variance_deg) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/azimuth_variance_deg");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "elevation_variance_deg", config_.elevation_variance_deg) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/elevation_variance_deg");
        }

        if ( v_ini_parser_.ParseConfig("radar_odometry", "gicp_max_point", config_.gicp_max_point) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/gicp_max_point");
        }

        if ( v_ini_parser_.ParseConfig("radar_odometry", "output_static_map", config_.output_static_map) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/output_static_map");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "output_map_max_range", config_.output_map_max_range) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/output_map_max_range");
        }

        if ( v_ini_parser_.ParseConfig("radar_odometry", "vod_dt", vod_dt_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/vod_dt_");
        }

        ROS_WARN("RadarOdometryNode: INI Updated!");
    }
}

void VodEvaluation::RunRadarOdometry(RadarDataStruct i_radar_struct)
{   
    std::chrono::system_clock::time_point run_radar_odometry_start_time = std::chrono::system_clock::now();
    int i_point_cloud_num = i_radar_struct.points.size();
    ROS_INFO_STREAM("RadarOdometryNode: Input radar points num: " << i_point_cloud_num);

    const auto &[frame, frame_global] =  odometry_.RegisterPoints(i_radar_struct.points, i_radar_struct.timestamp);
    radar_pose_ = odometry_.poses().back();

    std::chrono::duration<double>run_radar_odometry_time_sec = std::chrono::system_clock::now() - run_radar_odometry_start_time;

    ROS_INFO_STREAM("RadarOdometryNode: Total Time sec:  " << run_radar_odometry_time_sec.count());


}


void VodEvaluation::ProcessRadarFiles()
{
    std::cout<<"[ProcessRadarFiles] Started"<<std::endl;
    Init();
    std::cout<<"[ProcessRadarFiles] Init done"<<std::endl;

    //

    ros::NodeHandle nh("~");

    bool b_eval_whold_seq;
    std::string vod_seq_folder_path;
    std::string vod_eval_folder_path;
    bool b_debug_mode;

    nh.param<bool>("/radar_odometry_ros/b_eval_whold_seq", b_eval_whold_seq, false);
    nh.param<std::string>("/radar_odometry_ros/vod_seq_folder_path", vod_seq_folder_path, 
                        "/mnt/d/dataset/view_of_delft_PUBLIC/radar_odom_for_each_seq/03/");
    nh.param<std::string>("/radar_odometry_ros/vod_eval_folder_path", vod_eval_folder_path, 
                        "/mnt/d/dataset/view_of_delft_PUBLIC/radar4motion_eval_results/");
    nh.param<bool>("/radar_odometry_ros/b_debug_mode", b_debug_mode, true);

    rospub_vod_radar_points_ = nh.advertise<sensor_msgs::PointCloud2>("/vod_radar_points", 10);
    rospub_odom_accumulated_map_ = nh.advertise<sensor_msgs::PointCloud2>("/odom_accumulated_map", 1);

    // For Evo Bag evaluation. PoseStamped
    rospub_novatel_ref_pose_stamped_ = nh.advertise<geometry_msgs::PoseStamped>("/eval_ref", 10);
    rospub_radar_odom_pose_stamped_ = nh.advertise<geometry_msgs::PoseStamped>("/radar_odom_pose_stamped", 10);
    rospub_radar_odom_eval_pose_stamped_ = nh.advertise<geometry_msgs::PoseStamped>("/eval_radar_odom", 10);

    // Rviz PoseArray Visualization
    rospub_gt_odom_pose_ = nh.advertise<geometry_msgs::PoseArray>("/gt_odom_pose", 10);
    rospub_radar_odom_pose_ = nh.advertise<geometry_msgs::PoseArray>("/radar_odom_pose", 10);

    // TODO: make config
    vod_cam2radar_mat_ <<   -0.013857, -0.9997468, 0.01772762, 0.05283124,
                            0.10934269, -0.01913807, -0.99381983, 0.98100483,
                            0.99390751, -0.01183297, 0.1095802, 1.44445002,
                            0.0, 0.0, 0.0, 1.0;
    result_tf_ = Eigen::Affine3d::Identity();

    // Read whole folder root in vod_seq_folder_path
    if (b_eval_whold_seq)
    {
        std::vector<std::string> vod_seq_folders;
        for (const auto& entry : std::filesystem::directory_iterator(vod_seq_folder_path)) {
            if (entry.is_directory()) {
                std::cout << entry.path().filename() << std::endl;
                vod_seq_folders.push_back(entry.path().filename());
            }
        }

        for (const auto& seq_folder : vod_seq_folders)
        {
            std::string seq_folder_str = vod_seq_folder_path + seq_folder;
            ProcessRadarFile(seq_folder_str, b_debug_mode, vod_eval_folder_path);

            // Init class
            Init();
        }
    }
    else // only one folder
    {   
        ProcessRadarFile(vod_seq_folder_path, b_debug_mode, vod_eval_folder_path);
    }
}

void VodEvaluation::ProcessRadarFile(std::string& vod_seq_folder_path, bool b_debug_mode, std::string& vod_eval_folder_path)
{
    std::cout<<"[ProcessRadarFile] Started"<<std::endl;
    
    std::string pose_directory_path = vod_seq_folder_path + "/pose";
    std::string radar_directory_path = vod_seq_folder_path + "/velodyne";

    // VoD
    Eigen::Affine3d cam2radar_transform(vod_cam2radar_mat_);

    // Get all .json files in the pose directory
    std::vector<std::filesystem::path> pose_files;
    GetPoseJsonFiles(pose_directory_path, pose_files);

    // Parse pose JSON files
    std::vector<Eigen::Affine3d> gt_odom2radar_transforms;
    ParsePoseJsonFiles(pose_files, cam2radar_transform, gt_odom2radar_transforms);
    std::cout << "[ProcessRadarFile] JSON File Read Complete. Num: "<<gt_odom2radar_transforms.size() << std::endl;

    // Ego motion-only estimation
    std::vector<Eigen::Affine3d> ego_motion_only_transforms;
    
    // Get all .bin files in the radar directory
    std::vector<std::filesystem::path> radar_files;
    GetRadarFiles(radar_directory_path, radar_files);

    std::cout << "[ProcessRadarFile] Bin File Read Complete. Num: "<< radar_files.size() << std::endl;

    // Read radar files
    std::vector<pcl::PointCloud<RadarPoint>::Ptr> clouds;
    std::vector<Eigen::Affine3d> estimate_odom2radar_transforms;
    for (int idx = 0; idx < radar_files.size(); ++idx) {
        const auto& file_path = radar_files[idx];
        std::ifstream file(file_path, std::ios::binary);

        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << file_path << std::endl;
            continue;
        }

        std::cout<<"[ProcessRadarFile] Processing Bin index: "<<idx<<std::endl;

        // Convert the .bin file to a point cloud
        // pcl::PointCloud<RadarPoint>::Ptr cloud (new pcl::PointCloud<RadarPoint>);
        std::vector<SRadarPoint> radar_points;
        ConvertBin2PointCloud(file, radar_points, idx);
        // clouds.push_back(cloud);

        // Publish GT for evaluation
        Eigen::Affine3d odom2radar = gt_odom2radar_transforms[idx];
        pubVodGtPose(odom2radar);

        // Run odometry for each .bin files
        RunRadarOdometry( i_vod_radar_struct_);

        Eigen::Affine3d result_tf(radar_pose_);
        estimate_odom2radar_transforms.push_back(result_tf);
        // ego_motion_only_transforms.push_back(result_ego_motion_estimation_);

        // Transform tf from Map to radar
        BroadcastTFAndVisualizeOdomPose(result_tf);

        std_msgs::Header local_map_header;
        local_map_header.stamp = ros::Time( i_vod_radar_struct_.timestamp);
        local_map_header.frame_id = "Map";
        // Visualize Local Map
        if(config_.output_static_map == true){
            o_cur_radar_global_cloud_ = *SRadarPointToPointCloud2(odometry_.StaticLocalMap(), local_map_header);
        }else{
            o_cur_radar_global_cloud_ = *SRadarPointToPointCloud2(odometry_.LocalMap(), local_map_header);
        }

        rospub_odom_accumulated_map_.publish(o_cur_radar_global_cloud_);

        // Wait until key input
        if (b_debug_mode)
        {
            std::cout << "Press any key to continue..." << std::endl;
            // std::cin.get();
            std::string input;
            std::cin >> input;

            if (input == "q" || input == "Q") {
                break;
            }
        }

        // ros spin 20hz
        if(ros::ok())
        {
            ros::Rate loop_rate(100);
            loop_rate.sleep();
        }
        else{
            break;
        }

        file.close();
    }   

    std::cout<<"[ProcessRadarFile] Radar Odom Loop Done"<<std::endl;

    // save as txt file
    std::string curr_folder_seq = vod_eval_folder_path + vod_seq_folder_path.substr(vod_seq_folder_path.size() - 2);

    savePosesToFile(estimate_odom2radar_transforms, curr_folder_seq + "_estimate_poses.txt");
    // savePosesToFile(ego_motion_only_transforms, curr_folder_seq + "_em_only.txt");
    savePosesToFile(gt_odom2radar_transforms, curr_folder_seq + "_gt_poses.txt");

}

void VodEvaluation::savePosesToFile(const std::vector<Eigen::Affine3d>& poses, const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    for (const auto& pose : poses) {
        Eigen::Matrix4d matrix = pose.matrix();
        // Write the first three rows of the 4x4 matrix
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col) {
                file << matrix(row, col);
                if (row != 2 || col != 3) {
                    file << " ";
                }
            }
        }
        file << "\n";
    }

    file.close();
}

void VodEvaluation::GetPoseJsonFiles(std::string& pose_directory_path, std::vector<std::filesystem::path>& pose_files)
{
    pose_files.clear();
    for (const auto& entry : std::filesystem::directory_iterator(pose_directory_path)) {
        if (entry.path().extension() == ".json") {
            pose_files.push_back(entry.path());
        }
    }

    // Sort the files by their sequence number
    std::sort(pose_files.begin(), pose_files.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
        return std::stoi(a.stem()) < std::stoi(b.stem());
    });
}

void VodEvaluation::GetRadarFiles(std::string& radar_directory_path, std::vector<std::filesystem::path>& radar_files)
{
    radar_files.clear();
    
    for (const auto& entry : std::filesystem::directory_iterator(radar_directory_path)) {
        if (entry.path().extension() == ".bin") {
            radar_files.push_back(entry.path());
        }
    }

    // Sort the files by their sequence number
    std::sort(radar_files.begin(), radar_files.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
        return std::stoi(a.stem()) < std::stoi(b.stem());
    });
}

void VodEvaluation::ParsePoseJsonFiles(const std::vector<std::filesystem::path>& pose_files, const Eigen::Affine3d& cam2radar_tfrom, std::vector<Eigen::Affine3d>& odom2radar_transforms)
{
    odom2radar_transforms.clear();
    // odom2radar_transforms.reserve(pose_files.size());

    bool b_first_pose = true;
    Eigen::Affine3d first_odom2radar;

    for (const auto& file_path : pose_files) {
        std::ifstream file(file_path);

        // 파일 잘 열리는지 확인
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << file_path << std::endl;
            continue;
        }

        // Read the JSON file
        nlohmann::json json_data;
        try {
            file >> json_data;
            file.close();
        } catch (const nlohmann::json::parse_error& ex) {
            std::cerr << "Parse error at byte "<< ex.byte << ": " << ex.what() << std::endl;
            continue;
        }

        // Checking and logging for each key
        if (json_data.contains("odomToCamera") && json_data["odomToCamera"].is_array()) {
            Eigen::Affine3d odom2radar = jsonToAffine3d(json_data["odomToCamera"]) * cam2radar_tfrom;

            if (b_first_pose)
            {
                first_odom2radar = odom2radar;
                b_first_pose = false;
            }
            odom2radar_transforms.push_back(first_odom2radar.inverse() * odom2radar);
        } else {
            std::cerr << "Missing or invalid 'odomToCamera' data in file: " << file_path << std::endl;
        }
    }
}

void VodEvaluation::ConvertBin2PointCloud(std::ifstream& bin_file, std::vector<SRadarPoint>& o_s_radar_points, int time_idx)
{
    o_s_radar_points.clear();
    std::vector<SRadarPoint> points;
    

    VodRadarPointType vod_point;
    while (bin_file.read((char*)&vod_point, sizeof(VodRadarPointType))) {
        // RadarPoint radar_point;
        // radar_point.x = vod_point.x;
        // radar_point.y = vod_point.y;
        // radar_point.z = vod_point.z;
        // radar_point.vel = vod_point.v_r;
        // radar_point.RCS = vod_point.RCS;

        if (std::isnan(vod_point.x) || std::isnan(vod_point.y) || std::isnan(vod_point.z) ||
            std::isnan(vod_point.v_r) || std::isnan(vod_point.RCS)) {
            continue; // 유효하지 않은 값이 있으면 스킵
        }

        SRadarPoint iter_point;
        iter_point.pose.x() = vod_point.x;
        iter_point.pose.y() = vod_point.y;
        iter_point.pose.z() = vod_point.z;
        iter_point.local = iter_point.pose;

        iter_point.power = vod_point.RCS;
        iter_point.rcs = vod_point.RCS;
        iter_point.range = iter_point.pose.norm();
        iter_point.vel = vod_point.v_r;
        iter_point.azi_angle = atan2(iter_point.pose.y(),iter_point.pose.x()) * 180.0/M_PI;
        iter_point.ele_angle = atan2(iter_point.pose.z(),
                                sqrt(iter_point.pose.x()*iter_point.pose.x() +
                                    iter_point.pose.y()*iter_point.pose.y())) * 180.0/M_PI;
        iter_point.timestamp = vod_dt_*time_idx;
            
        points.push_back(iter_point);

    }

    // Publishing Current Reading Points
    sensor_msgs::PointCloud2 output;
    std_msgs::Header header;
    header.stamp = ros::Time(vod_dt_*time_idx);
    header.frame_id = "radar";
    output = *SRadarPointToPointCloud2(points, header);
    rospub_vod_radar_points_.publish(output);

    
    o_s_radar_points = points;

    i_vod_radar_struct_.timestamp = vod_dt_*time_idx;
    i_vod_radar_struct_.points = points;
    i_vod_radar_struct_.frame_id = "radar";

}

Eigen::Affine3d VodEvaluation::jsonToAffine3d(const nlohmann::json& json)
{
    if (json.is_null()) {
        // tsfm이 null이면, 단위 변환(Identity)을 반환
        std::cerr<<"jsonToAffine3d: json is null"<<std::endl;
        return Eigen::Affine3d::Identity();
    } else {
        // tsfm이 null이 아니면, 주어진 tsfm으로부터 변환 행렬을 생성
        Eigen::Matrix4d mat;
        // JSON 배열에서 값을 읽기 전에 해당 키를 확인
        if (json.is_array()) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    mat(i, j) = json[i * 4 + j];  // JSON 배열에서 순차적으로 접근
                }
            }
        } else {
            throw std::runtime_error("Expected a JSON array for transformation matrix.");
        }
        return Eigen::Affine3d(mat);
    }
}

void VodEvaluation::pubVodGtPose(const Eigen::Affine3d& vod_gt_pose)
{
    static geometry_msgs::PoseArray vod_gt_arr;

    geometry_msgs::Pose vod_gt_pose_msg;
    vod_gt_pose_msg.position.x = vod_gt_pose.translation().x();
    vod_gt_pose_msg.position.y = vod_gt_pose.translation().y();
    vod_gt_pose_msg.position.z = vod_gt_pose.translation().z();

    // Convert rotation matrix to quaternion
    Eigen::Quaterniond quat(vod_gt_pose.rotation());
    vod_gt_pose_msg.orientation.x = quat.x();
    vod_gt_pose_msg.orientation.y = quat.y();
    vod_gt_pose_msg.orientation.z = quat.z();
    vod_gt_pose_msg.orientation.w = quat.w();
    
    vod_gt_arr.header.frame_id = "Map";
    // vod_gt_arr.header.stamp = i_point_cloud_.header.stamp;
    vod_gt_arr.header.stamp = ros::Time( i_vod_radar_struct_.timestamp);
    vod_gt_arr.poses.push_back(vod_gt_pose_msg);

    // For rviz visualization
    rospub_gt_odom_pose_.publish(vod_gt_arr);

    // For evo eval tool
    novatel_pose_stamped_.header.frame_id = "Map";
    // novatel_pose_stamped_.header.stamp = i_point_cloud_.header.stamp;
    novatel_pose_stamped_.header.stamp = ros::Time( i_vod_radar_struct_.timestamp);
    novatel_pose_stamped_.pose = vod_gt_pose_msg;
    rospub_novatel_ref_pose_stamped_.publish(novatel_pose_stamped_);
}

void VodEvaluation::BroadcastTFAndVisualizeOdomPose(const Eigen::Affine3d& result_tf_in_ego_pose)
{
    double odom_x, odom_y, odom_z, odom_roll, odom_pitch, odom_yaw = 0.0;
    pcl::getTranslationAndEulerAngles (result_tf_in_ego_pose, odom_x, odom_y, odom_z, odom_roll, odom_pitch, odom_yaw);

    // KUSV to Radar
    tf::Quaternion rpy2quat_odom;
    rpy2quat_odom.setRPY( odom_roll, odom_pitch, odom_yaw );
    tf::Vector3 tfvec_offset_odom = tf::Vector3(odom_x, odom_y, odom_z);

    static tf::TransformListener listener;
    static tf::TransformBroadcaster tfbroad_estimated_pose_odom;
    
    tf::Transform transform_radar_to_map = tf::Transform(rpy2quat_odom, tfvec_offset_odom);

    
    // Map to Radar
    tfbroad_estimated_pose_odom.sendTransform( 
        tf::StampedTransform(transform_radar_to_map, ros::Time( i_vod_radar_struct_.timestamp), "Map", "radar"));
    
    // Visualziation
    VisualizeAndPubOdomPose(odom_x, odom_y, odom_z, rpy2quat_odom);
}

void VodEvaluation::VisualizeAndPubOdomPose(double odom_x, double odom_y, double odom_z, tf::Quaternion odom_rpy)
{
    // static: For publishing acculumated pose arrays
    static geometry_msgs::PoseArray pos_arr_odom;
    geometry_msgs::PoseStamped odom_pose_stamped;

    geometry_msgs::Pose odom_pose;
    odom_pose.position.x = odom_x;
    odom_pose.position.y = odom_y;
    odom_pose.position.z = odom_z;

    odom_pose.orientation.x = odom_rpy.getX();
    odom_pose.orientation.y = odom_rpy.getY();
    odom_pose.orientation.z = odom_rpy.getZ();
    odom_pose.orientation.w = odom_rpy.getW();

    odom_pose_stamped.pose = odom_pose;

    // odom_pose_stamped.header.stamp = i_point_cloud_.header.stamp;
    odom_pose_stamped.header.stamp = ros::Time( i_vod_radar_struct_.timestamp);
    
    pos_arr_odom.header.frame_id = "Map";
    pos_arr_odom.poses.push_back(odom_pose);

    rospub_radar_odom_pose_stamped_.publish(odom_pose_stamped);
    rospub_radar_odom_pose_.publish(pos_arr_odom);

    // radar_odom_pose_stamped_.header = i_point_cloud_.header;
    radar_odom_pose_stamped_.header.stamp = ros::Time( i_vod_radar_struct_.timestamp);
    radar_odom_pose_stamped_.header.frame_id = "radar";
    radar_odom_pose_stamped_.pose = odom_pose;

    rospub_radar_odom_eval_pose_stamped_.publish(radar_odom_pose_stamped_);
}
