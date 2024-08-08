/****************************************************************************/
// Module:      radar_odometry_node.hpp
// Description: radar_odometry_node
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      Jul 24, 2023: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX - 
/****************************************************************************/

#include "radar_odometry_node.hpp"


RadarOdometryNode::RadarOdometryNode(int id, std::string task_node, double period)
: AtomTask(id, task_node, period)
{
    i_main_lidar_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    i_main_radar_ptr_.reset(new pcl::PointCloud<PointXYZPRVAE>());
    temp_radar_ptr_.reset(new pcl::PointCloud<PointXYZPRVAE>());

    o_vel_comp_radar_ptr_.reset(new pcl::PointCloud<PointXYZPRVAE>());
    o_cur_radar_global_ptr_.reset(new pcl::PointCloud<PointXYZPRVAE>());

    radar_pose_ = Eigen::Matrix4d::Identity();
    last_radar_pose_ = Eigen::Matrix4d::Identity();
    d_last_radar_time_sec_ = 0.0f;
}

RadarOdometryNode::~RadarOdometryNode(){}

void RadarOdometryNode::Init()
{
    NodeHandle nh;
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/perception.ini");
    v_ini_parser_.Init((dir + ini_dir).c_str());

    ProcessINI();

    // s_point_cloud_ = nh.subscribe(cfg_str_sensor_topic_pc1_, 10, &RadarOdometryNode::CallbackPointCloud, this, ros::TransportHints().tcpNoDelay());    
    s_point_cloud2_ = nh.subscribe(cfg_str_sensor_topic_pc2_, 10, &RadarOdometryNode::CallbackPointCloud2, this, ros::TransportHints().tcpNoDelay());                               
    s_can_ = nh.subscribe(cfg_str_can_topic_, 10, &RadarOdometryNode::CallbackCAN, this, ros::TransportHints().tcpNoDelay());    
    s_imu_ = nh.subscribe(cfg_str_imu_topic_, 10, &RadarOdometryNode::CallbackIMU, this, ros::TransportHints().tcpNoDelay());    

    p_vel_comp_radar_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("vel_comp_radar_cloud", 100);
    p_cur_radar_global_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("cur_radar_global_cloud", 100);
    p_radar_vel_heading_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>("radar_vel_heading_markers", 10);
    p_odom_ = nh.advertise<nav_msgs::Odometry>("radar_odom", 10);


    // #ifdef USE_TBB
    //     ROS_INFO("Building with TBB support");
    // #else
    //     ROS_INFO("Building without TBB support");
    // #endif

    config_.odometry_type = radar_odometry::pipeline::OdometryType(cfg_i_odometry_type_);
    config_.icp_type = radar_odometry::IcpType(cfg_i_icp_type_);

    odometry_ = radar_odometry::pipeline::RadarOdometry(config_);
    
}

void RadarOdometryNode::Run()
{   
    if(b_is_new_point_cloud_ == false) return;
    std::cout<<" "<<std::endl;
    ROS_INFO("RadarOdometryNode: Run");

    RadarDataStruct cur_radar_struct;
    CanStruct cur_can_struct;


    {
        std::lock_guard<std::mutex> lock(mutex_main_point_cloud_);
        cur_radar_struct = i_radar_struct_;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_can_);
        cur_can_struct = i_can_struct_;
    }

    if(cfg_str_sensor_type_=="lidar"){

    }
    else{
        ROS_INFO("RadarOdometryNode: RunRadarOdometry");

        RunRadarOdometry(cur_radar_struct);
    }
    

    b_is_new_point_cloud_ = false;
}

void RadarOdometryNode::Publish()
{
    p_vel_comp_radar_cloud_.publish(o_vel_comp_radar_cloud_);
    p_cur_radar_global_cloud_.publish(o_cur_radar_global_cloud_);
    p_radar_vel_heading_marker_array_.publish(o_radar_vel_heading_markers_);

    //
    Eigen::Matrix4d calibrated_radar_pose = radar_pose_;

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "afi910";

    // 포즈 설정
    odom.pose.pose.position.x = calibrated_radar_pose(0, 3);
    odom.pose.pose.position.y = calibrated_radar_pose(1, 3);
    odom.pose.pose.position.z = calibrated_radar_pose(2, 3);

    // Extract rotation matrix
    Eigen::Matrix3d rotation_matrix = calibrated_radar_pose.block<3, 3>(0, 0);
    Eigen::Quaterniond quaternion(rotation_matrix);

    // Assign quaternion to odom message
    odom.pose.pose.orientation.x = quaternion.x();
    odom.pose.pose.orientation.y = quaternion.y();
    odom.pose.pose.orientation.z = quaternion.z();
    odom.pose.pose.orientation.w = quaternion.w();

    // If you have velocity information, set the twist part of the odom message
    // Example velocities (replace with actual values)
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    p_odom_.publish(odom);
}

void RadarOdometryNode::Terminate()
{

}

void RadarOdometryNode::ProcessINI()
{
    if (v_ini_parser_.IsFileUpdated()){
        NodeHandle nh;

        if ( v_ini_parser_.ParseConfig("radar_odometry", "sensor_type", cfg_str_sensor_type_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/sensor_type");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "sensor_topic_pc1", cfg_str_sensor_topic_pc1_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/sensor_topic_pc1");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "sensor_topic_pc2", cfg_str_sensor_topic_pc2_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/sensor_topic_pc2");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "can_topic", cfg_str_can_topic_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/can_topic");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "imu_topic", cfg_str_imu_topic_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/imu_topic");
        }

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
        if ( v_ini_parser_.ParseConfig("radar_odometry", "radar_delay_sec", cfg_d_radar_delay_sec_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/radar_delay_sec_");
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

        ROS_WARN("RadarOdometryNode: INI Updated!");
    }
}


void RadarOdometryNode::RunRadarOdometry(RadarDataStruct i_radar_struct)
{   
    std::chrono::system_clock::time_point run_radar_odometry_start_time = std::chrono::system_clock::now();
    int i_point_cloud_num = i_radar_struct.points.size();
    ROS_INFO_STREAM("RadarOdometryNode: Input radar points num: " << i_point_cloud_num);

    o_vel_comp_radar_ptr_->points.clear();

    const auto &[frame, frame_global] =  odometry_.RegisterPoints(i_radar_struct.points, i_radar_struct.timestamp);
    radar_pose_ = odometry_.poses().back();

    std::chrono::duration<double>run_radar_odometry_time_sec = std::chrono::system_clock::now() - run_radar_odometry_start_time;

    ROS_INFO_STREAM("RadarOdometryNode: Total Time sec:  " << run_radar_odometry_time_sec.count());

    auto frame_header = i_point_cloud2_msg_.header;
    frame_header.frame_id = "afi910";
    

    auto local_map_header = i_point_cloud2_msg_.header;
    local_map_header.frame_id = "world";

    o_vel_comp_radar_cloud_ = *SRadarPointToPointCloud2(frame_global, local_map_header);

    // o_cur_radar_global_cloud_ = *SRadarPointToPointCloud2(odometry_.StaticLocalMap(), local_map_header);

    if(config_.output_static_map == true){
        o_cur_radar_global_cloud_ = *SRadarPointToPointCloud2(odometry_.StaticLocalMap(), local_map_header);
    }else{
        o_cur_radar_global_cloud_ = *SRadarPointToPointCloud2(odometry_.LocalMap(), local_map_header);
    }

}

void RadarOdometryNode::ProcessRadarFiles()
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

    nh.getParam("/imaging_radar_odometry/i_save_odom_txt_type_"  , i_save_odom_txt_type_);
    nh.getParam("/radar_odometry_ros/str_base_link_"  , str_base_link_);

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
            // odom_kiss_icp.Init();
        }
    }
    else // only one folder
    {   
        ProcessRadarFile(vod_seq_folder_path, b_debug_mode, vod_eval_folder_path);
    }
}

void RadarOdometryNode::ProcessRadarFile(std::string& vod_seq_folder_path, bool b_debug_mode, std::string& vod_eval_folder_path)
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
        RunRadarOdometry(i_radar_struct_);

        b_is_new_point_cloud_ = false;
        Eigen::Affine3d result_tf(radar_pose_);
        estimate_odom2radar_transforms.push_back(result_tf);
        // ego_motion_only_transforms.push_back(result_ego_motion_estimation_);

        // Transform tf from Map to radar
        BroadcastTFAndVisualizeOdomPose(result_tf);

        std_msgs::Header local_map_header;
        local_map_header.stamp = ros::Time(i_radar_struct_.timestamp);
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

void RadarOdometryNode::savePosesToFile(const std::vector<Eigen::Affine3d>& poses, const std::string& filename)
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

void RadarOdometryNode::GetPoseJsonFiles(std::string& pose_directory_path, std::vector<std::filesystem::path>& pose_files)
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

void RadarOdometryNode::GetRadarFiles(std::string& radar_directory_path, std::vector<std::filesystem::path>& radar_files)
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

void RadarOdometryNode::ParsePoseJsonFiles(const std::vector<std::filesystem::path>& pose_files, const Eigen::Affine3d& cam2radar_tfrom, std::vector<Eigen::Affine3d>& odom2radar_transforms)
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

void RadarOdometryNode::ConvertBin2PointCloud(std::ifstream& bin_file, std::vector<SRadarPoint>& o_s_radar_points, int time_idx)
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
        iter_point.timestamp = 0.1*time_idx;
            
        points.push_back(iter_point);

    }

    // Publishing Current Reading Points
    sensor_msgs::PointCloud2 output;
    std_msgs::Header header;
    header.stamp = ros::Time(0.1*time_idx);
    header.frame_id = "radar";
    output = *SRadarPointToPointCloud2(points, header);
    rospub_vod_radar_points_.publish(output);

    
    o_s_radar_points = points;

    i_radar_struct_.timestamp = 0.1*time_idx;
    i_radar_struct_.points = points;
    i_radar_struct_.frame_id = "radar";

    b_is_new_point_cloud_ = true;
}

Eigen::Affine3d RadarOdometryNode::jsonToAffine3d(const nlohmann::json& json)
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

void RadarOdometryNode::pubVodGtPose(const Eigen::Affine3d& vod_gt_pose)
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
    vod_gt_arr.header.stamp = ros::Time(i_radar_struct_.timestamp);
    vod_gt_arr.poses.push_back(vod_gt_pose_msg);

    // For rviz visualization
    rospub_gt_odom_pose_.publish(vod_gt_arr);

    // For evo eval tool
    novatel_pose_stamped_.header.frame_id = "Map";
    // novatel_pose_stamped_.header.stamp = i_point_cloud_.header.stamp;
    novatel_pose_stamped_.header.stamp = ros::Time(i_radar_struct_.timestamp);
    novatel_pose_stamped_.pose = vod_gt_pose_msg;
    rospub_novatel_ref_pose_stamped_.publish(novatel_pose_stamped_);
}

void RadarOdometryNode::BroadcastTFAndVisualizeOdomPose(const Eigen::Affine3d& result_tf_in_ego_pose)
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

    // if (str_base_link_ != "")
    // {
    //     tf::StampedTransform transform_baselink_to_radar;
    //     tf::Transform transform_baselink_to_map;
    //     try{
    //         listener.lookupTransform(i_point_cloud_.header.frame_id, str_base_link_, ros::Time(0), transform_baselink_to_radar);
    //         transform_baselink_to_map = transform_radar_to_map * transform_baselink_to_radar;
    //     }
    //     catch (tf::TransformException &ex) {
    //         ROS_ERROR("%s",ex.what());
    //     }

    //     // Map to baselink
    //     // tfbroad_estimated_pose_odom.sendTransform( 
    //     //     tf::StampedTransform(transform_baselink_to_map, i_point_cloud_.header.stamp, "Map", str_base_link_));
    // }
    
    // Map to Radar
    tfbroad_estimated_pose_odom.sendTransform( 
        tf::StampedTransform(transform_radar_to_map, ros::Time(i_radar_struct_.timestamp), "Map", "radar"));
    
    // Visualziation
    VisualizeAndPubOdomPose(odom_x, odom_y, odom_z, rpy2quat_odom);
}

void RadarOdometryNode::VisualizeAndPubOdomPose(double odom_x, double odom_y, double odom_z, tf::Quaternion odom_rpy)
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
    odom_pose_stamped.header.stamp = ros::Time(i_radar_struct_.timestamp);
    
    pos_arr_odom.header.frame_id = "Map";
    pos_arr_odom.poses.push_back(odom_pose);

    rospub_radar_odom_pose_stamped_.publish(odom_pose_stamped);
    rospub_radar_odom_pose_.publish(pos_arr_odom);

    // radar_odom_pose_stamped_.header = i_point_cloud_.header;
    radar_odom_pose_stamped_.header.stamp = ros::Time(i_radar_struct_.timestamp);
    radar_odom_pose_stamped_.header.frame_id = "radar";
    radar_odom_pose_stamped_.pose = odom_pose;

    rospub_radar_odom_eval_pose_stamped_.publish(radar_odom_pose_stamped_);
}


int main(int argc, char** argv) {
    std::string node_name = "radar_odometry_ros";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");

    int id = 0;
    double period = 0.01;
    RadarOdometryNode main_task(id, node_name, period);

    ROS_INFO("Complete to get parameters! (ID: %d, Period: %.3f)", id, period);

    bool b_flag_run_vod_dataset = false;
    nh.getParam("/radar_odometry_ros/b_flag_run_vod_dataset", b_flag_run_vod_dataset);

    if(b_flag_run_vod_dataset){
        main_task.ProcessRadarFiles();

    }else{
        
        main_task.Exec();
    }



    return 0;
}