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


RadarOdometry::RadarOdometry(int id, std::string task_node, double period)
: AtomTask(id, task_node, period)
{
    i_main_lidar_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    i_main_radar_ptr_.reset(new pcl::PointCloud<PointXYZPRVAE>());
    temp_radar_ptr_.reset(new pcl::PointCloud<PointXYZPRVAE>());

    o_vel_comp_radar_ptr_.reset(new pcl::PointCloud<PointXYZPRVAE>());
    o_cur_radar_global_ptr_.reset(new pcl::PointCloud<PointXYZPRVAE>());

    static_radar_ptr_.reset(new pcl::PointCloud<PointXYZPRVAE>());

    radar_pose_ = Eigen::Matrix4f::Identity();
    last_radar_pose_ = Eigen::Matrix4f::Identity();
    radar_calib_pose_ = Eigen::Matrix4f::Identity();
    d_last_radar_time_sec_ = 0.0f;
}

RadarOdometry::~RadarOdometry(){}

void RadarOdometry::Init()
{
    NodeHandle nh;
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/perception.ini");
    v_ini_parser_.Init((dir + ini_dir).c_str());

    ProcessINI();

    s_point_cloud_ = nh.subscribe(cfg_str_sensor_topic_pc1_, 10, &RadarOdometry::CallbackPointCloud, this, ros::TransportHints().tcpNoDelay());    
    s_point_cloud2_ = nh.subscribe(cfg_str_sensor_topic_pc2_, 10, &RadarOdometry::CallbackPointCloud2, this, ros::TransportHints().tcpNoDelay());                               
    s_can_ = nh.subscribe(cfg_str_can_topic_, 10, &RadarOdometry::CallbackCAN, this, ros::TransportHints().tcpNoDelay());    
    s_imu_ = nh.subscribe(cfg_str_imu_topic_, 10, &RadarOdometry::CallbackIMU, this, ros::TransportHints().tcpNoDelay());    

    p_vel_comp_radar_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("vel_comp_radar_cloud", 100);
    p_cur_radar_global_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("cur_radar_global_cloud", 100);
    p_radar_vel_heading_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>("radar_vel_heading_markers", 10);
    p_odom_ = nh.advertise<nav_msgs::Odometry>("radar_odom", 10);

    //
    radar_calib_pose_(0, 0) = std::cos(cfg_d_ego_to_radar_yaw_deg_*M_PI/180.0);
    radar_calib_pose_(0, 1) = -std::sin(cfg_d_ego_to_radar_yaw_deg_*M_PI/180.0);
    radar_calib_pose_(1, 0) = std::sin(cfg_d_ego_to_radar_yaw_deg_*M_PI/180.0);
    radar_calib_pose_(1, 1) = std::cos(cfg_d_ego_to_radar_yaw_deg_*M_PI/180.0);

    // #ifdef USE_TBB
    //     ROS_INFO("Building with TBB support");
    // #else
    //     ROS_INFO("Building without TBB support");
    // #endif
    
}

void RadarOdometry::Run()
{   
    if(b_is_new_point_cloud_ == false) return;

    ROS_INFO("RadarOdometry: Run");

    LidarDataStruct cur_lidar_struct;
    RadarDataStruct cur_radar_struct;
    CanStruct cur_can_struct;

    if(cfg_str_sensor_type_ == "lidar"){
        std::lock_guard<std::mutex> lock(mutex_main_point_cloud_);
        cur_lidar_struct = i_lidar_struct_;
        int i_point_cloud_num = cur_lidar_struct.points->size();
        ROS_INFO_STREAM("RadarOdometry: Input liadr points num: " << i_point_cloud_num);
    }
    else{
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
        ROS_INFO("RadarOdometry: RunRadarOdometry");

        RunRadarOdometry(cur_radar_struct);
    }

    b_is_new_point_cloud_ = false;
    b_is_new_publish_ = true;
}

void RadarOdometry::Publish()
{
    pcl::toROSMsg(*o_vel_comp_radar_ptr_, o_vel_comp_radar_cloud_);
    o_vel_comp_radar_cloud_.header.stamp = ros::Time::now();
    o_vel_comp_radar_cloud_.header.frame_id = "afi910";

    p_vel_comp_radar_cloud_.publish(o_vel_comp_radar_cloud_);

    // 
    pcl::toROSMsg(*o_cur_radar_global_ptr_, o_cur_radar_global_cloud_);
    o_cur_radar_global_cloud_.header.stamp = ros::Time::now();
    o_cur_radar_global_cloud_.header.frame_id = "world";

    p_cur_radar_global_cloud_.publish(o_cur_radar_global_cloud_);

    //
    p_radar_vel_heading_marker_array_.publish(o_radar_vel_heading_markers_);

    //
    Eigen::Matrix4f calibrated_radar_pose = radar_pose_ * radar_calib_pose_;
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "afi910";

    // 포즈 설정
    odom.pose.pose.position.x = calibrated_radar_pose(0, 3);
    odom.pose.pose.position.y = calibrated_radar_pose(1, 3);
    odom.pose.pose.position.z = calibrated_radar_pose(2, 3);

    // Extract rotation matrix
    Eigen::Matrix3f rotation_matrix = calibrated_radar_pose.block<3, 3>(0, 0);
    Eigen::Quaternionf quaternion(rotation_matrix);

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

void RadarOdometry::Terminate()
{

}

void RadarOdometry::ProcessINI()
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

        if ( v_ini_parser_.ParseConfig("radar_odometry", "ego_to_radar_x_m", cfg_d_ego_to_radar_x_m_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/ego_to_radar_x_m");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "ego_to_radar_yaw_deg", cfg_d_ego_to_radar_yaw_deg_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/ego_to_radar_yaw_deg");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "radar_delay_sec", cfg_d_radar_delay_sec_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/radar_delay_sec_");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "max_distance_m", cfg_d_max_distance_m_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/max_distance_m");
        }

        ROS_WARN("RadarOdometry: INI Updated!");
    }
}


void RadarOdometry::RunRadarOdometry(RadarDataStruct i_radar_struct)
{
    int i_point_cloud_num = i_radar_struct.points->size();
    ROS_INFO_STREAM("RadarOdometry: Input radar points num: " << i_point_cloud_num);

    o_vel_comp_radar_ptr_->points.clear();
    static_radar_ptr_->points.clear();

    //
    double d_delta_radar_time_sec; 

    if(abs(d_last_radar_time_sec_) < FLT_MIN){
        d_delta_radar_time_sec = 0.0;
    }
    else{
        d_delta_radar_time_sec = i_radar_struct.timestamp - d_last_radar_time_sec_;
    }
    d_last_radar_time_sec_ = i_radar_struct.timestamp;
    

    // double radar_v_lat_can = cfg_d_ego_to_radar_x_m_ * i_can_struct_.yaw_rate_rad;
    // double radar_v_lon_can = i_can_struct_.vel_mps;
    // double radar_alpha_angle_can_rad = atan2(radar_v_lat_can, radar_v_lon_can);
    // double radar_v_total_can = sqrt(radar_v_lat_can*radar_v_lat_can + radar_v_lon_can*radar_v_lon_can);

    // ROS_INFO_STREAM("RadarOdometry: i_can_struct_.yaw_rate_rad: " << i_can_struct_.yaw_rate_rad);
    // ROS_INFO_STREAM("RadarOdometry: radar_alpha_angle_can_rad: " << radar_alpha_angle_can_rad);


    // 1. Points Preprocessing based on CV model
    // pcl::PointCloud<PointXYZPRVAE>::Ptr cv_margin_radar_ptr(new pcl::PointCloud<PointXYZPRVAE>);

    double d_total_vel = 0.0;
    double d_total_comp_vel = 0.0;
    // for (int i = 0; i < i_point_cloud_num; i++)
    // {
    //     PointXYZPRVAE iter_point, comp_point;
    //     iter_point = i_radar_struct.points->points[i];

        // double d_comp_vel_ms = GetEgoMotionCompVel(iter_point, i_can_struct_);

        // std::cout<<"Vel: "<<iter_point.vel<<" Comp Vel: "<<d_comp_vel_ms<<" azim: "<<iter_point.azi_angle<<std::endl;

        // d_total_vel += iter_point.vel;
        // d_total_comp_vel += d_comp_vel_ms;

        // comp_point = iter_point;
        // comp_point.vel = d_comp_vel_ms;

        // o_vel_comp_radar_ptr_->points.push_back(comp_point);

        // if (abs(d_comp_vel_ms) < 1.0){
        //     static_radar_ptr_->points.push_back(iter_point);
        // }
    // }
    
    // 2. RANSAC 
    pcl::PointCloud<PointXYZPRVAE>::Ptr ransac_radar_ptr(new pcl::PointCloud<PointXYZPRVAE>);
    ransac_radar_ptr = RansacFit(i_radar_struct.points, 3, 100);

    // *o_vel_comp_radar_ptr_ = *ransac_radar_ptr;


    // 3. LSQ Fitting
    Eigen::Vector2f est_vel = FitSine(ransac_radar_ptr);

    double d_est_azim = atan2(est_vel[1], est_vel[0]);
    double d_est_vel = sqrt(est_vel[0]*est_vel[0] + est_vel[1]*est_vel[1]);

    // Static Point Generation
    CanStruct est_can_struct;
    est_can_struct.vel_mps = d_est_vel;
    est_can_struct.yaw_rate_rad = d_est_vel * sin(d_est_azim + cfg_d_ego_to_radar_yaw_deg_*M_PI/180.0) / cfg_d_ego_to_radar_x_m_;
    for (int i = 0; i < ransac_radar_ptr->points.size(); i++)
    {
        PointXYZPRVAE iter_point, comp_point;
        iter_point = ransac_radar_ptr->points[i];

        double d_comp_vel_ms = GetEgoMotionCompVel(iter_point, est_can_struct);

        // std::cout<<"Vel: "<<iter_point.vel<<" Comp Vel: "<<d_comp_vel_ms<<" azim: "<<iter_point.azi_angle<<std::endl;

        comp_point = iter_point;
        comp_point.vel = d_comp_vel_ms;

        o_vel_comp_radar_ptr_->points.push_back(comp_point);

        if (abs(d_comp_vel_ms) < 1.0){
            static_radar_ptr_->points.push_back(iter_point);
        }
    }

    // ROS_INFO_STREAM("RadarOdometry: Can vel: " << radar_v_total_can <<" ms, Can Heading "<<  radar_alpha_angle_can_rad * 180.0/M_PI << " deg");
    ROS_INFO_STREAM("RadarOdometry: Est vel: " << d_est_vel <<" ms, Est Heading "<<  d_est_azim * 180.0/M_PI << " deg");

    // double d_avg_vel = d_total_vel / i_point_cloud_num;
    // double d_avg_comp_vel = d_total_comp_vel / i_point_cloud_num;

    // ROS_INFO_STREAM("RadarOdometry: Avg vel: " << d_avg_vel <<" ms, "<<  d_avg_vel * 3.6 << "kph");
    // ROS_INFO_STREAM("RadarOdometry: Com vel: " << d_avg_comp_vel <<" ms, "<<  d_avg_comp_vel * 3.6 << "kph");

    // Radar pose DR with can
    Eigen::Matrix4f delta_pose = Eigen::Matrix4f::Identity();

    // double radar_dx = d_delta_radar_time_sec * radar_v_lon_can;
    // double radar_dy = d_delta_radar_time_sec * radar_v_lat_can;
    // double radar_dyaw = d_delta_radar_time_sec * i_can_struct_.yaw_rate_rad;

    // Radar pose DR with Radar
    double radar_dx = d_delta_radar_time_sec * d_est_vel * cos(d_est_azim + cfg_d_ego_to_radar_yaw_deg_*M_PI/180.0);
    double radar_dy = d_delta_radar_time_sec * d_est_vel * sin(d_est_azim + cfg_d_ego_to_radar_yaw_deg_*M_PI/180.0);
    double radar_dyaw = d_delta_radar_time_sec * d_est_vel * sin(d_est_azim + cfg_d_ego_to_radar_yaw_deg_*M_PI/180.0) / cfg_d_ego_to_radar_x_m_;

    delta_pose(0, 0) = std::cos(radar_dyaw);
    delta_pose(0, 1) = -std::sin(radar_dyaw);
    delta_pose(1, 0) = std::sin(radar_dyaw);
    delta_pose(1, 1) = std::cos(radar_dyaw);

    // Translation part
    delta_pose(0, 3) = radar_dx;
    delta_pose(1, 3) = radar_dy;


    radar_pose_ = radar_pose_ * delta_pose;

    pcl::transformPointCloud(*ransac_radar_ptr, *o_cur_radar_global_ptr_,  radar_pose_ * radar_calib_pose_);

    // Radar Vel arrow
    o_radar_vel_heading_markers_.markers.clear();

    // visualization_msgs::Marker can_radar_vel_heading_marker;
    // can_radar_vel_heading_marker.header.frame_id = "afi910";
    // can_radar_vel_heading_marker.header.stamp = ros::Time::now();
    // can_radar_vel_heading_marker.ns = "can_radar_vel_heading";

    // can_radar_vel_heading_marker.id = 0;
    // can_radar_vel_heading_marker.type = visualization_msgs::Marker::ARROW;
    // can_radar_vel_heading_marker.action = visualization_msgs::Marker::ADD;

    // can_radar_vel_heading_marker.pose.position.x = 0.0;
    // can_radar_vel_heading_marker.pose.position.y = 0.0;
    // can_radar_vel_heading_marker.pose.position.z = 0.0;

    // tf::Quaternion can_quat = tf::createQuaternionFromYaw(radar_alpha_angle_can_rad - cfg_d_ego_to_radar_yaw_deg_*M_PI/180); // Y축 회전을 기준으로 화살표 방향 설정
    // can_radar_vel_heading_marker.pose.orientation.x = can_quat.x();
    // can_radar_vel_heading_marker.pose.orientation.y = can_quat.y();
    // can_radar_vel_heading_marker.pose.orientation.z = can_quat.z();
    // can_radar_vel_heading_marker.pose.orientation.w = can_quat.w();

    // // 화살표의 크기 설정
    // can_radar_vel_heading_marker.scale.x = abs(radar_v_total_can);  // 화살표의 길이
    // can_radar_vel_heading_marker.scale.y = 0.2;  // 화살표의 샤프트 직경
    // can_radar_vel_heading_marker.scale.z = 0.2;  // 화살표의 헤드 직경

    // // 화살표의 색상 설정
    // can_radar_vel_heading_marker.color.r = 0.0f;
    // can_radar_vel_heading_marker.color.g = 1.0f;
    // can_radar_vel_heading_marker.color.b = 0.0f;
    // can_radar_vel_heading_marker.color.a = 1.0;

    // o_radar_vel_heading_markers_.markers.push_back(can_radar_vel_heading_marker);

    //
    visualization_msgs::Marker est_radar_vel_heading_marker;
    est_radar_vel_heading_marker.header.frame_id = "afi910";
    est_radar_vel_heading_marker.header.stamp = ros::Time::now();
    est_radar_vel_heading_marker.ns = "est_radar_vel_heading";

    est_radar_vel_heading_marker.id = 1;
    est_radar_vel_heading_marker.type = visualization_msgs::Marker::ARROW;
    est_radar_vel_heading_marker.action = visualization_msgs::Marker::ADD;

    est_radar_vel_heading_marker.pose.position.x = 0.0;
    est_radar_vel_heading_marker.pose.position.y = 0.0;
    est_radar_vel_heading_marker.pose.position.z = 0.0;

    tf::Quaternion est_quat = tf::createQuaternionFromYaw(d_est_azim); // Y축 회전을 기준으로 화살표 방향 설정
    est_radar_vel_heading_marker.pose.orientation.x = est_quat.x();
    est_radar_vel_heading_marker.pose.orientation.y = est_quat.y();
    est_radar_vel_heading_marker.pose.orientation.z = est_quat.z();
    est_radar_vel_heading_marker.pose.orientation.w = est_quat.w();

    // 화살표의 크기 설정
    est_radar_vel_heading_marker.scale.x = abs(d_est_vel);  // 화살표의 길이
    est_radar_vel_heading_marker.scale.y = 0.2;  // 화살표의 샤프트 직경
    est_radar_vel_heading_marker.scale.z = 0.2;  // 화살표의 헤드 직경

    // 화살표의 색상 설정
    est_radar_vel_heading_marker.color.r = 0.0f;
    est_radar_vel_heading_marker.color.g = 0.5f;
    est_radar_vel_heading_marker.color.b = 1.0f;
    est_radar_vel_heading_marker.color.a = 1.0;

    o_radar_vel_heading_markers_.markers.push_back(est_radar_vel_heading_marker);
}

pcl::PointCloud<PointXYZPRVAE>::Ptr RadarOdometry::CvFiltering(const pcl::PointCloud<PointXYZPRVAE>::Ptr& cloud, const Eigen::Matrix4f mat_prediction){
    pcl::PointCloud<PointXYZPRVAE>::Ptr filtered_cloud(new pcl::PointCloud<PointXYZPRVAE>);



    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    return filtered_cloud;
}

Eigen::Vector3f RadarOdometry::FitQuadratic(const pcl::PointCloud<PointXYZPRVAE>::Ptr& cloud) {
    // Prepare matrices for the least squares problem
    Eigen::MatrixXf A(cloud->points.size(), 3);
    Eigen::VectorXf b(cloud->points.size());

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        float x = cloud->points[i].azi_angle * M_PI/180.0f;
        float y = cloud->points[i].vel;

        A(i, 0) = x * x;
        A(i, 1) = x;
        A(i, 2) = 1.0;
        b(i) = y;
    }

    // Solve the normal equation A^T * A * coeffs = A^T * b
    Eigen::Vector3f coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    
    return coeffs;  // [a, b, c] coefficients
}

Eigen::Vector2f RadarOdometry::FitSine(const pcl::PointCloud<PointXYZPRVAE>::Ptr& cloud){
    Eigen::MatrixXf A(cloud->points.size(), 2);
    Eigen::VectorXf b(cloud->points.size());

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        float x = cloud->points[i].azi_angle * M_PI/180.0f;
        float y = -cloud->points[i].vel;

        A(i, 0) = cos(x);
        A(i, 1) = sin(x);
        b(i) = y;
    }

    // Solve the normal equation A^T * A * coeffs = A^T * b
    Eigen::Vector2f coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    
    return coeffs;  // [vx, vy]
}

Eigen::Vector2f RadarOdometry::FitSine(const pcl::PointCloud<PointXYZPRVAE>::Ptr& cloud, const std::vector<int>& indices) {
    Eigen::MatrixXf A(indices.size(), 2);
    Eigen::VectorXf b(indices.size());

    for (size_t i = 0; i < indices.size(); ++i) {
        float x = cloud->points[indices[i]].azi_angle * M_PI / 180.0f;
        float y = -cloud->points[indices[i]].vel;

        A(i, 0) = cos(x);
        A(i, 1) = sin(x);
        b(i) = y;
    }

    // Solve the normal equation A^T * A * coeffs = A^T * b
    Eigen::Vector2f coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    return coeffs;  // [vx, vy]
}

pcl::PointCloud<PointXYZPRVAE>::Ptr RadarOdometry::RansacFit(const pcl::PointCloud<PointXYZPRVAE>::Ptr& cloud, float margin, int max_iterations) {
    pcl::PointCloud<PointXYZPRVAE>::Ptr inliers(new pcl::PointCloud<PointXYZPRVAE>);
    std::vector<int> best_inliers;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, cloud->points.size() - 1);

    int best_inliers_count = 0;
    Eigen::Vector2f best_coeffs;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Randomly select 2 points
        std::vector<int> sample_indices = {dis(gen), dis(gen)};

        // Fit model to the sample points
        Eigen::Vector2f coeffs = FitSine(cloud, sample_indices);

        // Count inliers
        std::vector<int> current_inliers;
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            float x = cloud->points[i].azi_angle * M_PI / 180.0f;
            float y = -cloud->points[i].vel;
            float predicted_y = coeffs[0] * cos(x) + coeffs[1] * sin(x);

            if (std::abs(predicted_y - y) <= margin) {
                current_inliers.push_back(i);
            }
        }

        // Update the best model if the current one is better
        if (current_inliers.size() > best_inliers_count) {
            best_inliers_count = current_inliers.size();
            best_coeffs = coeffs;
            // inliers->points.clear();
            // for (int idx : current_inliers) {
            //     inliers->points.push_back(cloud->points[idx]);
            // }

            best_inliers = current_inliers;
        }
    }
    
    
    for (int idx : best_inliers) {
        inliers->points.push_back(cloud->points[idx]);
    }
        
    inliers->width = inliers->points.size();
    inliers->height = 1;
    inliers->is_dense = true;

    return inliers;
}

double RadarOdometry::GetEgoMotionCompVel(PointXYZPRVAE i_radar_point, CanStruct i_can_struct)
{
    // Currently Only cal linear vel

    double point_angle_rad = i_radar_point.azi_angle * M_PI/180.0f;

    double radar_v_lat_can = cfg_d_ego_to_radar_x_m_ * i_can_struct.yaw_rate_rad;
    double radar_v_lon_can = i_can_struct.vel_mps;
    double radar_v_total_can = sqrt(radar_v_lat_can*radar_v_lat_can + radar_v_lon_can*radar_v_lon_can);

    double radar_alpha_angle_can_rad = atan2(radar_v_lat_can, radar_v_lon_can);
    double comp_vel = radar_v_total_can * cos(radar_alpha_angle_can_rad - point_angle_rad);

    // double o_comp_vel_ms = i_radar_point.vel/cos(point_angle_rad) + radar_v_lon;

    double o_comp_vel_ms = i_radar_point.vel + comp_vel;

    return o_comp_vel_ms;
}

void RadarOdometry::EstimateEgoMotion(RadarDataStruct i_radar_struct, CanStruct & o_can_struct)
{

}

int main(int argc, char** argv) {
    std::string node_name = "radar_odometry";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");

    int id = 0;

    double period = 0.01;

    ROS_INFO("Complete to get parameters! (ID: %d, Period: %.3f)", id, period);

    RadarOdometry main_task(id, node_name, period);
    main_task.Exec();

    return 0;
}