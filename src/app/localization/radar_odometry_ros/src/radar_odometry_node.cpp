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

        if ( v_ini_parser_.ParseConfig("radar_odometry", "vod_dt", vod_dt_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/vod_dt_");
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

    if(config_.output_static_map == true){
        o_cur_radar_global_cloud_ = *SRadarPointToPointCloud2(odometry_.StaticLocalMap(), local_map_header);
    }else{
        o_cur_radar_global_cloud_ = *SRadarPointToPointCloud2(odometry_.LocalMap(), local_map_header);
    }

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
        // main_task.ProcessRadarFiles();
        main_task.vod_evaluation_.ProcessRadarFiles();
        std::cout<<"HEllo"<<std::endl;

    }else{
        
        main_task.Exec();
    }



    return 0;
}