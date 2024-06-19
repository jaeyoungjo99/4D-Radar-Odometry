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

    static_radar_ptr_.reset(new pcl::PointCloud<PointXYZPRVAE>());

    radar_pose_ = Eigen::Matrix4f::Identity();
    last_radar_pose_ = Eigen::Matrix4f::Identity();
    radar_calib_pose_ = Eigen::Matrix4f::Identity();
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

    s_point_cloud_ = nh.subscribe(cfg_str_sensor_topic_pc1_, 10, &RadarOdometryNode::CallbackPointCloud, this, ros::TransportHints().tcpNoDelay());    
    s_point_cloud2_ = nh.subscribe(cfg_str_sensor_topic_pc2_, 10, &RadarOdometryNode::CallbackPointCloud2, this, ros::TransportHints().tcpNoDelay());                               
    s_can_ = nh.subscribe(cfg_str_can_topic_, 10, &RadarOdometryNode::CallbackCAN, this, ros::TransportHints().tcpNoDelay());    
    s_imu_ = nh.subscribe(cfg_str_imu_topic_, 10, &RadarOdometryNode::CallbackIMU, this, ros::TransportHints().tcpNoDelay());    

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

    config_.ego_to_radar_x_m = cfg_d_ego_to_radar_x_m_;
    config_.ego_to_radar_yaw_deg = cfg_d_ego_to_radar_yaw_deg_;
    config_.odometry_type = radar_odometry::pipeline::OdometryType(cfg_i_odometry_type_);

    odometry_ = radar_odometry::pipeline::RadarOdometry(config_);
    
}

void RadarOdometryNode::Run()
{   
    if(b_is_new_point_cloud_ == false) return;

    ROS_INFO("RadarOdometryNode: Run");

    LidarDataStruct cur_lidar_struct;
    RadarDataStruct cur_radar_struct;
    CanStruct cur_can_struct;

    if(cfg_str_sensor_type_ == "lidar"){
        std::lock_guard<std::mutex> lock(mutex_main_point_cloud_);
        cur_lidar_struct = i_lidar_struct_;
        int i_point_cloud_num = cur_lidar_struct.points->size();
        ROS_INFO_STREAM("RadarOdometryNode: Input liadr points num: " << i_point_cloud_num);
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
        ROS_INFO("RadarOdometryNode: RunRadarOdometry");

        RunRadarOdometry(cur_radar_struct);
    }

    b_is_new_point_cloud_ = false;
    b_is_new_publish_ = true;
}

void RadarOdometryNode::Publish()
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

        ROS_WARN("RadarOdometryNode: INI Updated!");
    }
}


void RadarOdometryNode::RunRadarOdometry(RadarDataStruct i_radar_struct)
{
    int i_point_cloud_num = i_radar_struct.points->size();
    ROS_INFO_STREAM("RadarOdometryNode: Input radar points num: " << i_point_cloud_num);

    o_vel_comp_radar_ptr_->points.clear();
    static_radar_ptr_->points.clear();

    bool success =  odometry_.RegisterPoints(i_radar_struct.points, i_radar_struct.timestamp);
    radar_pose_ = odometry_.poses().back();

    pcl::transformPointCloud(*i_radar_struct.points, *o_cur_radar_global_ptr_,  radar_pose_ * radar_calib_pose_);

    // // Radar Vel arrow
    // o_radar_vel_heading_markers_.markers.clear();

    // //
    // visualization_msgs::Marker est_radar_vel_heading_marker;
    // est_radar_vel_heading_marker.header.frame_id = "afi910";
    // est_radar_vel_heading_marker.header.stamp = ros::Time::now();
    // est_radar_vel_heading_marker.ns = "est_radar_vel_heading";

    // est_radar_vel_heading_marker.id = 1;
    // est_radar_vel_heading_marker.type = visualization_msgs::Marker::ARROW;
    // est_radar_vel_heading_marker.action = visualization_msgs::Marker::ADD;

    // est_radar_vel_heading_marker.pose.position.x = 0.0;
    // est_radar_vel_heading_marker.pose.position.y = 0.0;
    // est_radar_vel_heading_marker.pose.position.z = 0.0;

    // tf::Quaternion est_quat = tf::createQuaternionFromYaw(d_est_azim); // Y축 회전을 기준으로 화살표 방향 설정
    // est_radar_vel_heading_marker.pose.orientation.x = est_quat.x();
    // est_radar_vel_heading_marker.pose.orientation.y = est_quat.y();
    // est_radar_vel_heading_marker.pose.orientation.z = est_quat.z();
    // est_radar_vel_heading_marker.pose.orientation.w = est_quat.w();

    // // 화살표의 크기 설정
    // est_radar_vel_heading_marker.scale.x = abs(d_est_vel);  // 화살표의 길이
    // est_radar_vel_heading_marker.scale.y = 0.2;  // 화살표의 샤프트 직경
    // est_radar_vel_heading_marker.scale.z = 0.2;  // 화살표의 헤드 직경

    // // 화살표의 색상 설정
    // est_radar_vel_heading_marker.color.r = 0.0f;
    // est_radar_vel_heading_marker.color.g = 0.5f;
    // est_radar_vel_heading_marker.color.b = 1.0f;
    // est_radar_vel_heading_marker.color.a = 1.0;

    // o_radar_vel_heading_markers_.markers.push_back(est_radar_vel_heading_marker);
}

double RadarOdometryNode::GetEgoMotionCompVel(PointXYZPRVAE i_radar_point, CanStruct i_can_struct)
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

int main(int argc, char** argv) {
    std::string node_name = "radar_odometry_ros";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");

    int id = 0;

    double period = 0.01;

    ROS_INFO("Complete to get parameters! (ID: %d, Period: %.3f)", id, period);

    RadarOdometryNode main_task(id, node_name, period);
    main_task.Exec();

    return 0;
}