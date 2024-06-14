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
}

RadarOdometry::~RadarOdometry(){}

void RadarOdometry::Init()
{
    NodeHandle nh;
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/perception.ini");
    v_ini_parser_.Init((dir + ini_dir).c_str());

    ProcessINI();

    s_main_point_cloud_ = nh.subscribe(cfg_str_sensor_topic_, 10, &RadarOdometry::CallbackMainPointCloud, this, ros::TransportHints().tcpNoDelay());                               
    s_can_ = nh.subscribe(cfg_str_can_topic_, 10, &RadarOdometry::CallbackCAN, this, ros::TransportHints().tcpNoDelay());    
    s_imu_ = nh.subscribe(cfg_str_imu_topic_, 10, &RadarOdometry::CallbackIMU, this, ros::TransportHints().tcpNoDelay());    
}

void RadarOdometry::Run()
{   
    if(b_is_new_point_cloud_ == false) return;

    ROS_INFO("RadarOdometry: Run");

    LidarDataStruct cur_lidar_struct;
    RadarDataStruct cur_radar_struct;

    if(cfg_str_sensor_type_ == "lidar"){
        std::lock_guard<std::mutex> lock(mutex_main_point_cloud_);
        cur_lidar_struct = i_lidar_struct_;
        int i_point_cloud_num = cur_lidar_struct.points->size();
        ROS_INFO_STREAM("RadarOdometry: Input liadr points num: " << i_point_cloud_num);
    }
    else{
        std::lock_guard<std::mutex> lock(mutex_main_point_cloud_);
        cur_radar_struct = i_radar_struct_;
        int i_point_cloud_num = cur_radar_struct.points->size();
        ROS_INFO_STREAM("RadarOdometry: Input radar points num: " << i_point_cloud_num);
    }

    b_is_new_point_cloud_ = false;
}

void RadarOdometry::Publish()
{

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
        if ( v_ini_parser_.ParseConfig("radar_odometry", "sensor_topic", cfg_str_sensor_topic_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/sensor_topic");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "can_topic", cfg_str_can_topic_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/can_topic");
        }
        if ( v_ini_parser_.ParseConfig("radar_odometry", "imu_topic", cfg_str_imu_topic_) == false ) {
            ROS_ERROR_STREAM("Failed to get param: /radar_odometry/imu_topic");
        }

        ROS_WARN("RadarOdometry: INI Updated!");
    }
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