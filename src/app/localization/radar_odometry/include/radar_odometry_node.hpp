/****************************************************************************/
// Module:      radar_odometry_node.hpp
// Description: radar_odometry_node
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com)
// Version: 0.1
//
// Revision History
//      April 2, 2024: Jaeyoung Jo - Created.
//      XXXX XX, 2023: XXXXXXX XX - 
/****************************************************************************/

#ifndef __RADAR_ODOMETRY_NODE_HPP__
#define __RADAR_ODOMETRY_NODE_HPP__
#pragma once

// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <omp.h>
#include <unordered_map>

// ROS header
#include <ros/ros.h>

// PCL
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>

// Utility header
#include <atom_task.h>
#include <ini_parser.h>


// Message header
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>

#include "point_type.hpp"

// Namespace
using namespace ros;
// using namespace tf;
using namespace std;
// using namespace autoku_types;

typedef struct {
    double timestamp; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr points;
    std::string frame_id;
} LidarDataStruct;

typedef struct {
    double timestamp; 
    pcl::PointCloud<PointXYZPRVAE>::Ptr points;
    std::string frame_id;
} RadarDataStruct;

class RadarOdometry : public AtomTask {
    public:
        // Constructor
        explicit RadarOdometry(int id, std::string task_node, double period);
        // Destructor
        virtual ~RadarOdometry();
        
    public:
        void Init();
        void Run();
        void Publish();
        void Terminate();
        void ProcessINI();

    private:
        inline void CallbackMainPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_main_point_cloud_);
            i_main_point_cloud_ = *msg;
            if(cfg_str_sensor_type_ == "lidar"){
                pcl::moveFromROSMsg(i_main_point_cloud_, *i_main_lidar_ptr_);

                i_main_lidar_raw_tuple_ = std::make_tuple(i_main_lidar_ptr_, i_main_point_cloud_.header.frame_id, i_main_point_cloud_.header.stamp);
                
                i_lidar_struct_.timestamp = i_main_point_cloud_.header.stamp.toSec();
                i_lidar_struct_.points = i_main_lidar_ptr_;
                i_lidar_struct_.frame_id = i_main_point_cloud_.header.frame_id;
            }

            if(cfg_str_sensor_type_ == "radar"){
                pcl::moveFromROSMsg(i_main_point_cloud_, *i_main_radar_ptr_);

                i_main_radar_raw_tuple_ = std::make_tuple(i_main_radar_ptr_, i_main_point_cloud_.header.frame_id, i_main_point_cloud_.header.stamp);
                
                i_radar_struct_.timestamp = i_main_point_cloud_.header.stamp.toSec();
                i_radar_struct_.points = i_main_radar_ptr_;
                i_radar_struct_.frame_id = i_main_point_cloud_.header.frame_id;
            }

            b_is_new_point_cloud_ = true;

        }

        inline void CallbackCAN(const geometry_msgs::TwistStampedConstPtr& msg){
            i_can_ = *msg;
        }

        inline void CallbackIMU(const sensor_msgs::ImuConstPtr& msg){
            i_imu_ = *msg;
        }

    private:
        mutex mutex_main_point_cloud_;

        IniParser v_ini_parser_;

        ros::Subscriber s_main_point_cloud_;
        ros::Subscriber s_can_;
        ros::Subscriber s_imu_;

        // input
        sensor_msgs::PointCloud2 i_main_point_cloud_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr i_main_lidar_ptr_;
        pcl::PointCloud<PointXYZPRVAE>::Ptr i_main_radar_ptr_;
        std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, string, ros::Time> i_main_lidar_raw_tuple_;
        std::tuple<pcl::PointCloud<PointXYZPRVAE>::Ptr, string, ros::Time> i_main_radar_raw_tuple_;
        LidarDataStruct i_lidar_struct_;
        RadarDataStruct i_radar_struct_;

        sensor_msgs::Imu i_imu_;
        geometry_msgs::TwistStamped i_can_;

        // Variables
        bool b_is_new_point_cloud_ = false;

        // Configure

        std::string cfg_str_sensor_type_;
        std::string cfg_str_sensor_topic_;
        std::string cfg_str_can_topic_;
        std::string cfg_str_imu_topic_;
        
        
};

#endif  // __CAD_REGISTRATION_HPP__