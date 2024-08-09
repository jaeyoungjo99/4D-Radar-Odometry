// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <omp.h>
#include <unordered_map>
#include <cmath>
#include <Eigen/Dense>
#include <random>

// ROS header
#include <ros/ros.h>

// PCL
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>


#include <fstream>
#include <filesystem>

#include "types/point_type.hpp"

// json
#include <nlohmann/json.hpp>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Float32MultiArray.h>
#include "types/point_type.hpp"

// Message header
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Point.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <nav_msgs/Odometry.h>

#include <ini_parser.h>

#include "radar_odometry.hpp"

using namespace ros;
// using namespace tf;
using namespace std;


class VodEvaluation{
public:
    VodEvaluation();
    ~VodEvaluation();
    void ProcessRadarFiles();
    void ProcessRadarFile(std::string& vod_seq_folder_path, bool b_debug_mode, std::string& vod_eval_folder_path);
    
private:
    void Init();
    void ProcessINI();

    void RunRadarOdometry(RadarDataStruct i_radar_struct);


    void savePosesToFile(const std::vector<Eigen::Affine3d>& poses, const std::string& filename);
    void GetPoseJsonFiles(std::string& pose_directory_path, std::vector<std::filesystem::path>& pose_files);
    void GetRadarFiles(std::string& radar_directory_path, std::vector<std::filesystem::path>& radar_files);
    void ParsePoseJsonFiles(const std::vector<std::filesystem::path>& pose_files, const Eigen::Affine3d& cam2radar_tfrom, std::vector<Eigen::Affine3d>& odom2radar_transforms);
    void ConvertBin2PointCloud(std::ifstream& bin_file, std::vector<SRadarPoint>& o_s_radar_points, int time_idx);
    Eigen::Affine3d jsonToAffine3d(const nlohmann::json& json);
    // Pub GT
    void pubVodGtPose(const Eigen::Affine3d& vod_gt_pose);
    void BroadcastTFAndVisualizeOdomPose(const Eigen::Affine3d& result_tf_in_ego_pose);
    void VisualizeAndPubOdomPose(double odom_x, double odom_y, double odom_z, tf::Quaternion odom_rpy);

    // VOD
    ros::Publisher rospub_odom_accumulated_map_;
    ros::Publisher rospub_radar_ego_motion_odom_;
    ros::Publisher rospub_nov_pose_;
    ros::Publisher rospub_radar_odom_pose_;
    ros::Publisher rospub_radar_odom_pose_stamped_;
    ros::Publisher rospub_vod_radar_points_;
    ros::Publisher rospub_gt_odom_pose_;
    ros::Publisher rospub_correspondences_;
    ros::Publisher rospub_novatel_ref_pose_stamped_;
    ros::Publisher rospub_radar_odom_eval_pose_stamped_;

    IniParser v_ini_parser_;

    double vod_dt_;

    RadarDataStruct i_vod_radar_struct_;


    // VOD
    // Calibration params
    Eigen::Matrix4d vod_cam2radar_mat_;
    sensor_msgs::PointCloud2 i_point_cloud_;
    Eigen::Affine3d result_tf_;

    geometry_msgs::PoseStamped novatel_pose_stamped_;
    geometry_msgs::PoseStamped radar_odom_pose_stamped_;

    // Odometry
    Eigen::Matrix4d radar_pose_;

    radar_odometry::pipeline::RadarOdometry odometry_;
    radar_odometry::pipeline::RadarOdometryConfig config_;
    int cfg_i_odometry_type_;
    int cfg_i_icp_type_;

    sensor_msgs::PointCloud2 o_cur_radar_global_cloud_;
};
