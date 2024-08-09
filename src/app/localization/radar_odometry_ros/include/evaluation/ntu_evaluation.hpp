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

class NtuEvaluation{
public:
    NtuEvaluation();
    ~NtuEvaluation();
    void Evaluation(std::vector<std::pair<double, Eigen::Matrix4d>> vec_time_radar_pose_mat, 
                    std::string result_folder_path, std::string calib_file_path);
    
private:
    
    // Calibration params
    Eigen::Matrix4d ntu_radar2imu_mat_;
};
