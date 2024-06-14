/****************************************************************************/
// Module:      rosbridge_pointcloud.hpp
// Description: ROS bridge for pointcloud
//
// Authors: Jaeyoung Jo (woudd3743@gmail.com)
// Version: 0.1
//
// Revision History
//      Aug 15, 2023: Jaeyoung Jo - Created.
/****************************************************************************/

#ifndef __ROSBRIDGE_POINTCLOUD__
#define __ROSBRIDGE_POINTCLOUD__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// PCL header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS Message Header
#include <sensor_msgs/PointCloud2.h>

// Interface Header
#include "rosbridge_time.hpp"

using namespace autoku_types;

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint16_t, ring, ring) (uint16_t, ambient, ambient) (uint32_t, range, range)
)

struct LivoxPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint8_t tag;
    uint8_t line;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint8_t, tag, tag) (uint8_t, line, line) (double, timestamp, timestamp)
)

namespace autoku_functions{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    template<typename pointType>
    pcl::PointCloud<pointType> GetPointCloud(const sensor_msgs::PointCloud2 &msg) {
        sensor_msgs::PointCloud2 i_point_cloud = msg;

        pcl::PointCloud<pointType> point_cloud;
        pcl::fromROSMsg(i_point_cloud, point_cloud);
        return point_cloud;
    }   

} // namespace autoku_functions

#endif  // __ROSBRIDGE_POINTCLOUD__