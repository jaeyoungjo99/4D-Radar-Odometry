/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com
 * @file lidar_preprocessing.hpp
 * @brief preprocessing tools for lidar
 * @version 1.0
 * @date 14-07-2022
 * @bug No known bugs
 * @warning No warnings
 */
#ifndef __POINTXYZIT_HPP__
#define __POINTXYZIT_HPP__

/* Includes */
#include <string>
// STL
#include <vector>
#include <deque>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>

/* Types */
struct PointXYZIT
{
    PCL_ADD_POINT4D;
    float intensity;
    float time;      // point time after scan start
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // make sure our new allocators are aligned
} EIGEN_ALIGN16;    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZIT,        
				    ( float, x, x )
				    ( float, y, y )
				    ( float, z, z )
				    ( float, intensity, intensity )
				    ( float, time, time )
)

#endif //__POINTXYZIT_HPP__