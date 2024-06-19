#ifndef __POINTTYPE_HPP__
#define __POINTTYPE_HPP__

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>


/* Types */
struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    float    intensity;
    uint16_t ring;
    float    time;      // point time after scan start
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // make sure our new allocators are aligned
} EIGEN_ALIGN16;    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZIRT,
                    ( float, x, x )
                    ( float, y, y )
                    ( float, z, z )
                    ( float, intensity, intensity )
                    ( uint16_t, ring, ring )
				    ( float, time, time )
)

/* Types */
struct PointXYZPRVAE
{
    PCL_ADD_POINT4D;
    float       power;
    float       range;
    float       vel;
    float       azi_angle;
    float       ele_angle;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // make sure our new allocators are aligned
} EIGEN_ALIGN16;    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZPRVAE,
                    ( float, x, x )
                    ( float, y, y )
                    ( float, z, z )
                    ( float, power, power )
                    ( float, range, range )
				    ( float, vel, vel )
                    ( float, azi_angle, azi_angle )
                    ( float, ele_angle, ele_angle )
)

#endif
