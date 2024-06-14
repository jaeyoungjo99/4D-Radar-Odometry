#ifndef __EKF_COMMON_STRUCT__
#define __EKF_COMMON_STRUCT__
#pragma once

// STD header
#include <iostream>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <deque>
#include <string>
#include <unordered_map>

// Libraries
#include <Eigen/Dense>

typedef enum {
    NO_SOURCE, 
    NO_MATCHING,
    UNVALID_LANE,
    SUCCESS
} MatchingStatus;

typedef enum { 
    LATERAL,
    LONGITUDINAL,
    XYYaw
} MatchingType;

typedef enum {
    NOVATEL,
    MOBILEYE,
    BOUNDARY,
    LANEPOINT
} GnssSource;

typedef struct{
    double ld2;
    double la;
    double lb;
    bool b_proj;
} PointToLineInfo;

typedef struct{
    double timestamp;

    double x_m;
    double y_m;
    double vel_ms;
    double yaw_rad;

    double latitude;
    double longitude;

    double latitude_std; // world
    double longitude_std; // world
    
    double x_cov_m; // ego
    double y_cov_m; // ego
    double vel_cov_ms;
    double yaw_cov_rad;
} EgoState;

typedef struct{
    double timestamp;

    GnssSource gnss_source;

    double latitude;
    double longitude;
    double height;
    double x_m;
    double y_m;
    double vel_ms;
    double yaw_rad;

    double latitude_std;
    double longitude_std;
    double height_std;
    double x_cov_m; // local_x
    double y_cov_m; // local_y
    double vel_cov_ms;
    double yaw_cov_rad;
} GnssStruct;

typedef struct{
    double timestamp;
    int matching_status;
    int matching_type;

    double lat_correction_m;
    double lon_correction_m;
    double yaw_correction_rad;

    double lat_correction_std_m;
    double lon_correction_std_m;
    double yaw_correction_std_rad;

} CorrectionStruct;

typedef struct{
    double timestamp;

    double vel_mps;
    double yaw_rate_rad;
} CanStruct;


typedef struct{
    int lane_type;
    int quality; // should be over 2
    std::vector<Eigen::Vector2d> lane_points;

} MobileyeLaneStruct;

typedef struct{
    double timestamp;

    std::vector<MobileyeLaneStruct> lanes;

} MobileyeLanesStruct;

typedef struct{
    Eigen::Vector2d point_start;
    Eigen::Vector2d point_end;
} LineStruct;

typedef struct{
    int line_id; // may be id from osm?
    std::vector<Eigen::Vector2d> line_points;
} PolyLineStruct;

typedef std::vector<PolyLineStruct> PolyLineVec;

typedef struct{
    int grid_id;
    double grid_center_x_m;
    double grid_center_y_m;
    PolyLineVec lines;
} PolyLineGridStruct;

#endif