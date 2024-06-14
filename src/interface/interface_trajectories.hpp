/****************************************************************************/
// Module:      interface_trajectories.hpp
// Description: trajectories interface
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
// Version: 0.1
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
/****************************************************************************/

#ifndef __INTERFACE_TRAJECTORIES_HPP__
#define __INTERFACE_TRAJECTORIES_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>

namespace autoku_types {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //     
    typedef enum { 
        LANE_KEEPING,
        NO_OVERTAKING, 
        CAR_FOLLOWING, 
        OVERTAKING,
        OVERTAKE_SLOW_VEHICLE,
        OVERTAKE_STATIC_VEHICLE,
        LAP_TIME,
        STOP,
        COME_BACK,
    } BehaviorType;

    typedef enum {        
        DISTANCE_BASED = 0,
        TIME_BASED,
    } ResampleMethod;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //  

    typedef struct {
        float x;
        float y;
        float z;
        float s;
        float n;
    } TrajectoryBoundaryPoint;

    typedef struct {
        std::vector<TrajectoryBoundaryPoint> point;
    } TrajectoryBoundary;

    typedef struct {
        float time;
        float x;
        float y;
        float z;
        float yaw;
        float curvature;
        float distance;
        float speed;
        float acceleration;
        float s;
        float n;
        float ds;
        float dn;
        float dds;
        float ddn;  
        float yaw_sn;
    } TrajectoryPoint;

    typedef struct {
        double time_stamp;
        uint16_t id;
        BehaviorType behavior_zone;
        BehaviorType next_behavior_zone;
        BehaviorType decision_behavior;
        double behavior_distance;
        bool is_target;

        TrajectoryBoundary left_boundary;
        TrajectoryBoundary right_boundary;
        TrajectoryBoundary center_boundary;
        std::vector<TrajectoryPoint> point;
    } Trajectory;

    typedef struct {
        double time_stamp;
        BehaviorType behavior_zone;
        BehaviorType decision_behavior;
        std::vector<Trajectory> trajectory;
    } Trajectories;
    
    typedef struct {
        bool on_ego;
        bool collision;
        double x;
        double y;
        double r;
    } CollisionBoundary;
    
    typedef struct{
        double time_stamp;
        std::vector<CollisionBoundary> boundary;
    } CollisionBoundaries;
}

#endif // __INTERFACE_TRAJECTORIES_HPP__
