/****************************************************************************/
// Module:      interface_control_points.hpp
// Description: control points interface
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
// Version: 0.1
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
/****************************************************************************/

#ifndef __INTERFACE_CONTROL_POINTS_HPP__
#define __INTERFACE_CONTROL_POINTS_HPP__
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


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //          
    typedef struct{
        double x;
        double y;
        double yaw;
        double curvature;
        double vx;
        double time;
        double distance;
    } LateralControlPoint;

    typedef struct {
        std::string frame_id;
        std::vector<LateralControlPoint> control_point;
    } LateralControlTrajectory;

    typedef struct {
        float steering_tire_angle;
        LateralControlTrajectory control_path;
    } LateralControlCommand;
    
    typedef struct {
        double cross_track_error_m;
        double yaw_error_deg;
        double yawrate_error_degs;
        double understeer_gradient_deg;
        double yawrate_compensation_deg;
    } PathTrackingInfos;

    typedef struct{
        double x;
        double y;
        double vx;
        double ax;
        double fx;
        double trq;
        double time;
        double distance;
    } LongitudinalControlPoint;


    typedef struct {
        std::string frame_id;
        std::vector<LongitudinalControlPoint> control_point;
    } LongitudinalControlTrajectory;
    
    typedef struct {
        float ax;
        float trq;
        LongitudinalControlTrajectory control_profile;
    } LongitudinalControlCommand;

    typedef struct {
        double speed_error_kph;
        double accel_error_mps2;
        double compensated_torque_Nm;
        double speed_compensation_error_kph;
    } SpeedTrackingInfos;

    typedef struct { 
        double steering_angle;
        double front_tire_angle;
        double speed;
        double accel;
        double gas;
        double brake;
        double torque;
    } VehicleCmd;

    typedef struct { 
        bool lateral_mode;
        bool longitudinal_mode;
        bool ready_mode;
        bool manual_mode;
    } ADModeInput;

    typedef struct { 
        uint8_t siren_on_off;
        uint8_t led_on_off;
    } SirenOnOff;
}

#endif // __INTERFACE_CONTROL_POINTS_HPP__
