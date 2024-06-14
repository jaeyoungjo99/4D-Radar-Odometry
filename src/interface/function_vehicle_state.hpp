/**
 * Module:      function_vehicle_state.hpp
 * Description: vehicle state util functions for AutoKU
 * 
 * Authors: Yuseung Na (ys.na0220@gmail.com)
 *          XXXXXXX XXX (@gmail.com)
 * 
 * Revision History
 *      Aug. 03, 2023: Yuseung Na - Created.
 *      XXXX XX, XXXX: XXXXXXX XX - 
 */

#ifndef __FUNCTION_VEHICLE_STATE_HPP__
#define __FUNCTION_VEHICLE_STATE_HPP__
#pragma once

// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// Utility header
#include <spline.h>

// Interface Header
#include "interface_constants.hpp"
#include "interface_vehicle_state.hpp"
#include "function_print.hpp"

using namespace autoku_types;

namespace autoku_functions{
    inline FrenetVehicleState ConvertFrenetState(const VehicleState& vehicle_state,
                                                 tk::Map& road_map) {
        
        FrenetVehicleState frenet_state;
        frenet_state.time_stamp = vehicle_state.time_stamp;

        // Convert Cartesian to Frenet coordinate
        std::vector<double> sn = road_map.ToFrenet(vehicle_state.x, vehicle_state.y);
        frenet_state.s = sn[0];
        frenet_state.n = sn[1];

        double vx_global = vehicle_state.vx*cos(vehicle_state.yaw)-vehicle_state.vy*sin(vehicle_state.yaw);
        double vy_global = vehicle_state.vx*sin(vehicle_state.yaw)+vehicle_state.vy*cos(vehicle_state.yaw);
        std::vector<double> frenet_speed = road_map.ToFrenetVelocity(vx_global, vy_global, frenet_state.s);

        frenet_state.ds = frenet_speed[0];
        frenet_state.dn = frenet_speed[1];
        // frenet_state.ds = sqrt(pow(vehicle_state.vx, 2) + pow(vehicle_state.vy, 2));//frenet_speed[0];
        // frenet_state.ds = floor(frenet_state.ds * 10) / 10.0;

        double ax_global = vehicle_state.ax*cos(vehicle_state.yaw)-vehicle_state.ay*sin(vehicle_state.yaw);
        double ay_global = vehicle_state.ax*sin(vehicle_state.yaw)+vehicle_state.ay*cos(vehicle_state.yaw);
        std::vector<double> frenet_accel = road_map.ToFrenetVelocity(ax_global, ay_global, frenet_state.s);

        frenet_state.dds = frenet_accel[0];
        frenet_state.ddn = frenet_accel[1];
    
        return frenet_state;
    }

    inline double SlerpTwoRad(double upper_rad, double lower_rad, double ratio){
        double interpolated_v, interpolated_u;

        interpolated_v = ratio * sin(upper_rad) + (1.0 - ratio) * sin(lower_rad);
        interpolated_u = ratio * cos(upper_rad) + (1.0 - ratio) * cos(lower_rad);

        if (fabs(interpolated_u) < 1e-10) interpolated_u = 1e-10;

        return atan2(interpolated_v, interpolated_u);
    }

    inline double NormalizeAngleDiffRad(double rad_a, double rad_b) {
        double diff = rad_a - rad_b;
        while (diff > M_PI) {
            diff -= 2 * M_PI;
        }
        while (diff < -M_PI) {
            diff += 2 * M_PI;
        }
        return diff;
    }

    inline double NormalizeAngleRad(double i_rad){
        while (i_rad > M_PI) {
            i_rad -= 2 * M_PI;
        }
        while (i_rad < -M_PI) {
            i_rad += 2 * M_PI;
        }
        return i_rad;  
    }
}

#endif // __FUNCTION_VEHICLE_STATE_HPP__