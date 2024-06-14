/****************************************************************************/
// Module:      interface_novatel.hpp
// Description: novatel ros message interface
//
// Authors: Jiwon Seok (pauljiwon96@gmail.com)
//
// Revision History
//      Oct 05, 2023: Jiwon Seok - First file create
/****************************************************************************/
#ifndef __INTERFACE_NOVATEL_HPP__
#define __INTERFACE_NOVATEL_HPP__
#pragma once

#include <novatel_oem7_msgs/CORRIMU.h>
#include <novatel_oem7_msgs/INSPVAX.h>
#include <stdint.h>

// Novatel struct
namespace autoku_types {
    typedef struct {
        double   time_stamp;
        uint32_t imu_data_count;
        double   pitch_rate;
        double   roll_rate;
        double   yaw_rate;
        double   lateral_acc;
        double   longitudinal_acc;
        double   vertical_acc;
        uint32_t reserved1;
        uint32_t reserved2;
    } CORRIMU;
    typedef struct {
        double   time_stamp;
        uint32_t ins_status;
        uint32_t pos_type;
        double   latitude;
        double   longitude;
        double   height;
        float    undulation;
        double   north_velocity;
        double   east_velocity;
        double   up_velocity;
        double   roll;
        double   pitch;
        double   azimuth;
        float    latitude_stdev;
        float    longitude_stdev;
        float    height_stdev;
        float    north_velocity_stdev;
        float    east_velocity_stdev;
        float    up_velocity_stdev;
        float    roll_stdev;
        float    pitch_stdev;
        float    azimuth_stdev;
        uint32_t ext_sol_status;
        uint16_t time_since_update;
    } INSPVAX;
} // namespace autoku_types

#endif // __INTERFACE_NOVATEL_HPP__