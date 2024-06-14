/****************************************************************************/
// Module:      rosbridge_novatel.hpp
// Description: ROS bridge for novatel
//
// Authors: Jiwon Seok (pauljiwon96@gmail.com)
// Version: 0.1
//
// Revision History
//      Oct 05, 2023: Initial file create
/****************************************************************************/

#ifndef __ROSBRIDGE_NOVATEL__
#define __ROSBRIDGE_NOVATEL__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <novatel_oem7_msgs/CORRIMU.h>
#include <novatel_oem7_msgs/INSPVAX.h>

// Interface Header
#include "interface_novatel.hpp"
#include "rosbridge_time.hpp"

using namespace autoku_types;

namespace autoku_functions {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    CORRIMU GetCorrImu(const novatel_oem7_msgs::CORRIMU& msg) {
        novatel_oem7_msgs::CORRIMU i_corrimu = msg;

        CORRIMU corrimu;
        corrimu.time_stamp = GetTimeStamp(i_corrimu.header.stamp);

        corrimu.imu_data_count   = i_corrimu.imu_data_count;
        corrimu.pitch_rate       = i_corrimu.pitch_rate;
        corrimu.roll_rate        = i_corrimu.roll_rate;
        corrimu.yaw_rate         = i_corrimu.yaw_rate;
        corrimu.lateral_acc      = i_corrimu.lateral_acc;
        corrimu.longitudinal_acc = i_corrimu.longitudinal_acc;
        corrimu.vertical_acc     = i_corrimu.vertical_acc;
        corrimu.reserved1        = i_corrimu.reserved1;
        corrimu.reserved2        = i_corrimu.reserved2;

        return corrimu;
    }

    INSPVAX GetInspvax(const novatel_oem7_msgs::INSPVAX& msg) {
        novatel_oem7_msgs::INSPVAX i_inspvax = msg;

        INSPVAX inspvax;
        inspvax.time_stamp = GetTimeStamp(i_inspvax.header.stamp);

        inspvax.ins_status           = i_inspvax.ins_status.status;
        inspvax.pos_type             = i_inspvax.pos_type.type;
        inspvax.latitude             = i_inspvax.latitude;
        inspvax.longitude            = i_inspvax.longitude;
        inspvax.height               = i_inspvax.height;
        inspvax.undulation           = i_inspvax.undulation;
        inspvax.north_velocity       = i_inspvax.north_velocity;
        inspvax.east_velocity        = i_inspvax.east_velocity;
        inspvax.up_velocity          = i_inspvax.up_velocity;
        inspvax.roll                 = i_inspvax.roll;
        inspvax.pitch                = i_inspvax.pitch;
        inspvax.azimuth              = i_inspvax.azimuth;
        inspvax.latitude_stdev       = i_inspvax.latitude_stdev;
        inspvax.longitude_stdev      = i_inspvax.longitude_stdev;
        inspvax.height_stdev         = i_inspvax.height_stdev;
        inspvax.north_velocity_stdev = i_inspvax.north_velocity_stdev;
        inspvax.east_velocity_stdev  = i_inspvax.east_velocity_stdev;
        inspvax.up_velocity_stdev    = i_inspvax.up_velocity_stdev;
        inspvax.roll_stdev           = i_inspvax.roll_stdev;
        inspvax.pitch_stdev          = i_inspvax.pitch_stdev;
        inspvax.azimuth_stdev        = i_inspvax.azimuth_stdev;
        inspvax.ext_sol_status       = i_inspvax.ext_sol_status.status;
        inspvax.time_since_update    = i_inspvax.time_since_update;

        return inspvax;
    }
} // namespace autoku_functions

#endif // __ROSBRIDGE_NOVATEL__