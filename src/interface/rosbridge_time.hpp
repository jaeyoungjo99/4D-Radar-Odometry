/****************************************************************************/
// Module:      rosbridge_time.hpp
// Description: ROS bridge for time
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
// Version: 0.1
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
/****************************************************************************/

#ifndef __ROSBRIDGE_TIME__
#define __ROSBRIDGE_TIME__
#pragma once

// ROS header
#include <ros/ros.h>

// ROS Message Header

// Interface Header
#include "interface_constants.hpp"

using namespace autoku_types;

namespace autoku_functions {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    double GetTimeStamp(const ros::Time& stamp) {
        return (double)stamp.sec + (double)stamp.nsec * 1e-9;
    }
} // namespace autoku_functions

#endif  // __ROSBRIDGE_TIME__