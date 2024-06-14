/**
 * Module:      rosbridge_control_points.hpp
 * Description: ROS bridge for control points
 * 
 * Authors: Yuseung Na (ys.na0220@gmail.com)
 *          XXXXXXX XXX (@gmail.com)
 * 
 * Revision History
 *      June 14, 2023: Yuseung Na - Created.
 *      XXXX XX, XXXX: XXXXXXX XX - 
 */

#ifndef __ROSBRIDGE_CONTROL_POINTS__
#define __ROSBRIDGE_CONTROL_POINTS__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <autoku_msgs/Trajectories.h>

// Interface Header
#include "rosbridge_time.hpp"
#include "interface_control_points.hpp"

using namespace autoku_types;

namespace autoku_functions{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
 
} // namespace autoku_functions

#endif  // __ROSBRIDGE_CONTROL_POINTS__