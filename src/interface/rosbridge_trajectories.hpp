/**
 * Module:      rosbridge_trajectories.hpp
 * Description: ROS bridge for trajectories
 * 
 * Authors: Yuseung Na (ys.na0220@gmail.com)
 *          XXXXXXX XXX (@gmail.com)
 * 
 * Revision History
 *      June 14, 2023: Yuseung Na - Created.
 *      XXXX XX, XXXX: XXXXXXX XX - 
 */

#ifndef __ROSBRIDGE_TRAJECTORIES__
#define __ROSBRIDGE_TRAJECTORIES__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <autoku_msgs/Trajectories.h>

// Interface Header
#include "rosbridge_time.hpp"
#include "interface_trajectories.hpp"
#include "function_trajectories.hpp"

using namespace autoku_types;

namespace autoku_functions{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    Trajectory GetTrajectory(const autoku_msgs::Trajectory& msg, const bool use_boundary = true) {
        autoku_msgs::Trajectory i_trajectory = msg;

        Trajectory behavior_trajectory;
        behavior_trajectory.time_stamp = GetTimeStamp(i_trajectory.header.stamp);
        behavior_trajectory.behavior_zone = (BehaviorType)i_trajectory.behavior_zone;
        behavior_trajectory.next_behavior_zone = (BehaviorType)i_trajectory.next_behavior_zone;
        behavior_trajectory.decision_behavior = (BehaviorType)i_trajectory.decision_behavior;
        behavior_trajectory.behavior_distance = i_trajectory.behavior_distance;

        for (auto i_point : i_trajectory.point) {            
            TrajectoryPoint tp;
            tp.time = i_point.time;
            tp.x = i_point.x;
            tp.y = i_point.y;
            tp.z = i_point.z;
            tp.yaw = i_point.yaw;
            tp.curvature = i_point.curvature;
            tp.distance = i_point.distance;
            tp.speed = i_point.speed;
            tp.acceleration = i_point.acceleration;

            behavior_trajectory.point.push_back(tp);
        }

        if(use_boundary == false){
            return behavior_trajectory;
        }

        for (auto i_point : i_trajectory.left_boundary.point) {
            TrajectoryBoundaryPoint tbp;
            tbp.x = i_point.x;
            tbp.y = i_point.y;
            tbp.z = i_point.z;
            tbp.s = i_point.s;
            tbp.n = i_point.n;

            behavior_trajectory.left_boundary.point.push_back(tbp);
        }
        for (auto i_point : i_trajectory.right_boundary.point) {
            TrajectoryBoundaryPoint tbp;
            tbp.x = i_point.x;
            tbp.y = i_point.y;
            tbp.z = i_point.z;
            tbp.s = i_point.s;
            tbp.n = i_point.n;

            behavior_trajectory.right_boundary.point.push_back(tbp);
        }
        for (auto i_point : i_trajectory.center_boundary.point) {
            TrajectoryBoundaryPoint tbp;
            tbp.x = i_point.x;
            tbp.y = i_point.y;
            tbp.z = i_point.z;
            tbp.s = i_point.s;
            tbp.n = i_point.n;

            behavior_trajectory.center_boundary.point.push_back(tbp);
        }

        return behavior_trajectory;
    }   
} // namespace autoku_functions

#endif  // __ROSBRIDGE_TRAJECTORIES__