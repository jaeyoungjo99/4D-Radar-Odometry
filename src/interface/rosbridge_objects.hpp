/****************************************************************************/
// Module:      rosbridge_objects.hpp
// Description: ROS bridge for objects
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
// Version: 0.1
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
/****************************************************************************/

#ifndef __ROSBRIDGE_OBJECTS__
#define __ROSBRIDGE_OBJECTS__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <autoku_msgs/DetectObjects.h>
#include <autoku_msgs/TrackObjects.h>
#include <autoku_msgs/PredictObjects.h>

// Interface Header
#include "rosbridge_time.hpp"
#include "interface_objects.hpp"

using namespace autoku_types;

namespace autoku_functions{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    DetectObjects GetDetectObjects(const autoku_msgs::DetectObjects& msg) {
        autoku_msgs::DetectObjects i_objects = msg;

        DetectObjects objects;
        objects.time_stamp = GetTimeStamp(i_objects.header.stamp);
        for (auto i_object : i_objects.object) {
            DetectObject object;
            object.id = i_object.id;
            object.detection_confidence = i_object.detection_confidence;

            object.classification = (ObjectClass)i_object.classification;
            object.dynamic_state = (ObjectDynamicState)i_object.dynamic_state;
            
            object.dimension.length = i_object.dimension.length;
            object.dimension.width = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;

            object.state.time_stamp = GetTimeStamp(i_object.state.header.stamp);
            object.state.x = i_object.state.x;
            object.state.y = i_object.state.y;
            object.state.z = i_object.state.z;
            object.state.roll = i_object.state.roll;
            object.state.pitch = i_object.state.pitch;
            object.state.yaw = i_object.state.yaw;
            object.state.v_x = i_object.state.v_x;
            object.state.v_y = i_object.state.v_y;
            object.state.v_z = i_object.state.v_z;
            object.state.a_x = i_object.state.a_x;
            object.state.a_y = i_object.state.a_y;
            object.state.a_z = i_object.state.a_z;
            object.state.roll_rate = i_object.state.roll_rate;
            object.state.pitch_rate = i_object.state.pitch_rate;
            object.state.yaw_rate = i_object.state.yaw_rate;

            objects.object.push_back(object);
        }

        return objects;
    }  

    TrackObjects GetTrackObjects(const autoku_msgs::TrackObjects& msg) {
        autoku_msgs::TrackObjects i_objects = msg;

        TrackObjects objects;
        objects.time_stamp = GetTimeStamp(i_objects.header.stamp);
        for (auto i_object : i_objects.object) {
            TrackObject object;
            object.id = i_object.id;

            object.classification = (ObjectClass)i_object.classification;
            object.dynamic_state = (ObjectDynamicState)i_object.dynamic_state;
            
            object.dimension.length = i_object.dimension.length;
            object.dimension.width = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;            

            object.imm_model_probability.cv = i_object.imm_model_probability.cv;
            object.imm_model_probability.ca = i_object.imm_model_probability.ca;
            object.imm_model_probability.ctrv = i_object.imm_model_probability.ctrv;
            object.imm_model_probability.ctra = i_object.imm_model_probability.ctra;
            
            object.state.time_stamp = GetTimeStamp(i_object.state.header.stamp);
            object.state.x = i_object.state.x;
            object.state.y = i_object.state.y;
            object.state.z = i_object.state.z;
            object.state.roll = i_object.state.roll;
            object.state.pitch = i_object.state.pitch;
            object.state.yaw = i_object.state.yaw;
            object.state.v_x = i_object.state.v_x;
            object.state.v_y = i_object.state.v_y;
            object.state.v_z = i_object.state.v_z;
            object.state.a_x = i_object.state.a_x;
            object.state.a_y = i_object.state.a_y;
            object.state.a_z = i_object.state.a_z;
            object.state.roll_rate = i_object.state.roll_rate;
            object.state.pitch_rate = i_object.state.pitch_rate;
            object.state.yaw_rate = i_object.state.yaw_rate;
            
            object.state_covariance.x = i_object.state_covariance.x;
            object.state_covariance.y = i_object.state_covariance.y;
            object.state_covariance.z = i_object.state_covariance.z;
            object.state_covariance.roll = i_object.state_covariance.roll;
            object.state_covariance.pitch = i_object.state_covariance.pitch;
            object.state_covariance.yaw = i_object.state_covariance.yaw;
            object.state_covariance.v_x = i_object.state_covariance.v_x;
            object.state_covariance.v_y = i_object.state_covariance.v_y;
            object.state_covariance.v_z = i_object.state_covariance.v_z;
            object.state_covariance.a_x = i_object.state_covariance.a_x;
            object.state_covariance.a_y = i_object.state_covariance.a_y;
            object.state_covariance.a_z = i_object.state_covariance.a_z;
            object.state_covariance.roll_rate = i_object.state_covariance.roll_rate;
            object.state_covariance.pitch_rate = i_object.state_covariance.pitch_rate;
            object.state_covariance.yaw_rate = i_object.state_covariance.yaw_rate;

            objects.object.push_back(object);
        }

        return objects;
    }

    PredictObjects GetPredictObjects(const autoku_msgs::PredictObjects& msg) {
        autoku_msgs::PredictObjects i_objects = msg;

        PredictObjects objects;
        objects.time_stamp = GetTimeStamp(i_objects.header.stamp);
        for (auto i_object : i_objects.object) {
            PredictObject object;
            object.id = i_object.id;

            object.classification = (ObjectClass)i_object.classification;
            object.dynamic_state = (ObjectDynamicState)i_object.dynamic_state;
            
            object.dimension.length = i_object.dimension.length;
            object.dimension.width = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;

            for (const auto& i_state : i_object.state) {       
                ObjectState state;
                state.time_stamp = GetTimeStamp(i_state.header.stamp);
                state.x = i_state.x;
                state.y = i_state.y;
                state.z = i_state.z;
                state.roll = i_state.roll;
                state.pitch = i_state.pitch;
                state.yaw = i_state.yaw;
                state.v_x = i_state.v_x;
                state.v_y = i_state.v_y;
                state.v_z = i_state.v_z;
                state.a_x = i_state.a_x;
                state.a_y = i_state.a_y;
                state.a_z = i_state.a_z;
                state.roll_rate = i_state.roll_rate;
                state.pitch_rate = i_state.pitch_rate;
                state.yaw_rate = i_state.yaw_rate;

                object.state.push_back(state);
            }            

            objects.object.push_back(object);
        }

        return objects;
    }

} // namespace autoku_functions

#endif  // __ROSBRIDGE_OBJECTS__