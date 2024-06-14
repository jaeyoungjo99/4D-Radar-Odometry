/****************************************************************************/
// Module:      interface_objects.hpp
// Description: objects interface
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
// Version: 0.1
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
//      October 5, 2023: 
/****************************************************************************/

#ifndef __INTERFACE_OBJECTS_HPP__
#define __INTERFACE_OBJECTS_HPP__
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
        UNKNOWN = 0,
        CAR,
        BARRIER,
        SIGN
    } ObjectClass;

    typedef enum {
        UNKNOWN_STATE = 0,
        STATIC = 1,
        DYNAMIC = 2,
    } ObjectDynamicState;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    typedef struct {
        double time_stamp{0.0};
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
        double v_x;
        double v_y;
        double v_z;
        double a_x;
        double a_y;
        double a_z;
        double roll_rate;
        double pitch_rate;
        double yaw_rate;
    } ObjectState;

    typedef struct {
        double length;
        double width;
        double height;
    } ObjectDimension;

    typedef struct {
        double cv{0.0};
        double ca{0.0};
        double ctrv{0.0};
        double ctra{0.0};
    } IMMModelProbability;

    typedef struct {  
        unsigned int id;
        float detection_confidence;

        ObjectClass classification;
        ObjectDynamicState dynamic_state;

        ObjectDimension dimension;
        
        ObjectState state;
    } DetectObject;

    typedef struct {        
        double time_stamp{0.0};
        std::vector<DetectObject> object;
    } DetectObjects;

    typedef struct {
        unsigned int id;
        bool is_coasted;
        
        ObjectClass classification;
        ObjectDynamicState dynamic_state;

        ObjectDimension dimension;

        IMMModelProbability imm_model_probability;
        ObjectState state;
        ObjectState state_covariance;
    } TrackObject;

    typedef struct {
        double time_stamp{0.0};
        std::vector<TrackObject> object;
    } TrackObjects;

    typedef struct {
        unsigned int id;
        
        ObjectClass classification;
        ObjectDynamicState dynamic_state;
        
        ObjectDimension dimension;

        std::vector<ObjectState> state;
    } PredictObject;

    typedef struct {
        double time_stamp{0.0};
        std::vector<PredictObject> object;
    } PredictObjects;

    typedef struct {
        double time_stamp;
        bool is_valid;
        double lx;
        double ly;
        double rx;
        double ry;
        double dist;
    } TargetLine;

    typedef struct {
        double time_stamp;

        double dist;

        double s;
        double ds;

        double n;
        double dn;
    } TargetFrenetObject;
}

#endif // __INTERFACE_OBJECTS_HPP__
