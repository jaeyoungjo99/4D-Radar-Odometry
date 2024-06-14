/****************************************************************************/
// Module:      interface_constants.hpp
// Description: constants for interface
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
// Version: 0.1
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
/****************************************************************************/

#ifndef __INTERFACE_CONSTANTS_HPP__
#define __INTERFACE_CONSTANTS_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>
#include <cmath>

namespace autoku_types {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // constants
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    const double DEG2RAD = M_PI/180.0;
    const double RAD2DEG = 180.0/M_PI;
    const double MPS2KPH = 3600.0/1000.0;
    const double KPH2MPS = 1000.0/3600.0;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // vehicle parameters
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //  
    const double VEHICLE_WIDTH  = 1.890;//2.081;              // [m]
    const double VEHICLE_LENGTH = 4.635;            // [m]
    const double VEHICLE_HEIGHT = 1.605;            // [m]
    const double VEHICLE_WHEEL_BASE = 3.0;        // [m]
    const double VEHICLE_FRONT_AXLE_TO_CG = 1.49;        // [m]
    const double VEHICLE_REAR_AXLE_TO_CG = 1.51;        // [m]
    const double VEHICLE_REAR_OVERHANG = 0.82;      // [m]
    const double VEHICLE_FRONT_OVERHANG = 0.845;    // [m]
    const double VEHICLE_MASS = 2300.00;            // [kg] Curb weight : 2160.00
    const double VEHICLE_MOMENT_OF_INERTIA = 3637.526;            // [kgm] 
    const double VEHICLE_WHEEL_RADIUS = 0.36;      // [m]
    const double VEHICLE_FRONT_AREA = 2.877;        // [m^2]

    const double STEERING_RATIO = 17.001267478493;  // 14.2;  // [ratio]
    const double STEERING_RATIO_CARMAKER = 14.46;   // [ratio]
    const double MAX_STEERING_WHEEL_ANGLE = 480.0;  // [Deg]
    const double MAX_STEERING_WHEEL_SPEED = 680.0;  // [Deg/sec]
    
    const double TIRE_TRACTION = 1.0;               // [ratio]
    const double FRONT_CORNERING_STIFFNESS = 98047;    // sum of left and right tire
    const double REAR_CORNERING_STIFFNESS  = 164000;    // sum of left and right tire
    const double AIR_DENSITY = 1.205;
    const double AIR_DRAG_COEFF = 0.3;
    const double GRAVITY_COEFF = 9.806;
    const double ROLLING_RESISTANCE = 0.015;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // control parameters
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - // 
    
    // Torque Limit
    const double G_L1 = 1.672592399651635e-05;	    // Torque limit model
    const double G_L2 = -0.008839782335377;
    const double G_L3 = 1.790566691502815;
    const double G_L4 = -1.727310161467643e+02;
    const double G_L5 =  7.919815391217100e+03;
    const double T_UL  = 6443.0;                    // Torque Max
    const double T_LL  = -10000.0;                   // Torque Min

    // Real Vehicle
    const double T_NEUTRAL = -1000.0;                 // Torque Neutral
    
    // Brake Pedal Model : 1st order poly
    // const double BRAKE1_1 = -0.01547;
    // const double BRAKE1_2 = -10.67 ;

    // Brake Pedal Model :3rd order poly
    const double BRAKE3_1 = -1.169e-10;
    const double BRAKE3_2 = -2.723e-6 ;
    const double BRAKE3_3 = -0.02749;
    const double BRAKE3_4 = -20.86 ;


    // Brake Pedal Model : pedal = (p1*torque^2 + p2*torque + p3) / (torque + q1)
    // const double BRAKE_P1 = -0.01367;
    // const double BRAKE_P2 = -13.44;
    // const double BRAKE_P3 = -578.6;
    // const double BRAKE_Q1 =  777.9;

    const double BRAKE_P1 = -0.01096;
    const double BRAKE_P2 = 5.281;
    const double BRAKE_P3 = 13630;
    const double BRAKE_Q1 =  373.8;

    // CarMaker   
    const double T_NEUTRAL_CM = -40.0;                 // Torque Neutral
    const double GAS_1 = 0.0001711;		            // Gas model
    const double GAS_2 = 0.005647;
    const double BRAKE_1_CM = -0.00009;		        // Brake model
    const double BRAKE_2_CM = -0.005772;

    // const double T_UL  = 3473.26;                    // Torque Max in CarMaker
    // const double T_LL  = -4650.0;                   // Torque Min in CarMaker
}

#endif // __INTERFACE_CONSTANTS_HPP__