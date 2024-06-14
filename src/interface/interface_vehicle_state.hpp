/****************************************************************************/
// Module:      interface_vehicle_state.hpp
// Description: vehicle state interface
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
// Version: 0.1
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
/****************************************************************************/

#ifndef __INTERFACE_VEHICLE_STATE_HPP__
#define __INTERFACE_VEHICLE_STATE_HPP__
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
        MANUAL     = 0,
        AUTONOMOUS = 1,
        FAIL       = 2,
    } OperationMode;

    typedef enum {
        READY = 0,
        RUN   = 1,
    } AutonomousMode;

    typedef enum {
        GEAR_P = 0,
        GEAR_R = 7,
        GEAR_N = 6,
        GEAR_D = 5
    } Gear;

    typedef enum {
        FLAG_READY   = 0,
        FLAG_START   = 1,
        FLAG_STOP    = 2,
        FLAG_WARN    = 3,
        FLAG_DEFAULT = 4
    } EFlag;

    typedef enum {
        BRAKE_OK      = 0,
        BRAKE_WARNING = 1,
        BRAKE_FADE    = 2,
    } BrakeState;

    typedef enum {
        MDPS_INIT             = 1,
        MDPS_READY            = 2,
        MDPS_STANDBY          = 3,
        MDPS_ACTIVATION_START = 4,
        MDPS_ACTIVATE         = 5,
        MDPS_ERROR            = 6,
        MDPS_ABORTED          = 7,
    } MdpsState;
  
    typedef enum {
        NONE = 0u,
        FIXEDPOS = 1u,
        FIXEDHEIGHT = 2u,
        DOPPLER_VELOCITY = 8u,
        SINGLE = 16u,
        PSRDIFF = 17u,
        WAAS = 18u,
        PROPAGATED = 19u,
        L1_FLOAT = 32u,
        NARROW_FLOAT = 34u,
        L1_INT = 48u,
        WIDE_INT = 49u,
        NARROW_INT = 50u,
        RTK_DIRECT_INS = 51u,
        INS_SBAS = 52u,
        INS_PSRSP = 53u,
        INS_PSRDIFF = 54u,
        INS_RTKFLOAT = 55u,
        INS_RTKFIXED = 56u,
        PPP_CONVERGING = 68u,
        PPP = 69u,
        OPERATIONAL = 70u,
        WARNING = 71u,
        OUT_OF_BOUNDS = 72u,
        INS_PPP_CONVERGING = 73u,
        INS_PPP = 74u,
        PPP_BASIC_CONVERGING = 77u,
        PPP_BASIC = 78u,
        INS_PPP_BASIC_CONVERGING = 79u,
        INS_PPP_BASIC = 80u,
        NOVATEL_DISCONNECTED = 99u,
    } NovatelPosType;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    // For CarMaker
    typedef struct {
        double time_stamp;
        double latitude;
        double longitude;
        double altitude;
        double heading;
        double latitude_sigma;
        double longitude_sigma;
        double altitude_sigma;
        double heading_sigma;
        double a_sigma;
        double b_sigma;
        int    quality;
        int    number_of_satellites;
        double HDOP;
    } Gnss;

    typedef struct {
        double time_stamp;
        double x;
        double y;
        double z;
    } Position;

    typedef struct {
        double time_stamp;

        double vx;
        double vy;
        double vz;
        double ax;
        double ay;
        double az;

        double roll;
        double pitch;
        double yaw;
        double roll_vel;
        double pitch_vel;
        double yaw_vel;
    } Motion;

    // For Real Vehicle
    typedef struct {
        double time_stamp;

        double cluster_odometer;

        double lateral_accel;
        double longitudinal_accel;
        double yaw_rate;

        double    steering_wheel_angle;
        double    steering_tire_angle;
        double    steering_speed;
        double    steering_torque;
        MdpsState steering_state;

        double wheel_velocity_fl;
        double wheel_velocity_fr;
        double wheel_velocity_rl;
        double wheel_velocity_rr;
        double wheel_velocity_f_avg;
        double wheel_velocity_r_avg;
        double wheel_velocity_avg;

        double motor_torque_f;
        double motor_torque_r;

        double  accel_position;
        double  brake_pressure;
        uint8_t brake_active;
        Gear    gear_select;

        OperationMode  operation_mode;
        AutonomousMode lateral_autonomous_mode;
        AutonomousMode longitudinal_autonomous_mode;
    } VehicleCAN;

    typedef struct {
        double time_stamp;

        EFlag e_flag{EFlag::FLAG_DEFAULT};

        uint8_t aim{0};
        uint8_t autoku_r{0};
        uint8_t eurecar_r{0};
        uint8_t kat{0};
        uint8_t save{0};
        uint8_t tayo{0};
    } ThreeSecsCAN;

    typedef struct {
        double time_stamp;

        double brake_temp_ch1;
        double brake_temp_ch2;
        double brake_temp_ch3;
        double brake_temp_ch4;
        double brake_temp_ch5;
        double brake_temp_ch6;
        double brake_temp_ch7;
        double brake_temp_ch8;
    } BrakeTempCAN;

    typedef struct {
        double time_stamp; // second

        NovatelPosType quality;  // Novatel gnss pose type
        int fault_state;   // State estimation fault state. 0: normal, 1: warning, 2: fault

        double x;          // East position in ENU frame [m]
        double y;          // North position in ENU frame [m]
        double z;          // Up position in ENU frame [m]

        double vx;         // Vehicle longitudinal velocity [m/s]
        double vy;         // Vehicle lateral velocity [m/s]
        double vz;         // Vehicle up velocity [m/s]

        double ax;         // Vehicle longitudinal acceleration [m/s2]
        double ay;         // Vehicle lateral acceleration [m/s2]
        double az;         // Vehicle up acceleration [m/s2]

        double roll;       // World to vehicle roll [rad]
        double pitch;      // World to vehicle pitch [rad]
        double yaw;        // World to vehicle yaw [rad]

        double roll_vel;   // Vehicle x axis rotation rate [rad/s]
        double pitch_vel;  // Vehicle y axis rotation rate [rad/s]
        double yaw_vel;    // Vehicle z axis rotation rate [rad/s]

        // Road
        double slope; // Road slope to vehicle longitudinal direction [rad]

        // Brake fade
        BrakeState brake_state; // Brake pad fade state
        double  brake_temperature;                   // Brake temperture [Celcius]

        // CAN
        double lateral_accel;                        // VehicleCan: lateral_acceleration [m/s2]
        double longitudinal_accel;                   // VehicleCan: longitudinal_accel [m/s2]
        double yaw_rate;                             // VehicleCan: yaw_rate [deg/s]

        double    steering_wheel_angle;              // Vehicle steering wheel angle [rad]
        double    steering_tire_angle;               // Tire direction angle [rad]
        double    steering_speed;                    // Steering wheel rotation speed [rad/s]
        double    steering_torque;                   // Steering torque [Nm]
        MdpsState steering_state;                    // MDPS steering state

        double wheel_vel_fl;                         // Wheel speed front left [m/s]
        double wheel_vel_fr;                         // Wheel speed front right [m/s]
        double wheel_vel_rl;                         // Wheel speed rear left [m/s]
        double wheel_vel_rr;                         // Wheel speed rear right [m/s]

        double motor_tq_f;                           // Motor front torque [Nm]
        double motor_tq_r;                           // Motor rear torque [Nm]
        double motor_tq_total;                       // Motor torque addition of front and rear  [Nm]

        double  accel_position;                      // Acceleration padal position
        double  brake_pressure;                      // Brake fluid presure [Pa]
        uint8_t brake_active;                        // Brake active signal
        Gear    gear_select;                         // Gear position P|R|N|D

        OperationMode  operation_mode;               // Operation mode [manual | AUTONOMOUS | FAIL]
        AutonomousMode lateral_autonomous_mode;      // Lateral autonomous mode [READY | RUN]
        AutonomousMode longitudinal_autonomous_mode; // Longitudinal autonomous mode [READY | RUN]
    } VehicleState;

    typedef struct {
        double time_stamp;

        double s;
        double ds;
        double dds;

        double n;
        double dn;
        double ddn;

        double mu;
        double dmu;
        double ddmu;
    } FrenetVehicleState;

    typedef struct {
        VehicleState       vehicle_state;
        FrenetVehicleState frenet_vehicle_state;
    } StartState;
} // namespace autoku_types

#endif // __INTERFACE_VEHICLE_STATE_HPP__
