/****************************************************************************/
// Module:      rosbridge_vehicle_state.hpp
// Description: ROS bridge for vehicle state
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
// Version: 0.1
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
/****************************************************************************/

#ifndef __ROSBRIDGE_VEHICLE_STATE__
#define __ROSBRIDGE_VEHICLE_STATE__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <carmaker_msgs/UAQ_Out.h>
#include <autoku_msgs/VehicleState.h>
#include <autoku_msgs/VehicleCAN.h>
#include <autoku_msgs/ThreeSecsCAN.h>
#include <autoku_msgs/BrakeTempCAN.h>

// Interface Header
#include "rosbridge_time.hpp"
#include "interface_constants.hpp"
#include "interface_vehicle_state.hpp"

using namespace autoku_types;

namespace autoku_functions{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    VehicleState GetCMUAQOut(const carmaker_msgs::UAQ_Out& msg) {
        carmaker_msgs::UAQ_Out i_uaq_out = msg;

        VehicleState vehicle_state;
        vehicle_state.time_stamp = GetTimeStamp(i_uaq_out.time);
        vehicle_state.quality = NovatelPosType::INS_RTKFIXED;
        
        vehicle_state.vx = i_uaq_out.Sensor_Inertial_0_Vel_B_x;
        vehicle_state.vy = i_uaq_out.Sensor_Inertial_0_Vel_B_y;
        vehicle_state.vz = i_uaq_out.Sensor_Inertial_0_Vel_B_z;
        
        vehicle_state.ax = i_uaq_out.Sensor_Inertial_0_Acc_B_x;
        vehicle_state.ay = i_uaq_out.Sensor_Inertial_0_Acc_B_y;
        vehicle_state.az = i_uaq_out.Sensor_Inertial_0_Acc_B_z;

        vehicle_state.roll = i_uaq_out.Car_Roll;
        vehicle_state.pitch = i_uaq_out.Car_Pitch;
        vehicle_state.yaw = tfNormalizeAngle(i_uaq_out.Car_Yaw);

        vehicle_state.roll_vel = i_uaq_out.Sensor_Inertial_0_Omega_B_x;
        vehicle_state.pitch_vel = i_uaq_out.Sensor_Inertial_0_Omega_B_y;
        vehicle_state.yaw_vel = i_uaq_out.Sensor_Inertial_0_Omega_B_z;

        vehicle_state.lateral_accel = i_uaq_out.Sensor_Inertial_0_Acc_B_y;
        vehicle_state.longitudinal_accel = i_uaq_out.Sensor_Inertial_0_Acc_B_x;
        vehicle_state.yaw_rate = i_uaq_out.Sensor_Inertial_0_Omega_B_z;
                
        vehicle_state.steering_wheel_angle = i_uaq_out.Steer_WhlAng;
        vehicle_state.steering_tire_angle = i_uaq_out.Steer_WhlAng/STEERING_RATIO_CARMAKER;//(i_uaq_out.Vhcl_FL_rz + i_uaq_out.Vhcl_FR_rz)/2.0;
        vehicle_state.steering_speed = 0.0; // TODO
        vehicle_state.steering_torque = 0.0; // TODO
        vehicle_state.steering_state = MdpsState::MDPS_ACTIVATE;

        vehicle_state.wheel_vel_fl = 0.0; // TODO
        vehicle_state.wheel_vel_fr = 0.0; // TODO
        vehicle_state.wheel_vel_rl = 0.0; // TODO
        vehicle_state.wheel_vel_rr = 0.0; // TODO

        vehicle_state.motor_tq_f = 0.0; // TODO
        vehicle_state.motor_tq_r = 0.0; // TODO
        vehicle_state.motor_tq_total = 0.0; // TODO

        vehicle_state.accel_position = i_uaq_out.VC_Gas;
        vehicle_state.brake_pressure = i_uaq_out.VC_Brake;
        vehicle_state.brake_active = 0.0; // TODO
        vehicle_state.brake_temperature = 0.0; // TODO
        vehicle_state.brake_state = BrakeState::BRAKE_OK; // TODO
        vehicle_state.gear_select = Gear::GEAR_D; // TODO

        vehicle_state.operation_mode = OperationMode::AUTONOMOUS;
        vehicle_state.lateral_autonomous_mode = AutonomousMode::RUN;
        vehicle_state.longitudinal_autonomous_mode = AutonomousMode::RUN;

        return vehicle_state;
    }

    VehicleCAN GetVehicleCAN(const autoku_msgs::VehicleCAN& msg) {
        autoku_msgs::VehicleCAN i_vehicle_can = msg;

        VehicleCAN vehicle_can;
        vehicle_can.time_stamp = GetTimeStamp(i_vehicle_can.header.stamp);

        vehicle_can.cluster_odometer = i_vehicle_can.cluster_odometer;

        vehicle_can.lateral_accel = i_vehicle_can.lateral_accel;
        vehicle_can.longitudinal_accel = i_vehicle_can.longitudinal_accel;
        vehicle_can.yaw_rate = i_vehicle_can.yaw_rate;

        vehicle_can.steering_wheel_angle = i_vehicle_can.steering_wheel_angle;
        vehicle_can.steering_tire_angle = i_vehicle_can.steering_tire_angle;
        vehicle_can.steering_speed = i_vehicle_can.steering_speed;
        vehicle_can.steering_torque = i_vehicle_can.steering_torque;
        vehicle_can.steering_state = (MdpsState)i_vehicle_can.steering_state;

        vehicle_can.wheel_velocity_fl = i_vehicle_can.wheel_velocity_fl;
        vehicle_can.wheel_velocity_fr = i_vehicle_can.wheel_velocity_fr;
        vehicle_can.wheel_velocity_rl = i_vehicle_can.wheel_velocity_rl;
        vehicle_can.wheel_velocity_rr = i_vehicle_can.wheel_velocity_rr;
        vehicle_can.wheel_velocity_f_avg = i_vehicle_can.wheel_velocity_f_avg;
        vehicle_can.wheel_velocity_r_avg = i_vehicle_can.wheel_velocity_r_avg;
        vehicle_can.wheel_velocity_avg = i_vehicle_can.wheel_velocity_avg;

        vehicle_can.motor_torque_f = i_vehicle_can.motor_torque_f;
        vehicle_can.motor_torque_r = i_vehicle_can.motor_torque_r;

        vehicle_can.accel_position = i_vehicle_can.accel_position;
        vehicle_can.brake_pressure = i_vehicle_can.brake_pressure;
        vehicle_can.brake_active = i_vehicle_can.brake_active;
        vehicle_can.gear_select = (Gear)i_vehicle_can.gear_select;

        vehicle_can.operation_mode = (OperationMode)i_vehicle_can.operation_mode;
        vehicle_can.lateral_autonomous_mode = (AutonomousMode)i_vehicle_can.lateral_autonomous_mode;
        vehicle_can.longitudinal_autonomous_mode = (AutonomousMode)i_vehicle_can.longitudinal_autonomous_mode;

        return vehicle_can;
    }

    ThreeSecsCAN GetThreeSecsCAN(const autoku_msgs::ThreeSecsCAN& msg) {
        autoku_msgs::ThreeSecsCAN i_three_secs_can = msg;

        ThreeSecsCAN three_secs_can;
        three_secs_can.time_stamp = GetTimeStamp(i_three_secs_can.header.stamp);

        three_secs_can.e_flag       = (EFlag)i_three_secs_can.e_flag;

        three_secs_can.aim          = i_three_secs_can.aim;
        three_secs_can.autoku_r     = i_three_secs_can.autoku_r;
        three_secs_can.eurecar_r    = i_three_secs_can.eurecar_r;
        three_secs_can.kat          = i_three_secs_can.kat;
        three_secs_can.save         = i_three_secs_can.save;
        three_secs_can.tayo         = i_three_secs_can.tayo;

        return three_secs_can;
    }

    BrakeTempCAN GetBrakeTempCAN(const autoku_msgs::BrakeTempCAN& msg) {
        autoku_msgs::BrakeTempCAN i_brake_temp_can = msg;

        BrakeTempCAN brake_temp_can;
        brake_temp_can.time_stamp = GetTimeStamp(i_brake_temp_can.header.stamp);

        brake_temp_can.brake_temp_ch1 = i_brake_temp_can.brake_temp_ch1;
        brake_temp_can.brake_temp_ch2 = i_brake_temp_can.brake_temp_ch2;
        brake_temp_can.brake_temp_ch3 = i_brake_temp_can.brake_temp_ch3;
        brake_temp_can.brake_temp_ch4 = i_brake_temp_can.brake_temp_ch4;
        brake_temp_can.brake_temp_ch5 = i_brake_temp_can.brake_temp_ch5;
        brake_temp_can.brake_temp_ch6 = i_brake_temp_can.brake_temp_ch6;
        brake_temp_can.brake_temp_ch7 = i_brake_temp_can.brake_temp_ch7;
        brake_temp_can.brake_temp_ch8 = i_brake_temp_can.brake_temp_ch8;

        return brake_temp_can;
    }

    VehicleState GetVehicleState(const autoku_msgs::VehicleState& msg) {
        autoku_msgs::VehicleState i_vehicle_state = msg;

        VehicleState vehicle_state;
        vehicle_state.time_stamp = GetTimeStamp(i_vehicle_state.header.stamp);

        vehicle_state.quality = (NovatelPosType)i_vehicle_state.quality;
        vehicle_state.fault_state = i_vehicle_state.fault_state;

        vehicle_state.x = i_vehicle_state.x;
        vehicle_state.y = i_vehicle_state.y;
        vehicle_state.z = i_vehicle_state.z;

        vehicle_state.vx = i_vehicle_state.vx;
        vehicle_state.vy = i_vehicle_state.vy;
        vehicle_state.vz = i_vehicle_state.vz;

        vehicle_state.ax = i_vehicle_state.ax;
        vehicle_state.ay = i_vehicle_state.ay;
        vehicle_state.az = i_vehicle_state.az;

        vehicle_state.roll = i_vehicle_state.roll;
        vehicle_state.pitch = i_vehicle_state.pitch;
        vehicle_state.yaw = i_vehicle_state.yaw;

        vehicle_state.roll_vel = i_vehicle_state.roll_vel;
        vehicle_state.pitch_vel = i_vehicle_state.pitch_vel;
        vehicle_state.yaw_vel = i_vehicle_state.yaw_vel;

        vehicle_state.slope = i_vehicle_state.slope;

        vehicle_state.brake_state = (BrakeState)i_vehicle_state.brake_state;
        vehicle_state.brake_temperature = i_vehicle_state.brake_temperature;

        vehicle_state.lateral_accel = i_vehicle_state.lateral_accel;
        vehicle_state.longitudinal_accel = i_vehicle_state.longitudinal_accel;
        vehicle_state.yaw_rate = i_vehicle_state.yaw_rate;

        vehicle_state.steering_wheel_angle = i_vehicle_state.steering_wheel_angle;
        vehicle_state.steering_tire_angle = i_vehicle_state.steering_tire_angle;
        vehicle_state.steering_speed = i_vehicle_state.steering_speed;
        vehicle_state.steering_torque = i_vehicle_state.steering_torque;
        vehicle_state.steering_state = (MdpsState)i_vehicle_state.steering_state;

        vehicle_state.wheel_vel_fl = i_vehicle_state.wheel_vel_fl;
        vehicle_state.wheel_vel_fr = i_vehicle_state.wheel_vel_fr;
        vehicle_state.wheel_vel_rl = i_vehicle_state.wheel_vel_rl;
        vehicle_state.wheel_vel_rr = i_vehicle_state.wheel_vel_rr;

        vehicle_state.motor_tq_f = i_vehicle_state.motor_tq_f;
        vehicle_state.motor_tq_r = i_vehicle_state.motor_tq_r;
        vehicle_state.motor_tq_total = i_vehicle_state.motor_tq_f + i_vehicle_state.motor_tq_r;

        vehicle_state.accel_position = i_vehicle_state.accel_position;
        vehicle_state.brake_pressure = i_vehicle_state.brake_pressure;
        vehicle_state.brake_active = i_vehicle_state.brake_active;
        vehicle_state.gear_select = (Gear)i_vehicle_state.gear_select;

        vehicle_state.operation_mode = (OperationMode)i_vehicle_state.operation_mode;
        vehicle_state.lateral_autonomous_mode = (AutonomousMode)i_vehicle_state.lateral_autonomous_mode;     
        vehicle_state.longitudinal_autonomous_mode = (AutonomousMode)i_vehicle_state.longitudinal_autonomous_mode;    

        return vehicle_state;
    }
} // namespace autoku_functions

#endif  // __ROSBRIDGE_VEHICLE_STATE__