/****************************************************************************/
// Module:      interface_vehicle_can.hpp
// Description: vehicle can interface
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
/****************************************************************************/

#ifndef __INTERFACE_VEHICLE_CAN_HPP__
#define __INTERFACE_VEHICLE_CAN_HPP__
#pragma once

// STD Header
#include <map>

#pragma pack(push, 1)
namespace hmg_ioniq {
    /////////////////////////////////////////////////////////////////
    // CAN ID
    const unsigned int CANID_GWAY1                  = 0x100;
    const unsigned int CANID_ADCMD                  = 0x50;
    const unsigned int CANID_AutoKUCMD              = 0x303;
    const unsigned int CANID_AutoKUSTA              = 0x304;
    const unsigned int CANID_AutoKU_HEALTH          = 0x305;
    const unsigned int CANID_AutoKU_AD_MODE         = 0x306;
    const unsigned int CANID_AutoKU_SIREN_ON_OFF    = 0x307;

    /////////////////////////////////////////////////////////////////
    // CAN DLC (Data Length, Kvaser)
    const unsigned int DLC_GWAY1                    = 32;
    const unsigned int DLC_ADCMD                    = 32;
    const unsigned int DLC_AutoKUCMD                = 12;
    const unsigned int DLC_AutoKUSTA                = 3;
    const unsigned int DLC_AutoKU_HEALTH            = 1;
    const unsigned int DLC_AutoKU_AD_MODE           = 4;
    const unsigned int DLC_AutoKU_SIREN_ON_OFF      = 2;

    /////////////////////////////////////////////////////////////////
    // // CAN DLC (Data Length Code)
    // const unsigned int DLC_GWAY1                    = 0xD;
    // const unsigned int DLC_ADCMD                    = 0xD;
    // const unsigned int DLC_AutoKUCMD                = 0x9;
    // const unsigned int DLC_AutoKUSTA                = 0x3;
    // const unsigned int DLC_AutoKU_HEALTH            = 0x1;
    // const unsigned int DLC_AutoKU_ADMODE            = 0x3;
    // const unsigned int DLC_AutoKU_SIREN_ON_OFF      = 0x2;

    /////////////////////////////////////////////////////////////////
    // CAN FACTOR
    const double FACTOR_TARGET_STEERING             = 0.1;
    const double FACTOR_TARGET_ACCEL                = 0.01;
    const double FACTOR_TARGET_SPEED                = 0.03125;
    const double FACTOR_TARGET_PEDAL_POSITION       = 0.392157;

        
    /////////////////////////////////////////////////////////////////
    // CAN structure Define
    typedef union _External_CAN_GWAY1_ {
        uint8_t data[DLC_GWAY1];                                // 32 bytes (256 bits)
        struct {
            int16_t  Gway_SAS_Angle                     :16;    // 16
            uint8_t  Gway_SAS_Speed                     : 8;    // 24
            uint32_t Gway_Cluster_Odometer              :24;    // 48
            uint16_t Gway_Lateral_Accel_Speed           :16;    // 64
            uint16_t Gway_BrakeCylinder_Pressure        :12;    // 76
            uint8_t  Gway_Steering_Status               : 4;    // 80
            uint16_t Gway_Longitudinal_Accel_Speed      :16;    // 96
            uint16_t Gway_Yaw_Rate_Sensor               :16;    // 112
            uint16_t Gway_Wheel_Velocity_FR             :14;    // 126
            uint8_t  Gway_Brake_Active                  : 2;    // 128
            uint16_t Gway_Wheel_Velocity_RL             :14;    // 142
            uint8_t  reserve1                           : 2;    // 144
            uint16_t Gway_Wheel_Velocity_RR             :14;    // 158
            uint8_t  reserve2                           : 2;    // 160
            uint16_t Gway_Wheel_Velocity_FL             :14;    // 174
            uint8_t  reserve3                           : 2;    // 176
            uint16_t Gway_Steering_Angle                :16;    // 192
            uint16_t Gway_Steering_Tq                   :13;    // 205
            uint8_t  reserve4                           : 3;    // 208
            uint8_t  Gway_Accel_Pedal_Position          : 8;    // 216
            uint8_t  Gway_GearSelDisp                   : 4;    // 220
            uint8_t  reserve5                           : 4;    // 224
            int16_t  F_MCU_Torque                       :14;    // 238
            uint8_t  reserve6                           : 2;    // 240
            int16_t  R_MCU_Torque                       :14;    // 254
            uint8_t  reserve7                           : 2;    // 256
        } str;
    } External_CAN_GWAY1;

    typedef union _External_CAN_ADCMD_ {
        uint8_t data[DLC_ADCMD];                                // 32 bytes (256 bits)
        struct {
            uint32_t reserve1                           :24;    // 24            
            uint8_t  ADCMD_ModeAct                      : 2;    // 26
            uint8_t  reserve2                           : 6;    // 32
            uint8_t  ADCMD_Brk1                         : 8;    // 40
            uint8_t  ADCMD_Brk2V                        : 8;    // 48
            uint16_t ADCMD_AccAps                       :16;    // 64
            uint16_t ADCMD_APS2                         :16;    // 80
            uint8_t  ADCMD_F1                           : 4;    // 84
            uint8_t  ADCMD_F2                           : 4;    // 88
            uint8_t  ADCMD_NO                           : 1;    // 89
            uint8_t  ADCMD_NC                           : 1;    // 90
            uint8_t  ADCMD_Enable                       : 1;    // 91
            uint8_t  ADCMD2                             : 1;    // 92
            uint8_t  ADCMD_ShifterRND                   : 4;    // 96
            uint8_t  ADCMD_ShifterP                     : 2;    // 98
            uint8_t  ADCMD_RButtonStatus                : 2;    // 100
            uint8_t  ADCMD_NButtonStatus                : 2;    // 102
            uint8_t  ADCMD_DButtonStatus                : 2;    // 104
            uint8_t  ADCMD_PButtonStatusReserved        : 2;    // 106
            uint8_t  ADCMD_RButtonStatusReserved        : 2;    // 108
            uint8_t  ADCMD_NButtonStatusReserved        : 2;    // 110
            uint8_t  ADCMD_DButtonStatusReserved        : 2;    // 112
            uint8_t  ADSteer_Mod1                       : 3;    // 115
            uint8_t  ADSteer_Mod2                       : 3;    // 118
            uint8_t  ADSteer_Be                         : 2;    // 120
            uint8_t  ADSteer_Sta                        : 4;    // 124
            uint8_t  ADSteer_TestModSta                 : 2;    // 126
            uint8_t  reserve3                           : 2;    // 128
            int16_t  ADSteer_StrAnglReq                 :16;    // 144
            uint8_t  ADSteer_OptInfo                    : 4;    // 148
            uint16_t reserve4                           :12;    // 160
            uint32_t reserve5                           :32;    // 192
            uint32_t reserve6                           :32;    // 224
            uint32_t reserve7                           :32;    // 256
        } str;
    } External_CAN_ADCMD;
    
    typedef union _AutoKU_CAN_CMD_ {
        uint8_t data[DLC_AutoKUCMD];                            // 12 bytes (96 bits)
        struct {
            int16_t  target_steering_angle              :14;    // 14
            uint8_t  reserve1                           : 2;    // 16
            int16_t  target_acceleration                :12;    // 27
            uint8_t  reserve2                           : 4;    // 32
            uint8_t  target_acc_pedal_pos               : 8;    // 40
            uint8_t  target_brake_pedal_pos             : 8;    // 48
            uint8_t  target_gear                        : 4;    // 52
            uint8_t  reserve3                           : 4;    // 56
            uint16_t target_speed                       :14;    // 70
            uint8_t  reserve4                           : 2;    // 72
            uint32_t reserve5                           :24;    // 96
        } str;
    } AutoKU_CAN_CMD;
    
    typedef union _AutoKU_CAN_STA_ {
        uint8_t data[DLC_AutoKUSTA];                            // 3 bytes (24 bits)
        struct {
            uint8_t  operation_mode                     : 8;    // 8
            uint8_t  lateral_mode                       : 8;    // 16
            uint8_t  longitudinal_mode                  : 8;    // 24
        } str;
    } AutoKU_CAN_STA;
    typedef union _AutoKU_CAN_AD_MODE_
    {
        uint8_t data[DLC_AutoKU_AD_MODE]; // 4 bytes (32 bits)
        struct
        {
            uint8_t operation_mode : 8;    // 8
            uint8_t lateral_mode : 8;      // 16
            uint8_t longitudinal_mode : 8; // 24
            uint8_t manual_mode : 8;       // 32
        } str;
    } AutoKU_CAN_AD_MODE;
    typedef union _AutoKU_CAN_HEALTH_
    {
        uint8_t data[DLC_AutoKU_HEALTH]; // 1 bytes (8 bits)
        struct
        {
            uint8_t health_sequence : 8; // 8
        } str;
    } AutoKU_CAN_HEALTH;
    typedef union _AutoKU_CAN_SIREN_ON_OFF_
    {
        uint8_t data[DLC_AutoKU_SIREN_ON_OFF]; // 2 bytes (16 bits)
        struct
        {
            uint8_t siren_on_off : 8; // 8  | 0 "siren off" 1 "siren on" 2 "siren blink"
            uint8_t led_on_off : 8;   // 16 | 0 "led off" 1 "blue on" 2 "blue blink" 3 "orange on" 4 "orange blink" 5 "blue&orange on" 6 "blue&orange blink" 7 "reserve"
        } str;
    } AutoKU_CAN_SIREN_ON_OFF;
}
namespace three_secs {
    /////////////////////////////////////////////////////////////////
    // CAN ID
    const unsigned int CANID_E_FLAG                 = 0x200;
    
    /////////////////////////////////////////////////////////////////
    // CAN DLC
    const unsigned int DLC_E_FLAG                   = 8;

    /////////////////////////////////////////////////////////////////
    // CAN structure Define
    typedef union _THREE_SECS_E_FLAG_ {
        uint8_t data[DLC_E_FLAG];                               // 8 bytes (64 bits)
        struct {
            uint8_t  e_flag                             : 3;    // 3
            uint16_t reserve1                           :13;    // 16
            uint8_t  aim                                : 3;    // 19
            uint8_t  reserve2                           : 1;    // 20
            uint8_t  autoku_r                           : 3;    // 23
            uint16_t reserve3                           : 9;    // 32
            uint8_t  eurecar_r                          : 3;    // 35
            uint8_t  reserve4                           : 1;    // 36
            uint8_t  kat                                : 3;    // 39
            uint16_t reserve5                           : 9;    // 48
            uint8_t  save                               : 3;    // 51
            uint8_t  reserve6                           : 1;    // 52
            uint8_t  tayo                               : 3;    // 55
            uint16_t reserve7                           : 9;    // 64
        } str;
    } THREE_SECS_E_FLAG;
}

namespace brake_sensor {
    /////////////////////////////////////////////////////////////////
    // CAN ID
    const unsigned int CANID_Brake_Ch1_to_Ch4       = 0x4C4;
    const unsigned int CANID_Brake_Ch5_to_Ch8       = 0x4C5;

    /////////////////////////////////////////////////////////////////
    // CAN DLC    
    const unsigned int DLC_Brake_Ch1_to_Ch4         = 8;
    const unsigned int DLC_Brake_Ch5_to_Ch8         = 8;

    /////////////////////////////////////////////////////////////////
    // CAN structure Define
    
    typedef union _BRAKE_CH1_TO_CH4_ {
        uint8_t data[DLC_Brake_Ch1_to_Ch4];
        struct {
            uint8_t temp_ch1_H;
            uint8_t temp_ch1_L;
            uint8_t temp_ch2_H;
            uint8_t temp_ch2_L;
            uint8_t temp_ch3_H;
            uint8_t temp_ch3_L;
            uint8_t temp_ch4_H;
            uint8_t temp_ch4_L;
        } str;
    } BRAKE_CH1_TO_CH4;
        
    typedef union _BRAKE_CH5_TO_CH8_ {
        uint8_t data[DLC_Brake_Ch5_to_Ch8];
        struct {
            uint8_t temp_ch5_H;
            uint8_t temp_ch5_L;
            uint8_t temp_ch6_H;
            uint8_t temp_ch6_L;
            uint8_t temp_ch7_H;
            uint8_t temp_ch7_L;
            uint8_t temp_ch8_H;
            uint8_t temp_ch8_L;
        } str;
    } BRAKE_CH5_TO_CH8;
}

namespace lab_ioniq {
    /////////////////////////////////////////////////////////////////
    // CAN ID
    const unsigned int CANID_ModeStatus             = 0x0F;
    const unsigned int CANID_DynamicState           = 0x10;
    const unsigned int CANID_WheelState             = 0x11;
    const unsigned int CANID_SteeringState          = 0x12;
    const unsigned int CANID_LongitudinalState      = 0x13;
    const unsigned int CANID_AccessoryState         = 0x14;
    const unsigned int CANID_GearState              = 0x15;
    const unsigned int CANID_LongitudinalControl    = 0x51;
    const unsigned int CANID_LateralControl         = 0x52;
    const unsigned int CANID_AccessoryControl       = 0x53;    
    const unsigned int CANID_AutoKUCMD              = 0x200;
    const unsigned int CANID_AutoKUSTA              = 0x300;
    
    /////////////////////////////////////////////////////////////////
    // CAN DLC    
    const unsigned int DLC_ModeStatus               = 4;
    const unsigned int DLC_DynamicState             = 12;
    const unsigned int DLC_WheelState               = 16;
    const unsigned int DLC_SteeringState            = 7;
    const unsigned int DLC_LongitudinalState        = 3;
    const unsigned int DLC_AccessoryState           = 3;
    const unsigned int DLC_GearState                = 3;
    const unsigned int DLC_LongitudinalControl      = 8;
    const unsigned int DLC_LateralControl           = 7;
    const unsigned int DLC_AccessoryControl         = 3;
    const unsigned int DLC_AutoKUCMD                = 7;
    const unsigned int DLC_AutoKUSTA                = 3;
        
    /////////////////////////////////////////////////////////////////
    // CAN structure Define
    typedef union _VCAN_MODE_STATUS_ {
        uint8_t data[DLC_ModeStatus];
        struct {
            uint16_t reserve0;
            uint8_t operation_mode                      : 1;
            uint8_t autonomous_mode                     : 2;
            uint16_t reserve1                           :13;
        } str;
    } VCAN_MODE_STATUS;

    typedef union _VCAN_DYNAMIC_STATE_ {
        uint8_t data[DLC_DynamicState];
        struct {
            uint32_t reserve0                           :24;
            int16_t longitudinal_acceleration           :12;
            uint8_t reserve1                            : 2;
            int16_t lateral_acceleration                :10;
            int16_t roll_rate                           :10;
            int16_t pitch_rate                          :10;
            int16_t yaw_rate                            :12;
            uint16_t reserve2;
        } str;
    } VCAN_DYNAMIC_STATE;

    typedef union _VCAN_WHEEL_STATE_ {
        uint8_t data[DLC_WheelState];
        struct {
            uint64_t reserve0;
            uint16_t wheel_speed_fl                     :13;
            uint16_t wheel_speed_fr                     :13;
            uint16_t wheel_speed_rl                     :13;
            uint16_t wheel_speed_rr                     :13;
            uint16_t reserve4                           :12;
        } str;
    } VCAN_WHEEL_STATE;

    typedef union _VCAN_STEERING_STATE_ {
        uint8_t data[DLC_SteeringState];
        struct {
            uint16_t reserve0;
            int16_t steering_wheel_angle                      :14;
            int16_t steering_torque                     :10;
            uint16_t reserve1                           :12;
            uint8_t steering_override                   : 1;
            uint16_t reserve2                           :11;
        } str;
    } VCAN_STEERING_STATE;

    typedef union _VCAN_LONGITUDIANL_STATE_ {
        uint8_t data[DLC_LongitudinalState];
        struct {
            uint16_t reserve0;
            uint8_t acceleration_pedal_status           : 1;
            uint8_t brake_pedal_status                  : 1;
            uint8_t reserve1                            : 6;
        } str;
    } VCAN_LONGITUDINAL_STATE;

    typedef union _VCAN_ACCESSORY_STATE_ {
        uint8_t data[DLC_AccessoryState];
        struct {
            uint16_t reserve0;
            uint8_t turn_signal_status                  : 2;
            uint8_t reserve1                            : 6;
        } str;
    } VCAN_ACCESSORY_STATE;

    typedef union _VCAN_GEAR_STATE_ {
        uint8_t data[DLC_GearState];
        struct {
            uint16_t reserve0;
            uint8_t gear_status                         : 3;
            uint8_t gear_override                       : 1;
            uint8_t reserve1                            : 4;
        } str;
    } VCAN_GEAR_STATE;

    typedef union _VCAN_LONGITUDINAL_CONTROL_ {
        uint8_t data[DLC_LongitudinalControl];
        struct {
            uint16_t reserve0;
            uint8_t longitudinal_control_mode           : 3;
            int16_t target_acceleration                 :11;
            uint32_t reserve1                           :31;
            uint8_t target_gear                         : 3;
        } str;
    } VCAN_LONGITUDINAL_CONTROL;

    typedef union _VCAN_LATERAL_CONTROL_ {
        uint8_t data[DLC_LateralControl];
        struct {
            uint16_t reserve0;
            uint8_t steering_control_mode               : 2;
            uint16_t reserve1;
            int16_t target_steering_angle               :14;
            uint16_t reserve2;
        } str;
    } VCAN_LATERAL_CONTROL;

    typedef union _VCAN_ACCESSORY_CONTROL_ {
        uint8_t data[DLC_AccessoryControl];
        struct {
            uint16_t reserve0;
            uint8_t target_turn_signal                  : 2;
            uint8_t reserve1                            : 6;
        } str;
    } VCAN_ACCESSORY_CONTROL;    
    
    typedef union _AutoKU_CAN_CMD_ {
        uint8_t data[DLC_AutoKUCMD];                            // 7 bytes (56 bits)
        struct {
            uint16_t target_steering_angle              :14;    // 14
            uint8_t  reserve1                           : 2;    // 16
            uint16_t target_acceleration                :12;    // 27
            uint8_t  reserve2                           : 4;    // 32
            uint8_t  target_acc_pedal_pos               : 8;    // 40
            uint8_t  target_brake_pedal_pos             : 8;    // 48
            uint8_t  target_gear                        : 4;    // 52
            uint8_t  reserve3                           : 4;    // 56
        } str;
    } AutoKU_CAN_CMD;
    
    typedef union _AutoKU_CAN_STA_ {
        uint8_t data[DLC_AutoKUSTA];                            // 3 bytes (24 bits)
        struct {
            uint8_t  operation_mode                     : 8;    // 8
            uint8_t  lateral_mode                       : 8;    // 16
            uint8_t  longitudinal_mode                  : 8;    // 24
        } str;
    } AutoKU_CAN_STA;

}
#pragma pack(pop)

#endif // __INTERFACE_VEHICLE_CAN_HPP__