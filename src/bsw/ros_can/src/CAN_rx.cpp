#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "ros_can/kusv_CanInfo.h"
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <math.h>
#include <iostream>

// ------------------------------------------------------
// Define & Macro

// Angle conversion
#define DEG2RAD	                0.01745329251199
#define RAD2DEG                 57.2957795131
#define KPH2MPS                 0.27777777777777
#define MPS2KPH                 3.6

// CAN ID
#define CAN_ID_GWAY1		0x100
#define CAN_ID_GWAY2		0x220 //ESP12
#define CAN_ID_GWAY3		0x316 //EMS11
#define CAN_ID_GWAY4		0x386 //WHL_SPD11
#define CAN_ID_GWAY5		0x2B0 //SAS11
#define CAN_ID_GWAY6        0x444


// CAN structure Define

#pragma pack(push, 1)
//-------------- CAN GW

typedef union _CAN_MSG_GWAY1_
{

}CAN_MSG_GWAY1;

typedef union _CAN_MSG_GWAY2_
{
    uint8_t CAN_GWAY2_Data[8];
    struct{
        uint8_t Gway_Lat_Accel_L;
        uint8_t Gway_Lat_Accel_H            	    : 3;
        uint8_t reserve1                                : 1;
        uint8_t reserve2                                : 1;
        uint8_t Gway_Long_Accel_L                   : 3;
        uint8_t Gway_Long_Accel_H;                  
        uint8_t reserve3                                : 1;
        uint8_t reserve4                                : 1;
        uint8_t Gway_Cyl_Pres_L                     : 6;
        uint8_t Gway_Cyl_Pres_H                     : 6;
        uint8_t reserve5                                : 1;
        uint8_t reserve6                                : 1;
        uint8_t Gway_Yaw_Rate_L;                     
        uint8_t Gway_Yaw_Rate_H                     : 5;
        uint8_t reserve7                                : 1;
        uint8_t reserve8                                : 1;
        uint8_t reserve9                                : 1;
        uint8_t reserve10                                : 4;
        uint8_t reserve11                                : 4;
    }Str;

}CAN_MSG_GWAY2;

typedef union _CAN_MSG_GWAY3_
{
     uint8_t CAN_GWAY3_Data[8];
    struct{
        uint8_t SWI_IGK                                 : 1;
        uint8_t F_N_ENG                           	    : 1;
        uint8_t ACK_TCS                                 : 1;
        uint8_t PUC_STAT                                : 1;
        uint8_t TQ_COR_STAT                             : 2;
        uint8_t RLY_AC                                  : 1;                  
        uint8_t F_SUB_TQI                               : 1;
        uint8_t TQI_ACOR;
        uint8_t N_L;
        uint8_t N_H;
        uint8_t TQI;
        uint8_t TQFR;
        uint8_t VS;
        uint8_t RATIO_TQI_BAS_MAX_STND;
    }Str;


}CAN_MSG_GWAY3;

typedef union _CAN_MSG_GWAY4_
{
    uint8_t CAN_GWAY4_Data[8];
    struct{
        uint8_t Gway_Wheel_Velocity_FL_L;
        uint8_t Gway_Wheel_Velocity_FL_H        	: 6;
        uint8_t reserve1                                : 2;
        uint8_t Gway_Wheel_Velocity_FR_L;
        uint8_t Gway_Wheel_Velocity_FR_H        	: 6;
        uint8_t reserve2                                : 2;
        uint8_t Gway_Wheel_Velocity_RL_L;
        uint8_t Gway_Wheel_Velocity_RL_H                : 6;
        uint8_t reserve3                                : 2;
        uint8_t Gway_Wheel_Velocity_RR_L;
        uint8_t Gway_Wheel_Velocity_RR_H                : 6;
        uint8_t reserve4                                : 2;
    }Str;

}CAN_MSG_GWAY4;

typedef union _CAN_MSG_GWAY5_
{
    uint8_t CAN_GWAY5_Data[8]; //CAN_GWAY5_Data[5]
    struct{
        uint8_t SAS_Angle_L;
        uint8_t SAS_Angle_H;        	
        uint8_t SAS_Speed;                                
        uint8_t SAS_Stat;
        uint8_t MsgCount                        	: 4;
        uint8_t CheckSum                                : 4;
    }Str;


}CAN_MSG_GWAY5;

#pragma pack(pop)



// ------------------------------------------------------
// Variables

// CAN message
static CAN_MSG_GWAY1 gway1;
static CAN_MSG_GWAY2 gway2;
static CAN_MSG_GWAY3 gway3;
static CAN_MSG_GWAY4 gway4;
static CAN_MSG_GWAY5 gway5;


// VARIABLE & SIGNAL NAME //

static double st_dLatAcc, st_dLonAcc, st_dYawRate;

static double st_dFL, st_dFR, st_dRL, st_dRR;

static double st_dVel;

static double st_dStrWhlAng, st_dStrWhlSpd;

static double st_dGTestSPD, st_dGTestAccX, st_dGTestAccY, st_dGTestAccZ, st_dGTestGyro;




void canmsg_Callback(const can_msgs::Frame::ConstPtr& L_msg)
{
    // ROS_INFO("canmsg_Callback");
    uint8_t test_buf[8];

    if (L_msg->id <= CAN_ID_GWAY6 && L_msg->id >= CAN_ID_GWAY1)
    {
        ROS_INFO("[%d] ID: 0x%x", L_msg->header.seq, L_msg->id);
        switch(L_msg->id)
        {
        case CAN_ID_GWAY1:
            //memcpy(gway1.CAN_GWAY1_Data, &L_msg->data[0], sizeof(test_buf));

            break;

        case CAN_ID_GWAY2:
            memcpy(gway2.CAN_GWAY2_Data, &L_msg->data[0], sizeof(test_buf));

            st_dLatAcc = (double)((int16_t)((gway2.Str.Gway_Lat_Accel_H << 8) + gway2.Str.Gway_Lat_Accel_L));
            st_dLatAcc = st_dLatAcc * (0.01) - 10.23;
            st_dLonAcc = (double)((int16_t)((gway2.Str.Gway_Long_Accel_H << 3) + gway2.Str.Gway_Long_Accel_L));
            st_dLonAcc = st_dLonAcc * (0.01) - 10.23;
            st_dYawRate =  (double)((int16_t)((gway2.Str.Gway_Yaw_Rate_H << 8) + gway2.Str.Gway_Yaw_Rate_L));
            st_dYawRate = st_dYawRate * (0.01) - 40.95;
            //ROS_INFO("latAcc: %f longAcc: %f, yawrate: %f", st_dLatAcc, st_dLonAcc, st_dYawRate);
            
            break;


        case CAN_ID_GWAY4:
            memcpy(gway4.CAN_GWAY4_Data, &L_msg->data[0], sizeof(test_buf));
            
            st_dFL = (double)((int16_t)((gway4.Str.Gway_Wheel_Velocity_FL_H << 8) + gway4.Str.Gway_Wheel_Velocity_FL_L));
            st_dFL = st_dFL * (0.03125);
            st_dFR = (double)((int16_t)((gway4.Str.Gway_Wheel_Velocity_FR_H << 8) + gway4.Str.Gway_Wheel_Velocity_FR_L));
            st_dFR = st_dFR * (0.03125);
            st_dRL = (double)((int16_t)((gway4.Str.Gway_Wheel_Velocity_RL_H << 8) + gway4.Str.Gway_Wheel_Velocity_RL_L));
            st_dRL = st_dRL * (0.03125);
            st_dRR = (double)((int16_t)((gway4.Str.Gway_Wheel_Velocity_RR_H << 8) + gway4.Str.Gway_Wheel_Velocity_RR_L));
            st_dRR = st_dRR * (0.03125);

            //ROS_INFO("GW1:%X %X %X %X %X %X %X %X",gway4.CAN_GWAY4_Data[0],gway4.CAN_GWAY4_Data[1],gway4.CAN_GWAY4_Data[2],gway4.CAN_GWAY4_Data[3],gway4.CAN_GWAY4_Data[4],gway4.CAN_GWAY4_Data[5],gway4.CAN_GWAY4_Data[6],gway4.CAN_GWAY4_Data[7]);
            //ROS_INFO("FRspd: %f\n RLspd: %f\n RRspd: %f\n FLspd: %f", st_dFR, st_dRL, st_dRR, st_dFL);


            break;

        case CAN_ID_GWAY5:
            memcpy(gway5.CAN_GWAY5_Data, &L_msg->data[0], sizeof(test_buf));
            
            st_dStrWhlAng = (double)((int16_t)((gway5.Str.SAS_Angle_H << 8) + gway5.Str.SAS_Angle_L));
            st_dStrWhlAng = st_dStrWhlAng * (0.1);
            st_dStrWhlSpd = (double)((int8_t)(gway5.Str.SAS_Speed));

            //ROS_INFO("StrWhlAng: %f\n StrWhlSpd: %f", st_dStrWhlAng, st_dStrWhlSpd);

            break;
        }
    }
}


void CAN_Info_Publish(ros::NodeHandle n, ros::Publisher pub)
{
    ros_can::kusv_CanInfo msg_CAN_Signal;

    msg_CAN_Signal.header.stamp = ros::Time::now();
    msg_CAN_Signal.speedfl = st_dFL;
    msg_CAN_Signal.speedfr = st_dFR;
    msg_CAN_Signal.speedrl = st_dRL;
    msg_CAN_Signal.speedrr = st_dRR;
    msg_CAN_Signal.speed_avr_r = (st_dRL + st_dRR)/2;

    msg_CAN_Signal.vehicle_speed_engine = st_dVel;
    msg_CAN_Signal.yaw_rate = st_dYawRate;
    msg_CAN_Signal.lat_acc_speed = st_dLatAcc;
    msg_CAN_Signal.lon_acc_speed = st_dLonAcc;

    msg_CAN_Signal.steering_wheel_angle = st_dStrWhlAng;
    msg_CAN_Signal.steering_wheel_angular = st_dStrWhlSpd;

    pub.publish(msg_CAN_Signal);
}
void CAN_Info_Geo_Publish(ros::NodeHandle n, ros::Publisher pub)
{
    geometry_msgs::TwistStamped msg_CAN_Signal;

    msg_CAN_Signal.header.stamp = ros::Time::now();
    msg_CAN_Signal.twist.linear.x = (st_dRL + st_dRR)/2.*KPH2MPS;
    msg_CAN_Signal.twist.angular.z = st_dYawRate*DEG2RAD;

    pub.publish(msg_CAN_Signal);
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "CAN_RX");
    ros::NodeHandle nh_pub;
    ros::NodeHandle nh_sub;

    ros::Publisher  ros_can_pub     = nh_pub.advertise<ros_can::kusv_CanInfo>("kusv_CanInfo", 100);
    ros::Publisher  ros_can_geo_pub = nh_pub.advertise<geometry_msgs::TwistStamped>("kusv_CanInfo_geo_msg", 100);
    ros::Subscriber ros_can_sub     = nh_sub.subscribe("/can_rx1", 500, canmsg_Callback);                // can Rx setting (In person side)

    ros::AsyncSpinner spinner(4);                                                                  // use 3 threads 1. can tx, 2. can rx, 3. Keyboard_Interface  // can rx2

    ros::Rate loop_rate(100);                                                                      // Looping 100Hz
    spinner.start();

    while(ros::ok())
    {
            ros::spinOnce();
            CAN_Info_Publish(nh_pub, ros_can_pub);
            CAN_Info_Geo_Publish(nh_pub, ros_can_geo_pub);
            loop_rate.sleep();
    }
    return 0;
}
