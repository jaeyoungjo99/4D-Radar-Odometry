#include <ros/ros.h>
// #include <can_msgs/Frame.h>
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

static double st_dLatAcc, st_dLonAcc, st_dYawRate;

static double st_dFL, st_dFR, st_dRL, st_dRR;

static double st_dVel;

static double st_dStrWhlAng, st_dStrWhlSpd;

static double st_dGTestSPD, st_dGTestAccX, st_dGTestAccY, st_dGTestAccZ, st_dGTestGyro;

ros::Time kusv_header_time;

void kusv_can_callback(const ros_can::kusv_CanInfo::ConstPtr& msg){
    st_dRL = msg->speedrl;
    st_dRR = msg->speedrr;
    st_dYawRate = msg->yaw_rate;
    kusv_header_time = msg->header.stamp;
    ROS_INFO("Callback kusv_CanInfo");
}

void CAN_Info_Geo_Publish(ros::NodeHandle n, ros::Publisher pub)
{
    geometry_msgs::TwistStamped msg_CAN_Signal;

    msg_CAN_Signal.header.stamp = kusv_header_time;
    msg_CAN_Signal.twist.linear.x = (st_dRL + st_dRR)/2.*KPH2MPS;
    msg_CAN_Signal.twist.angular.z = st_dYawRate*DEG2RAD;

    pub.publish(msg_CAN_Signal);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "CAN_RX");
    ros::NodeHandle nh_pub;
    ros::NodeHandle nh_sub;

    // ros::Publisher  ros_can_pub     = nh_pub.advertise<ros_can::kusv_CanInfo>("kusv_CanInfo", 100);
    ros::Publisher  ros_can_geo_pub = nh_pub.advertise<geometry_msgs::TwistStamped>("kusv_CanInfo_geo_msg", 100);
    ros::Subscriber ros_can_sub     = nh_sub.subscribe("/kusv_CanInfo", 500, kusv_can_callback);                // can Rx setting (In person side)

    ros::AsyncSpinner spinner(4);                                                                  // use 3 threads 1. can tx, 2. can rx, 3. Keyboard_Interface  // can rx2

    ros::Rate loop_rate(100);                                                                      // Looping 100Hz
    spinner.start();

    while(ros::ok())
    {
            ros::spinOnce();
            CAN_Info_Geo_Publish(nh_pub, ros_can_geo_pub);
            loop_rate.sleep();
    }
    return 0;
}
