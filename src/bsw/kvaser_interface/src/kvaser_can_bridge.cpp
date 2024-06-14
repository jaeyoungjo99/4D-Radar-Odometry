/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>

#include <ros/ros.h>
#include <kvaser_interface/kvaser_interface.h>
#include <can_msgs/Frame.h>
#include <autoku_msgs/FrameFD.h>

using namespace AS::CAN;

int bit_rate = 500000;
int hardware_id = 0;
int circuit_id = 0;
int tseg1 = 0;
int tseg2 = 0;
int sjw = 0;
int data_bit_rate = 2000000;
bool is_canfd = false;
bool is_opened = false;

KvaserCan can_driver;
std::mutex mutex_driver;

ros::Publisher can_rx_pub;
ros::Subscriber can_tx_sub;

void can_read()
{
  ReturnStatuses ret;

  while (true)
  {
    if (!is_opened)
    {
      if (is_canfd)
        ret = can_driver.open(hardware_id, circuit_id, bit_rate, data_bit_rate, tseg1, tseg2, sjw, false);
      else
        ret = can_driver.open(hardware_id, circuit_id, bit_rate, false);

      if (ret != ReturnStatuses::OK)
      {
        ROS_ERROR_THROTTLE(0.5, "Kvaser CAN Interface - Error opening reader: %d - %s",
          static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
        break;
      }
    }

    if (is_opened)
    {
      CanMsg msg;

      mutex_driver.lock();
      ret = can_driver.read(&msg);
      mutex_driver.unlock();      
      

      if (ret  == ReturnStatuses::OK)
      {
        // Only publish if msg is not CAN FD,
        // a wakeup message, a transmit acknowledgement,
        // a transmit request, a delay notification,
        // or a failed single-shot.
        if (!(msg.flags.wakeup_mode ||
              msg.flags.tx_ack ||
              msg.flags.tx_rq ||
              msg.flags.msg_delayed ||
              msg.flags.tx_nack))
        {
          if (is_canfd) {
            autoku_msgs::FrameFD can_pub_msg;
            can_pub_msg.header.frame_id = "0";
            can_pub_msg.id = msg.id;
            can_pub_msg.dlc = msg.dlc;
            can_pub_msg.is_extended = msg.flags.ext_id;
            can_pub_msg.is_error = msg.flags.error_frame;
            can_pub_msg.is_rtr = msg.flags.rtr;
            for (uint16_t i = 0; i < (uint16_t)msg.data.size(); i++) {
              can_pub_msg.data.push_back(msg.data.at(i));
            }
            can_pub_msg.header.stamp = ros::Time::now();
            can_rx_pub.publish(can_pub_msg);
          } else {
            can_msgs::Frame can_pub_msg;
            can_pub_msg.header.frame_id = "0";
            can_pub_msg.id = msg.id;
            can_pub_msg.dlc = msg.dlc;
            can_pub_msg.is_extended = msg.flags.ext_id;
            can_pub_msg.is_error = msg.flags.error_frame;
            can_pub_msg.is_rtr = msg.flags.rtr;
            std::copy(msg.data.begin(), msg.data.end(), can_pub_msg.data.begin());
            can_pub_msg.header.stamp = ros::Time::now();
            can_rx_pub.publish(can_pub_msg);
          }          
        }
      }
      else
      {
        if (ret != ReturnStatuses::NO_MESSAGES_RECEIVED)
          ROS_WARN_THROTTLE(0.5, "Kvaser CAN Interface - Error reading CAN message: %d - %s",
            static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());

        break;
      }
    }
  }
  // mutex_driver.unlock();
}

void can_tx_callback(const can_msgs::Frame::ConstPtr& ros_msg)
{
  ReturnStatuses ret;

  // mutex_driver.lock();
  if (!is_opened)
  {
    // Open the channel.
    if (is_canfd) {
      ret = can_driver.open(hardware_id, circuit_id, bit_rate, data_bit_rate, tseg1, tseg2, sjw, false);
    } else {
      ret = can_driver.open(hardware_id, circuit_id, bit_rate, false);
    }

    if (ret != ReturnStatuses::OK)
    {
      ROS_ERROR_THROTTLE(0.5, "Kvaser CAN Interface - Error opening writer: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
    }
  }

  if (is_opened)
  {
    CanMsg msg;

    msg.id = ros_msg->id;
    msg.dlc = ros_msg->dlc;
    msg.flags.ext_id = ros_msg->is_extended;
    msg.flags.rtr = ros_msg->is_rtr;

    auto msg_size = KvaserCanUtils::dlcToSize(ros_msg->dlc);

    for (size_t i = 0; i < msg_size; ++i)
    {
      msg.data.push_back(ros_msg->data[i]);
    }

    ret = can_driver.write(std::move(msg));
    
    if (ret != ReturnStatuses::OK)
    {
      ROS_WARN_THROTTLE(0.5, "Kvaser CAN Interface - CAN send error: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
    }
  }
}

void canfd_tx_callback(const autoku_msgs::FrameFD::ConstPtr& ros_msg)
{
  ReturnStatuses ret;

  if (!is_opened)
  {
    // Open the channel.
    if (is_canfd) {
      ret = can_driver.open(hardware_id, circuit_id, bit_rate, data_bit_rate, tseg1, tseg2, sjw, false);
    } else {
      ret = can_driver.open(hardware_id, circuit_id, bit_rate, false);
    }

    if (ret != ReturnStatuses::OK)
    {
      ROS_ERROR_THROTTLE(0.5, "Kvaser CAN Interface - Error opening writer: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
    }
  }

  if (is_opened)
  {
    CanMsg msg;

    msg.id = ros_msg->id;
    msg.dlc = ros_msg->dlc;
    msg.flags.ext_id = ros_msg->is_extended;
    msg.flags.rtr = ros_msg->is_rtr;
    msg.flags.fd_msg = ros_msg->is_canfd;

    auto msg_size = KvaserCanUtils::dlcToSize(ros_msg->dlc);

    for (size_t i = 0; i < msg_size; ++i)
    {
      msg.data.push_back(ros_msg->data[i]);
    }

    ret = can_driver.write(std::move(msg));

    if (ret != ReturnStatuses::OK)
    {
      ROS_WARN_THROTTLE(0.5, "Kvaser CAN Interface - CAN send error: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
    }
  }
}

int main(int argc, char** argv)
{
  bool exit = false;

  // ROS initialization
  ros::init(argc, argv, "kvaser_can_bridge");
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  ros::AsyncSpinner spinner(1);

  // Wait for time to be valid
  ros::Time::waitForValid();

  if (priv.getParam("can_hardware_id", hardware_id))
  {
    ROS_INFO("Kvaser CAN Interface - Got hardware_id: %d", hardware_id);

    if (hardware_id < 0)
    {
      ROS_ERROR("Kvaser CAN Interface - CAN hardware ID is invalid.");
      exit = true;
    }
  }

  if (priv.getParam("can_circuit_id", circuit_id))
  {
    ROS_INFO("Kvaser CAN Interface - Got can_circuit_id: %d", circuit_id);

    if (circuit_id < 0)
    {
      ROS_ERROR("Kvaser CAN Interface - Circuit ID is invalid.");
      exit = true;
    }
  }

  if (priv.getParam("can_bit_rate", bit_rate))
  {
    ROS_INFO("Kvaser CAN Interface - Got bit_rate: %d", bit_rate);

    if (bit_rate < 0)
    {
      ROS_ERROR("Kvaser CAN Interface - Bit Rate is invalid.");
      exit = true;
    }
  }

  if (priv.getParam("canfd", is_canfd))
  {
    if (is_canfd)
    {
      if (priv.getParam("canfd_tseg1", tseg1))
      {
        ROS_INFO("Kvaser CANFD Interface - Got tseg1: %d", tseg1);

        if (tseg1 < 0)
        {
          ROS_ERROR("Kvaser CANFD Interface - CANFD TSEG1 is invalid.");
          exit = true;
        }
      }

      if (priv.getParam("canfd_tseg2", tseg2))
      {
        ROS_INFO("Kvaser CANFD Interface - Got tseg2: %d", tseg2);

        if (tseg2 < 0)
        {
          ROS_ERROR("Kvaser CANFD Interface - CANFD hardware ID is invalid.");
          exit = true;
        }
      }

      if (priv.getParam("canfd_sjw", sjw))
      {
        ROS_INFO("Kvaser CANFD Interface - Got sjw: %d", sjw);

        if (sjw < 0)
        {
          ROS_ERROR("Kvaser CANFD Interface - CANFD hardware ID is invalid.");
          exit = true;
        }
      }

      if (priv.getParam("canfd_data_rate", data_bit_rate))
      {
        ROS_INFO("Kvaser CANFD Interface - Got data_bit_rate: %d", data_bit_rate);

        if (data_bit_rate < 0)
        {
          ROS_ERROR("Kvaser CANFD Interface - CANFD hardware ID is invalid.");
          exit = true;
        }
      }
    }
  }

  if (exit)
  {
    ros::shutdown();
    return 0;
  }

  ReturnStatuses ret;

  // Open CAN reader channel
  if (is_canfd) {
    ret = can_driver.open(hardware_id, circuit_id, bit_rate, data_bit_rate, tseg1, tseg2, sjw, false);
  } else {
    ret = can_driver.open(hardware_id, circuit_id, bit_rate, false);
  }

  if (ret == ReturnStatuses::OK)
  {
    ROS_INFO("Kvaser CAN Interface - Open complete");
    is_opened = true;

    // Set up read callback
    ret = can_driver.registerReadCallback(can_read);

    // Register publisher and subscriber
    if (is_canfd) {
      can_rx_pub = n.advertise<autoku_msgs::FrameFD>("can_rx" + std::to_string(circuit_id), 500);
      can_tx_sub = n.subscribe("can_tx" + std::to_string(circuit_id), 500, canfd_tx_callback);
    }
    else {
      can_rx_pub = n.advertise<can_msgs::Frame>("can_rx" + std::to_string(circuit_id), 500);
      can_tx_sub = n.subscribe("can_tx" + std::to_string(circuit_id), 500, can_tx_callback);
    }

    if (ret == ReturnStatuses::OK)
    {
      ROS_INFO("Initialization complete!");

      // Only start spinner if reader initialized OK
      spinner.start();
    }
    else
    {
      ROS_ERROR("Kvaser CAN Interface - Error registering reader callback: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
      ros::shutdown();
      return -1;
    }
  }
  else
  {
    ROS_ERROR("Kvaser CAN Interface - Error opening reader: %d - %s",
      static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
    ros::shutdown();
    return -1;
  }

  ros::waitForShutdown();

  if (can_driver.isOpen())
  {
    ret = can_driver.close();

    if (ret != ReturnStatuses::OK)
      ROS_ERROR("Kvaser CAN Interface - Error closing reader: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
  }

  if (can_driver.isOpen())
  {
    ret = can_driver.close();

    if (ret != ReturnStatuses::OK)
      ROS_ERROR("Kvaser CAN Interface - Error closing writer: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
  }

  return 0;
}
