/*
 * @Author: Wei Luo
 * @Date: 2022-11-16 16:26:53
 * @LastEditors: Wei Luo
 * @LastEditTime: 2022-11-21 08:02:29
 * @Note: Note
 */

#ifndef _ROBOT_SERIAL_CMD_HPP_
#define _ROBOT_SERIAL_CMD_HPP_

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>

#include <geometry_msgs/TwistStamped.h>

class SerialCMD {
public:
  SerialCMD(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
  ~SerialCMD();
  void run();

  bool is_initialized;

private:
  /* ROS related */
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber robot_cmd_sub_;

  std::string serial_port;
  int baud_rate;
  serial::Serial sp;
  uint8_t msg_buffer[10];

  std::vector<double> received_cmd = std::vector<double>(3);
  bool got_velocity_cmd;


  void robot_velocity_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr &v_cmd);
};
#endif