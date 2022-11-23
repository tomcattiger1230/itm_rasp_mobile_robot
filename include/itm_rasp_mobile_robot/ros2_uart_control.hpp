/*
 * @Author: Wei Luo
 * @Date: 2022-09-09 13:44:48
 * @LastEditors: Wei Luo
 * @LastEditTime: 2022-11-23 14:24:53
 * @Note: Note
 */

#ifndef _ROS2_UART_CONTROL_HPP_
#define _ROS2_UART_CONTROL_HPP_

// ros2
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <serial/serial.h>

#include <iostream>
#include <stdio.h>
#include <vector>

class UartControl : public rclcpp::Node {
public:
  UartControl();
  ~UartControl();
  void run();
  bool is_initialized;

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  std::string port_name_;
  int baud_rate;
  std::string cmd_topic_name;

  serial::Serial sp;
  std::vector<double> received_cmd = std::vector<double>(3);
  bool got_velocity_cmd;
  void robot_velocity_cmd_callback(
      const geometry_msgs::msg::TwistStamped::SharedPtr msg);
};

#endif