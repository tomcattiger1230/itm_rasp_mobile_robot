/*
 * @Author: Wei Luo
 * @Date: 2022-09-09 13:44:48
 * @LastEditors: Wei Luo
 * @LastEditTime: 2022-09-09 23:53:36
 * @Note: Note
 */


#ifndef _ROS2_UART_CONTROL_HPP_
#define _ROS2_UART_CONTROL_HPP_

// ros2
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>

// uart
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART


class UartControl: public rclcpp::Node{
    public:
    UartControl();
    ~UartControl();

    private:

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
    std::string port_name_;
    int serial_port;
};

#endif