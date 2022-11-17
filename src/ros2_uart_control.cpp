/*
 * @Author: Wei Luo
 * @Date: 2022-09-08 14:31:07
 * @LastEditors: Wei Luo
 * @LastEditTime: 2022-11-17 13:15:30
 * @Note: this cpp is to set up UART communication with BeagleBone onboard computer
 */

#include <itm_rasp_mobile_robot/ros2_uart_control.hpp>

UartControl::UartControl(): Node("uart_control_node")
{
    this->declare_parameter<std::string>("port", "/dev/ttyS0");
    this->get_parameter("port", port_name_);


}

UartControl::~UartControl(){}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Rate rate(20.0);

    auto uart_control_handle = std::make_shared<UartControl>();
}