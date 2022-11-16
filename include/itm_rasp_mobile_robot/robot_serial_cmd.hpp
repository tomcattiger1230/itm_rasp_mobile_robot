/*
 * @Author: Wei Luo
 * @Date: 2022-11-16 16:26:53
 * @LastEditors: Wei Luo
 * @LastEditTime: 2022-11-16 16:33:05
 * @Note: Note
 */

#ifndef _ROBOT_SERIAL_CMD_HPP_
#define _ROBOT_SERIAL_CMD_HPP_

#include <ros/ros.h>
#include <serial/serial.h>

class SerialCMD
{
public:
    SerialCMD(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
    ~SerialCMD() = default;

private:
    /* ROS related */
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber robot_cmd_sub_;
};
#endif