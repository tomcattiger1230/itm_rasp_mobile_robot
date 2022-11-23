#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2022-11-23 14:42:15
LastEditors: Wei Luo
LastEditTime: 2022-11-23 14:55:47
Note: Note
'''

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ld = LaunchDescription()

    rasp_robot_dir = get_package_share_directory('itm_rasp_mobile_robot')

    # get path to params file
    params_path = os.path.join(rasp_robot_dir, 'config', 'ros_cmd_uart.yaml')

    ld.add_action(
        Node(package='itm_rasp_mobile_robot',
             executable='uart_control',
             output='screen',
             name="robot_cmd_uart",
             parameters=[params_path]))

    return ld
