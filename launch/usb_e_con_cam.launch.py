#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2022-09-11 14:36:53
LastEditors: Wei Luo
LastEditTime: 2022-09-13 15:29:53
Note: Note
'''

import argparse
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import sys


def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-n',
                        '--node-name',
                        dest='node_name',
                        type=str,
                        help='name for device',
                        default='usb_cam')

    args, unknown = parser.parse_known_args(sys.argv[4:])

    usb_cam_dir = get_package_share_directory('itm_rasp_mobile_robot')

    # get path to params file
    params_path = os.path.join(usb_cam_dir, 'config', 'e_con_cam.yaml')

    node_name = args.node_name

    print(params_path)
    ld.add_action(
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            name=node_name,
            # namespace=ns,
            parameters=[params_path]))
    # ld.add_action(
    #     Node(
    #         package='usb_cam',
    #         executable='show_image.py',
    #         output='screen',
    #         # namespace=ns,
    #         # arguments=[image_manip_dir + "/data/mosaic.jpg"])
    #         # remappings=[('image_in', 'image_raw')]
    #     ))

    return ld
