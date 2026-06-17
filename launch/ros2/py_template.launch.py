#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # py_template
    template_param_path = FindPackageShare('hex_ros_sim_archer_y6').find(
        'hex_ros_sim_archer_y6') + '/config/ros2/params.yaml'
    template_node = Node(package='hex_ros_sim_archer_y6',
                         executable='py_template',
                         name='py_template',
                         output="screen",
                         emulate_tty=True,
                         parameters=[template_param_path],
                         remappings=[
                             ('in_str', 'in_str'),
                             ('in_int', 'in_int'),
                             ('out_str', 'out_str'),
                             ('out_int', 'out_int'),
                         ])

    return LaunchDescription([
        template_node,
    ])
