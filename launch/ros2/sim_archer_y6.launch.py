#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-17
################################################################

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_pkg_path = FindPackageShare("hex_ros_urdf_archer_y6")
    sim_pkg_path = FindPackageShare("hex_ros_sim_archer_y6")

    # arg
    viewer_arg = DeclareLaunchArgument(
        name='viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to turn on mujoco viewer')
    rviz_arg = DeclareLaunchArgument(name='rviz',
                                     default_value='true',
                                     choices=['true', 'false'],
                                     description='Flag to turn on rviz')
    test_arg = DeclareLaunchArgument(
        name='test',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to turn on test ctrl node')

    # sim node
    sim_param_path = PathJoinSubstitution(
        [sim_pkg_path, "config", "ros2", "params.yaml"])
    sim_mjcf_path = PathJoinSubstitution([sim_pkg_path, "mjcf", "scene.xml"])
    urdf_file_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "gr100_comp.urdf"])
    sim_node = Node(
        package='hex_ros_sim_archer_y6',
        executable='sim_archer_y6',
        name='sim_archer_y6',
        output="screen",
        emulate_tty=True,
        parameters=[
            sim_param_path,
            {
                "model_urdf": ParameterValue(urdf_file_path, value_type=str),
                "model_mjcf": ParameterValue(sim_mjcf_path, value_type=str),
                "prog_viewer": ParameterValue(LaunchConfiguration('viewer'), value_type=bool),
                "prog_rviz": ParameterValue(LaunchConfiguration('rviz'), value_type=bool),
            },
        ],
        remappings=[
            ('manip_state', 'manip_state'),
            ('manip_ctrl', 'manip_ctrl'),
            ('joint_states', 'joint_states'),
        ],
    )

    # rviz group
    rviz_config_path = PathJoinSubstitution(
        [sim_pkg_path, "config", "ros2", "display.rviz"])
    visual_urdf_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "gr100_full.urdf"])
    description_content = ParameterValue(Command(['xacro ', visual_urdf_path]),
                                         value_type=str)
    rviz_group = GroupAction(
        [
            Node(package='robot_state_publisher',
                 executable='robot_state_publisher',
                 parameters=[{
                     'robot_description': description_content,
                     'use_sim_time': True,
                 }]),
            Node(
                name="rviz2",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_path],
                parameters=[{
                    'use_sim_time': True,
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # test group
    test_group = GroupAction(
        [
            Node(
                package='hex_ros_sim_archer_y6',
                executable='test_ctrl',
                name='test_ctrl',
                output="screen",
                emulate_tty=True,
                parameters=[{
                    'use_sim_time': True,
                }],
                remappings=[
                    ('manip_ctrl', 'manip_ctrl'),
                ],
            )
        ],
        condition=IfCondition(LaunchConfiguration('test')),
    )

    return LaunchDescription([
        viewer_arg,
        rviz_arg,
        test_arg,
        sim_node,
        rviz_group,
        test_group,
    ])
