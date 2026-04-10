#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Launch file for Global-LVBA offline refinement.
Runs after FAST-LIVO2 has produced PCD + image + pose output.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    bringup_package = get_package_share_directory('handheld_bringup')
    default_config = os.path.join(bringup_package, 'config', 'global-lvba', 'config.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Full path to the Global-LVBA config YAML file',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to launch RViz2 for visualization',
    )

    rviz_config_file = os.path.join(bringup_package, 'config', 'rviz', 'fast_livo2.rviz')

    lvba_node = Node(
        package='global_lvba',
        executable='lidar_visual_ba',
        name='lv_ba',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        config_arg,
        use_rviz_arg,
        lvba_node,
        rviz_node,
    ])
