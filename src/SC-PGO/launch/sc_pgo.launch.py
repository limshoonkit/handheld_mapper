#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    sc_pgo_package = get_package_share_directory('sc_pgo')
    config_file = os.path.join(sc_pgo_package, 'config', 'sc_pgo_params.yaml')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )

    # SC-PGO node
    sc_pgo_node = Node(
        package='sc_pgo',
        executable='laser_posegraph_optimization',
        name='laser_pgo',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[

        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        sc_pgo_node,
    ])
