#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sc_pgo')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'params.yaml'),
        description='Path to the SC-PGO config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # SC-PGO node
    sc_pgo_node = Node(
        package='sc_pgo',
        executable='sc_pgo_node',
        name='sc_pgo_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Remap topics to match FAST-LIVO2 outputs
            ('/cloud_registered', '/cloud_registered'),
            ('/aft_mapped_to_init', '/aft_mapped_to_init'),
            # Rename PGO output
            ('/aft_pgo_odom', '/pgo_odom'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        sc_pgo_node
    ])
