#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Combined launch file for replaying bag data with FAST-LIVO2 and SC-PGO
This launch file is for offline processing of recorded bag files.
No sensor drivers are included.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    bringup_package = get_package_share_directory('handheld_bringup')
    sc_pgo_package = get_package_share_directory('sc_pgo')

    # Configuration file paths
    config_file_dir = os.path.join(bringup_package, "config", "fast-livo2")
    sc_pgo_config = os.path.join(sc_pgo_package, 'config', 'params.yaml')

    # FAST-LIVO2 parameters
    livox_config_cmd = os.path.join(config_file_dir, "mid360.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_pinhole_mid360.yaml")

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to launch Rviz2 for visualization",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from bag file'
    )

    livox_config_arg = DeclareLaunchArgument(
        'livox_params_file',
        default_value=livox_config_cmd,
        description='Full path to the ROS2 parameters file for FAST-LIVO2',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file for camera',
    )

    sc_pgo_config_arg = DeclareLaunchArgument(
        'sc_pgo_config_file',
        default_value=sc_pgo_config,
        description='Full path to SC-PGO config file'
    )

    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Whether to launch Foxglove bridge'
    )

    # Get launch configurations
    livox_params_file = LaunchConfiguration('livox_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # FAST-LIVO2 node
    fast_livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[
            livox_params_file,
            {"camera_config": camera_params_file},
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            # Add remappings if needed for your bag file topics
        ],
        arguments=['--ros-args', '--log-level', 'ERROR'],  # Only show warnings/errors
        output="log"  # Suppress console output, only log to file
    )

    # SC-PGO node
    sc_pgo_node = Node(
        package='sc_pgo',
        executable='sc_pgo_node',
        name='sc_pgo_node',
        output='screen',
        parameters=[
            LaunchConfiguration('sc_pgo_config_file'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # SC-PGO subscribes to FAST-LIVO2 outputs
            ('sc_pgo_cloud_registered', 'cloud_registered'),
            ('sc_pgo_odometry', 'aft_mapped_to_init'),
        ]
    )

    # Camera info publisher node
    camera_info_node = Node(
        package='handheld_bringup',
        executable='camera_info_publisher.py',
        name='camera_info_publisher',
        output='screen',
        parameters=[{
            'calibration_file': camera_params_file,
            'frame_id': 'left_camera_optical_frame',
            'use_sim_time': use_sim_time
        }]
    )

    # Foxglove bridge node
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[
            {'port': 8765},
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(LaunchConfiguration('use_foxglove')),
        output='screen'
    )

    # Static TF: camera_init -> livox_frame
    # This is the base lidar frame used by FAST-LIVO2
    static_tf_camera_init_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_livox_frame',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'livox_frame'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static TF: livox_frame -> left_camera_optical_frame
    # From mid360.yaml extrinsic calibration: Pcl and Rcl (Camera to Lidar)
    # Inverted to get Lidar to Camera transformation
    static_tf_livox_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_frame_to_left_camera',
        arguments=[
            '0.030592', '0.009768', '-0.124693',  # x, y, z (translation)
            '0.396385', '-0.398932', '0.586410', '-0.582971',  # qx, qy, qz, qw (rotation)
            'livox_frame', 'left_camera_optical_frame'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        use_sim_time_arg,
        livox_config_arg,
        camera_config_arg,
        sc_pgo_config_arg,
        use_foxglove_arg,

        # Static TF transforms
        static_tf_camera_init_to_livox,
        static_tf_livox_to_camera,

        # Nodes
        fast_livo_node,
        sc_pgo_node,
        camera_info_node,
        foxglove_bridge_node,
    ])
