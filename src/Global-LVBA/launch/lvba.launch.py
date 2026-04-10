#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('global_lvba')
    default_config = os.path.join(pkg_share, 'config', 'config.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Full path to the LVBA config YAML file',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to launch RViz2',
    )

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
        output='screen',
    )

    return LaunchDescription([
        config_arg,
        use_rviz_arg,
        lvba_node,
        rviz_node,
    ])
