#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('mvs_ros_driver2')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='left_camera_trigger.yaml',
        description='Configuration file name in config directory'
    )
    
    # use_rviz_arg = DeclareLaunchArgument(
    #     'use_rviz',
    #     default_value='true',
    #     description='Launch RViz visualization'
    # )
    
    # Build path to config file
    config_path = PathJoinSubstitution([
        pkg_share,
        'config',
        LaunchConfiguration('config_file')
    ])
    
    # Build path to RViz config file
    # rviz_config_path = PathJoinSubstitution([
    #     pkg_share,
    #     'rviz_cfg',
    #     'mvs_camera.rviz'
    # ])
    
    # Camera node
    mvs_camera_node = Node(
        package='mvs_ros_driver2',
        executable='mvs_camera_node',
        name='mvs_camera_trigger',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        arguments=[config_path],
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # RViz node
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path],
    #     condition=IfCondition(LaunchConfiguration('use_rviz'))
    # )
    
    return LaunchDescription([
        config_file_arg,
        # use_rviz_arg,
        mvs_camera_node,
        # rviz_node,
    ])