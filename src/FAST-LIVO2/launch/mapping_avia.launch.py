from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_fast_livo = get_package_share_directory('fast_livo')

    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('fast_livo'),
            'config',
            'camera_pinhole.yaml'
        ]),
        description='Camera configuration file path'
    )

    # Path to parameter file
    params_file = PathJoinSubstitution([
        FindPackageShare('fast_livo'),
        'config',
        'avia.yaml'
    ])

    # Fast-LIVO mapping node
    fastlivo_mapping_node = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='laserMapping',
        output='screen',
        parameters=[params_file],
        arguments=['--camera_config', LaunchConfiguration('camera_config')],
        additional_env={'MALLOC_TRIM_THRESHOLD_': '1073741824', 'MALLOC_ARENA_MAX': '4'}
    )

    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('fast_livo'),
        'rviz_cfg',
        'fast_livo2.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Image republish node (compressed to raw)
    image_republish_node = Node(
        package='image_transport',
        executable='republish',
        name='republish',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/left_camera/image/compressed'),
            ('out', '/left_camera/image')
        ],
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        camera_config_arg,
        fastlivo_mapping_node,
        rviz_node,
        image_republish_node
    ])
