#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    bag_output_path = LaunchConfiguration('bag_output_path')
    bag_output_path_arg = DeclareLaunchArgument(
        'bag_output_path',
        default_value='/home/nvidia/Desktop/data/fast_livo2_bag',
        description='Output directory for ROS2 bag files'
    )

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='False',
        description='Whether to record ROS2 bag files'
    )

    # Find path
    bringup_package = get_package_share_directory('handheld_bringup')
    config_file_dir = os.path.join(bringup_package, "config", "fast-livo2")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")
    livox_config = os.path.join(bringup_package, 'config', 'MID360_config.json')
    hik_camera_config = os.path.join(bringup_package, 'config', 'hik_camera_fast_livo2.yaml')
    mcap_writer_options = os.path.join(bringup_package, 'config', 'mcap_writer_options.yaml')

    #Load parameters
    livox_config_cmd = os.path.join(config_file_dir, "mid360.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_pinhole_mid360.yaml")

    # Param use_rviz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to launch Rviz2",
    )

    livox_config_arg = DeclareLaunchArgument(
        'livox_params_file',
        default_value=livox_config_cmd,
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file to use for vikit_ros nodes',
    )

    # https://github.com/ros-navigation/navigation2/blob/1c68c212db01f9f75fcb8263a0fbb5dfa711bdea/nav2_bringup/launch/navigation_launch.py#L40
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    livox_params_file = LaunchConfiguration('livox_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_respawn = LaunchConfiguration('use_respawn')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 1},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/nvidia/.ros/livox_test.lvx'},
            {'user_config_path': livox_config},
            {'cmdline_input_bd_code': '47MDN6A0030249'}
        ]
    )
    
    mvs_driver = Node(
        package='mvs_ros_driver2',
        executable='mvs_camera_node',
        name='mvs_camera_trigger',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        arguments=[hik_camera_config],
        parameters=[{
            'use_sim_time': False,
        }],
        remappings=[
            # ('/hik_camera/image', '/left_camera/image'),
        ],
    )

    fast_livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[
            livox_params_file,
            {"camera_config": camera_params_file},
        ],
        # https://docs.ros.org/en/humble/How-To-Guides/Getting-Backtraces-in-ROS-2.html
        prefix=[
            # ("gdb -ex run --args"),
            # ("valgrind --log-file=./valgrind_report.log --tool=memcheck --leak-check=full --show-leak-kinds=all -s --track-origins=yes --show-reachable=yes --undef-value-errors=yes --track-fds=yes")
        ],
        remappings=[
            # ('/rgb/image', '/left_camera/image'),
        ],
        output="screen"
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    jetson_stats_node = Node(
        package='ros2_jetson_stats',
        executable='ros2_jtop',
        name='ros2_jtop',
        condition=IfCondition(LaunchConfiguration('record_bag')),
        parameters=[
            {'interval': 0.1} # 0.1 ~ 10Hz max
        ],
        output='screen',
    )

    sensor_warmup = TimerAction(
        period=1.0,  # delay in seconds for init
        actions=[livox_driver, mvs_driver]
    )

        # ROS2 bag recording
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            '--storage-config-file', mcap_writer_options,
            '-o', bag_output_path,
            'diagnostics',
            'livox/lidar',
            'livox/imu',
            'hik_camera/image',
        ],
        output='screen',
    )

    record_delay = TimerAction(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        period=5.0,  # delay in seconds for init
        actions=[rosbag_record]
    )

    return LaunchDescription([
        use_rviz_arg,
        livox_config_arg,
        camera_config_arg,
        use_respawn_arg,
        bag_output_path_arg,
        record_bag_arg,

        fast_livo_node,
        jetson_stats_node,
        rviz_node,
        sensor_warmup,
        record_delay,
    ])
