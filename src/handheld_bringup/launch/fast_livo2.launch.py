#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    # Find path
    bringup_package = get_package_share_directory('handheld_bringup')
    config_file_dir = os.path.join(bringup_package, "config", "fast-livo2")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")
    livox_config = os.path.join(bringup_package, 'config', 'MID360_config.json')
    hik_camera_config = os.path.join(bringup_package, 'config', 'hik_camera_fast_livo2.yaml')

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

    sensor_warmup = TimerAction(
        period=2.0,  # delay in seconds for init
        actions=[livox_driver, mvs_driver]
    )

    return LaunchDescription([
        use_rviz_arg,
        livox_config_arg,
        camera_config_arg,
        use_respawn_arg,

        # play ros2 bag
        # ExecuteProcess(
        #     cmd=[['ros2 bag play ', '~/datasets/Retail_Street ', '--clock ', "-l"]], 
        #     shell=True
        # ),

        # republish compressed image to raw image
        # https://robotics.stackexchange.com/questions/110939/how-do-i-remap-compressed-video-to-raw-video-in-ros2
        # ros2 run image_transport republish compressed raw --ros-args --remap in:=/left_camera/image --remap out:=/left_camera/image
        # Node(
        #     package="image_transport",
        #     executable="republish",
        #     name="republish",
        #     arguments=[ # Array of strings/parametric arguments that will end up in process's argv
        #         'compressed', 
        #         'raw',
        #     ],
        #     remappings=[
        #         ("in",  "/left_camera/image"), 
        #         ("out", "/left_camera/image")
        #     ],
        #     output="screen",
        #     respawn=use_respawn,
        # ),
        
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                livox_params_file,
                camera_params_file,
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
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),

        sensor_warmup,
    ])
