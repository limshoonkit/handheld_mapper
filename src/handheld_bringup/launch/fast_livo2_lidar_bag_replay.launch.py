#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    bringup_package = get_package_share_directory("handheld_bringup")
    config_file_dir = os.path.join(bringup_package, "config", "fast-livo2")
    rviz_config_file = os.path.join(bringup_package, "config", "rviz", "fast_livo2.rviz")

    livox_config_default = os.path.join(config_file_dir, "mid360_offline_lidar_only_180.yaml")
    camera_config_default = os.path.join(config_file_dir, "camera_pinhole_mid360.yaml")

    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value="/home/ubuntu/Desktop/uosm_cirg/handheld_mapper/data/lidar_bag_2",
        description="Path to ros2 bag directory for replay",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to launch Rviz2",
    )

    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value="True",
        description="Whether to launch Foxglove bridge",
    )

    play_rate_arg = DeclareLaunchArgument(
        "play_rate",
        default_value="1.0",
        description="Bag playback rate",
    )

    loop_bag_arg = DeclareLaunchArgument(
        "loop_bag",
        default_value="False",
        description="Replay bag in loop mode",
    )

    livox_config_arg = DeclareLaunchArgument(
        "livox_params_file",
        default_value=livox_config_default,
        description="FAST-LIVO2 lidar config file",
    )

    camera_config_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=camera_config_default,
        description="Camera config file (kept for FAST-LIVO2 compatibility)",
    )

    livox_params_file = LaunchConfiguration("livox_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")

    fast_livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[
            livox_params_file,
            {"camera_config": camera_params_file},
            {"use_sim_time": True},
        ],
        arguments=["--ros-args", "--log-level", "WARN"],
        output="screen",
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    foxglove_bridge_node = Node(
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            {"port": 8765},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag_path"),
            "--clock",
            "-r",
            LaunchConfiguration("play_rate"),
            "--loop",
        ],
        condition=IfCondition(LaunchConfiguration("loop_bag")),
        output="screen",
    )

    bag_play_once = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag_path"),
            "--clock",
            "-r",
            LaunchConfiguration("play_rate"),
        ],
        condition=UnlessCondition(LaunchConfiguration("loop_bag")),
        output="screen",
    )

    delayed_bag_play = TimerAction(
        period=2.0,
        actions=[bag_play, bag_play_once],
    )

    return LaunchDescription(
        [
            bag_path_arg,
            use_rviz_arg,
            use_foxglove_arg,
            play_rate_arg,
            loop_bag_arg,
            livox_config_arg,
            camera_config_arg,
            fast_livo_node,
            rviz_node,
            foxglove_bridge_node,
            delayed_bag_play,
        ]
    )
