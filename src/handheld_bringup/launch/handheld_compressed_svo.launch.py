#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():    
    bag_output_path = LaunchConfiguration('bag_output_path')
    bag_output_path_arg = DeclareLaunchArgument(
        'bag_output_path',
        default_value='/home/nvidia/Desktop/data/calib_bag',
        description='Output directory for ROS2 bag files'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )

    # Configuration paths
    bringup_package = get_package_share_directory('handheld_bringup')
    zed_config_common = os.path.join(bringup_package, 'config', 'zed_config', 'common_stereo.yaml')
    zed_config_camera = os.path.join(bringup_package, 'config', 'zed_config', 'zed2i.yaml')
    mcap_writer_options = os.path.join(bringup_package, 'config', 'mcap_writer_options_compression.yaml')
    livox_config = os.path.join(bringup_package, 'config', 'MID360_config.json')
    hik_camera_config = os.path.join(bringup_package, 'config', 'hik_camera.yaml')
    rviz_config_path = os.path.join(bringup_package, 'config', 'rviz', 'handheld_sensors.rviz')

    # Zed camera component
    zed_component = ComposableNode(
        package='zed_components',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=[
            zed_config_common,
            zed_config_camera,
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    zed_module_container = ComposableNodeContainer(
        name='zed_module_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            zed_component,
        ]
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0},
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
        }]
    )

    # ROS2 bag recording (ZED topics excluded - using SVO format instead)
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            '--storage-config-file', mcap_writer_options,
            '-o', bag_output_path,
            '--max-cache-size', '100000000',  # 100MB cache limit
            'livox/lidar',
            'livox/imu',
            'left_camera/image',
        ],
        output='screen',
    )

    record_delay = TimerAction(
        period=3.0,
        actions=[rosbag_record]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path]
    )

    return LaunchDescription([
        # Launch arguments
        bag_output_path_arg,
        use_rviz_arg,

        # Nodes
        zed_module_container,
        livox_driver,
        mvs_driver,
        rviz_node,
        record_delay,
    ])