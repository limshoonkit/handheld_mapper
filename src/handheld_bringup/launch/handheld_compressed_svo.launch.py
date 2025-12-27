#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():
    use_image_view_arg = DeclareLaunchArgument(
        'use_image_view',
        default_value='true',
        description='Launch rqt_image_view'
    )

    # Configuration paths
    bringup_package = get_package_share_directory('handheld_bringup')
    zed_config_common = os.path.join(bringup_package, 'config', 'zed_config', 'common_stereo.yaml')
    zed_config_camera = os.path.join(bringup_package, 'config', 'zed_config', 'zed2i.yaml')
    livox_config = os.path.join(bringup_package, 'config', 'MID360_config.json')
    hik_camera_config = os.path.join(bringup_package, 'config', 'hik_camera.yaml')

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

    # NOTE: ROS2 bag recording is now triggered externally by the shell script
    # to ensure it starts at exactly the same time as SVO recording.
    # See handheld_svo_record.sh for the rosbag start command.

    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        output='screen',
        arguments=['/left_camera/image']
    )

    return LaunchDescription([
        # Launch arguments
        use_image_view_arg,

        # Nodes
        zed_module_container,
        livox_driver,
        mvs_driver,
        rqt_image_view_node,
    ])