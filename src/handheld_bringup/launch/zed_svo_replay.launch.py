#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():
    # Launch arguments
    svo_file_path_arg = DeclareLaunchArgument(
        'svo_file_path',
        default_value='/home/nvidia/Desktop/data/recording.svo2',
        description='Path to the .svo2 file to replay'
    )

    bag_output_path_arg = DeclareLaunchArgument(
        'bag_output_path',
        default_value='/home/nvidia/Desktop/data/zed_replay_bag',
        description='Output directory for ROS2 bag files'
    )

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='True',
        description='Whether to record ROS2 bag files'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz2'
    )

    # Configuration paths
    bringup_package = get_package_share_directory('handheld_bringup')
    zed_wrapper_package = get_package_share_directory('zed_wrapper')
    zed_config_common = os.path.join(bringup_package, 'config', 'zed_config', 'common_stereo.yaml')
    zed_config_camera = os.path.join(bringup_package, 'config', 'zed_config', 'zed2i.yaml')
    mcap_writer_options = os.path.join(bringup_package, 'config', 'mcap_writer_options_compression.yaml')
    rviz_config_path = os.path.join(bringup_package, 'config', 'rviz', 'zed_view_svo.rviz')
    xacro_path = os.path.join(zed_wrapper_package, 'urdf', 'zed_descr.urdf.xacro')

    # Launch configurations
    svo_file_path = LaunchConfiguration('svo_file_path')
    bag_output_path = LaunchConfiguration('bag_output_path')

    # Zed camera component configured for SVO replay
    zed_component = ComposableNode(
        package='zed_components',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=[
            zed_config_common,
            zed_config_camera,
            {
                'general.camera_name': 'zed',
                'general.camera_model': 'zed2i',
                'svo.svo_path': svo_file_path,
                'sensors.sensors_pub_rate': 200.,
                # 'depth.depth_mode': 'NEURAL_PLUS',  # PERFORMANCE, NEURAL, NEURAL_PLUS, ULTRA
                # 'pos_tracking.pos_tracking_enabled': True,
                # 'pos_tracking.pos_tracking_mode': 'GEN_3', # GEN_1, GEN_2, GEN_3
                # 'pos_tracking.publish_tf': True,
                # 'pos_tracking.publish_map_tf': True,
                # 'pos_tracking.publish_cam_path': True,
                # 'pos_tracking.reset_odom_with_loop_closure': True,
                # 'mapping.mapping_enabled': False,
            }
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

    # ROS2 bag recording
    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=[
            'ros2', 'bag', 'record',
            '--use-sim-time',
            '--storage', 'mcap',
            '--storage-config-file', mcap_writer_options,
            '-o', bag_output_path,
            'zed_node/left/color/rect/image',
            'zed_node/right/color/rect/image',
            'zed_node/left/color/rect/camera_info',
            'zed_node/right/color/rect/camera_info',
            'zed_node/imu/data',
        ],
        output='screen',
    )

    # rviz_node = Node(
    #     condition=IfCondition(LaunchConfiguration('use_rviz')),
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen',
    #     arguments=['--display-config', rviz_config_path],
    # )

    # Robot State Publisher - publishes the ZED URDF and static transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro', ' ', xacro_path, ' ',
                'camera_name:=zed', ' ',
                'camera_model:=zed2i'
            ])
        }]
    )

    return LaunchDescription([
        # Launch arguments
        svo_file_path_arg,
        bag_output_path_arg,
        record_bag_arg,
        use_rviz_arg,

        # Nodes
        robot_state_publisher,
        zed_module_container,
        # rviz_node,
        rosbag_record,
    ])