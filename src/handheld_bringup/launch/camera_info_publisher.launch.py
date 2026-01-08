from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch camera_info publisher for left camera."""

    # Declare launch arguments
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('handheld_bringup'),
            'config',
            'fast-livo2',
            'camera_pinhole_mid360.yaml'
        ]),
        description='Path to camera calibration YAML file'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='left_camera_optical_frame',
        description='Frame ID for camera_info messages'
    )

    # Camera info publisher node
    camera_info_node = Node(
        package='handheld_bringup',
        executable='camera_info_publisher.py',
        name='camera_info_publisher',
        output='screen',
        parameters=[{
            'calibration_file': LaunchConfiguration('calibration_file'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )

    return LaunchDescription([
        calibration_file_arg,
        frame_id_arg,
        camera_info_node,
    ])
