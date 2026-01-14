#!/usr/bin/env python3
"""
Convert ZED camera pose messages to TUM format for trajectory evaluation.
TUM format: timestamp x y z qx qy qz qw

Usage:
    # Launch with SVO replay (recommended - includes this node):
    ros2 launch handheld_bringup zed_svo_replay.launch.py svo_file_path:=/path/to/file.svo2 tum_output:=/path/to/output.txt

    # Or run standalone while ZED is publishing:
    ros2 run handheld_bringup zed_pose_to_tum.py --output zed_trajectory.txt
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import argparse
import sys
import os


class ZedPoseToTUM(Node):
    def __init__(self, output_file, topic='/zed_node/pose'):
        super().__init__('zed_pose_to_tum')

        self.output_file = output_file
        self.file_handle = open(output_file, 'w')
        self.pose_count = 0

        # Subscribe to ZED pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(f'Listening to {topic}')
        self.get_logger().info(f'Saving TUM trajectory to: {output_file}')
        self.get_logger().info('TUM format: timestamp x y z qx qy qz qw')

    def pose_callback(self, msg):
        """Convert PoseStamped to TUM format and write to file."""
        # Extract timestamp (convert to seconds with nanoseconds precision)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Extract orientation (quaternion)
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Write to file in TUM format
        line = f"{timestamp:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
        self.file_handle.write(line)
        self.file_handle.flush()  # Ensure data is written immediately

        self.pose_count += 1
        if self.pose_count % 100 == 0:
            self.get_logger().info(f'Saved {self.pose_count} poses')

    def cleanup(self):
        """Close the output file."""
        if self.file_handle and not self.file_handle.closed:
            self.file_handle.close()
            self.get_logger().info(f'Total poses saved: {self.pose_count}')
            self.get_logger().info(f'Output file: {self.output_file}')


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Convert ZED pose to TUM format')
    parser.add_argument(
        '--output',
        type=str,
        default='zed_trajectory_tum.txt',
        help='Output TUM trajectory file path'
    )
    parser.add_argument(
        '--topic',
        type=str,
        default='/zed_node/pose',
        help='ZED pose topic (default: /zed_node/pose)'
    )

    # Filter out ROS-specific arguments before parsing
    filtered_args = []
    for arg in sys.argv[1:]:
        if arg == '--ros-args':
            break  # Stop processing at --ros-args
        if arg.startswith('-r') or arg.startswith('__'):
            continue
        filtered_args.append(arg)

    parsed_args = parser.parse_args(args=filtered_args)

    # Expand user path if provided
    output_path = os.path.expanduser(parsed_args.output)

    rclpy.init()

    node = ZedPoseToTUM(output_path, parsed_args.topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
