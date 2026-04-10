#!/usr/bin/env python3
"""
Convert PoseStamped messages to TUM format for trajectory evaluation.
TUM format: timestamp x y z qx qy qz qw

Usage:
    # Subscribe to a single topic:
    ros2 run handheld_bringup pose_to_tum.py --topics /pose --output_dir ./trajectories
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import argparse
import sys
import os


def get_output_name_from_topic(topic):
    """Extract the first namespace from a topic to use as output filename."""
    # Remove leading slash and split by /
    parts = topic.strip('/').split('/')
    if parts:
        return parts[0]
    return 'trajectory'


class PoseToTUM(Node):
    def __init__(self, topics, output_dir):
        super().__init__('pose_to_tum')

        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        # Dictionary to store file handles and pose counts per topic
        self.file_handles = {}
        self.pose_counts = {}
        self._subs = []

        for topic in topics:
            output_name = get_output_name_from_topic(topic)
            output_file = os.path.join(output_dir, f'{output_name}_tum.txt')

            self.file_handles[topic] = open(output_file, 'w')
            self.pose_counts[topic] = 0

            # Create subscription with a closure to capture topic
            sub = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, t=topic: self.pose_callback(msg, t),
                10
            )
            self._subs.append(sub)

            self.get_logger().info(f'Listening to {topic} -> {output_file}')

        self.get_logger().info('TUM format: timestamp x y z qx qy qz qw')

    def pose_callback(self, msg, topic):
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
        self.file_handles[topic].write(line)
        self.file_handles[topic].flush()

        self.pose_counts[topic] += 1
        if self.pose_counts[topic] % 100 == 0:
            self.get_logger().info(f'[{topic}] Saved {self.pose_counts[topic]} poses')

    def cleanup(self):
        """Close all output files."""
        for topic, fh in self.file_handles.items():
            if fh and not fh.closed:
                fh.close()
                output_name = get_output_name_from_topic(topic)
                self.get_logger().info(f'[{topic}] Total poses: {self.pose_counts[topic]} -> {output_name}_tum.txt')


def main(args=None):
    parser = argparse.ArgumentParser(description='Convert PoseStamped topics to TUM format')
    parser.add_argument(
        '--topics',
        type=str,
        nargs='+',
        default=['/x_node/pose'],
        help='PoseStamped topics to subscribe to (default: /x_node/pose)'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        default='.',
        help='Output directory for TUM trajectory files (default: current directory)'
    )

    filtered_args = []
    for arg in sys.argv[1:]:
        if arg == '--ros-args':
            break  # Stop processing at --ros-args
        if arg.startswith('-r') or arg.startswith('__'):
            continue
        filtered_args.append(arg)

    parsed_args = parser.parse_args(args=filtered_args)

    output_dir = os.path.expanduser(parsed_args.output_dir)

    rclpy.init()

    node = PoseToTUM(parsed_args.topics, output_dir)

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
