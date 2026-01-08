#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
import yaml
import os


class CameraInfoPublisher(Node):
    """
    Publishes CameraInfo messages based on calibration parameters.
    """

    def __init__(self):
        super().__init__('camera_info_publisher')

        # Declare parameters
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('frame_id', 'left_camera_optical_frame')

        # Get parameters
        calib_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Load calibration from file
        self.camera_info_msg = self.load_calibration(calib_file)

        # Create publisher for camera info
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/left_camera/image/camera_info',
            10
        )

        # Subscribe to the image topic to sync timing
        self.image_sub = self.create_subscription(
            Image,
            '/left_camera/image',
            self.image_callback,
            10
        )

        self.get_logger().info(f'Publishing camera_info on: /left_camera/image/camera_info')
        self.get_logger().info(f'Subscribing to images on: /left_camera/image')

    def load_calibration(self, calib_file):
        """Load camera calibration from YAML file and create CameraInfo message."""

        camera_info = CameraInfo()

        if not calib_file or not os.path.exists(calib_file):
            self.get_logger().error(f'Calibration file not found: {calib_file}')
            raise FileNotFoundError(f'Calibration file not found: {calib_file}')

        self.get_logger().info(f'Loading calibration from: {calib_file}')

        with open(calib_file, 'r') as f:
            calib_data = yaml.safe_load(f)

        # Extract calibration parameters from YAML
        width = calib_data['cam_width']
        height = calib_data['cam_height']
        fx = calib_data['cam_fx']
        fy = calib_data['cam_fy']
        cx = calib_data['cam_cx']
        cy = calib_data['cam_cy']

        # Distortion coefficients (k1, k2, k3, k4)
        k1 = calib_data['k1']
        k2 = calib_data['k2']
        k3 = calib_data['k3']
        k4 = calib_data['k4']

        # Fill CameraInfo message
        camera_info.width = width
        camera_info.height = height
        camera_info.distortion_model = 'plumb_bob'  # Standard radial-tangential model

        # Distortion coefficients: [k1, k2, t1, t2, k3]
        # Your calibration has k1, k2, k3, k4 - assuming k3, k4 are tangential distortion (t1, t2)
        camera_info.d = [k1, k2, k3, k4, 0.0]

        # Intrinsic camera matrix K: [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        camera_info.k = [
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0
        ]

        # Rectification matrix (identity for monocular camera)
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix P: [fx, 0, cx, 0; 0, fy, cy, 0; 0, 0, 1, 0]
        camera_info.p = [
            fx,  0.0, cx,  0.0,
            0.0, fy,  cy,  0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.get_logger().info(f'Loaded calibration: {width}x{height}, fx={fx:.2f}, fy={fy:.2f}')

        return camera_info

    def image_callback(self, msg):
        """Callback for image messages - publish corresponding camera_info."""

        # Update header to match the image timestamp and frame
        self.camera_info_msg.header = Header()
        self.camera_info_msg.header.stamp = msg.header.stamp
        self.camera_info_msg.header.frame_id = self.frame_id

        # Publish camera info
        self.camera_info_pub.publish(self.camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
