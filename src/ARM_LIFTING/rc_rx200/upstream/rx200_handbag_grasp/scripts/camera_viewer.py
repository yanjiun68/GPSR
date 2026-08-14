#!/usr/bin/env python3
"""
Simple camera viewer for Step 2.

Displays the live compressed color image from the RealSense camera without any
detection or annotation pipeline.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraViewer(Node):
    """Display a live camera feed in an OpenCV window."""

    def __init__(self):
        super().__init__('camera_viewer')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('window_name', 'Camera View')

        image_topic = self.get_parameter('image_topic').value
        self.window_name = self.get_parameter('window_name').value

        self.subscription = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_callback,
            10,
        )

        self.get_logger().info(f'Camera viewer started, subscribing to: {image_topic}')

    def image_callback(self, msg: CompressedImage):
        """Decode and display the latest compressed frame."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                cv2.imshow(self.window_name, img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('Viewer closed by user')
                    rclpy.shutdown()
        except Exception as exc:
            self.get_logger().error(f'Error displaying image: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
