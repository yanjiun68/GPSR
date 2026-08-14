#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node


class InitialPoseRelay(Node):
    def __init__(self):
        super().__init__('initial_pose_relay')

        self.declare_parameter('input_topic', '/initialpose_raw')
        self.declare_parameter('output_topic', '/initialpose')
        self.declare_parameter('timestamp_offset_sec', 0.12)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.timestamp_offset_sec = max(
            0.0,
            float(self.get_parameter('timestamp_offset_sec').value),
        )
        self.timestamp_offset = Duration(
            nanoseconds=int(self.timestamp_offset_sec * 1e9)
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.output_topic,
            10,
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.input_topic,
            self.pose_callback,
            10,
        )

        self.get_logger().info(
            f'Relaying {self.input_topic} -> {self.output_topic} '
            f'with timestamp_offset={self.timestamp_offset_sec:.3f} s'
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        forwarded = PoseWithCovarianceStamped()
        forwarded.header.frame_id = msg.header.frame_id
        forwarded.header.stamp = (
            self.get_clock().now() - self.timestamp_offset
        ).to_msg()
        forwarded.pose = msg.pose
        self.publisher.publish(forwarded)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
