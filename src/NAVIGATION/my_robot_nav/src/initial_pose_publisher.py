#!/usr/bin/env python3

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/initialpose')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('covariance_x', 0.25)
        self.declare_parameter('covariance_y', 0.25)
        self.declare_parameter('covariance_yaw', 0.06853891909122467)
        self.declare_parameter('delay_sec', 8.0)
        self.declare_parameter('repeat_count', 5)
        self.declare_parameter('repeat_interval', 0.5)
        self.declare_parameter('tf_target_frame', 'odom')
        self.declare_parameter('tf_source_frame', 'base_footprint')

        self.frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.covariance_x = float(self.get_parameter('covariance_x').value)
        self.covariance_y = float(self.get_parameter('covariance_y').value)
        self.covariance_yaw = float(self.get_parameter('covariance_yaw').value)
        self.delay = Duration(seconds=float(self.get_parameter('delay_sec').value))
        self.repeat_count = int(self.get_parameter('repeat_count').value)
        self.repeat_interval = Duration(
            seconds=float(self.get_parameter('repeat_interval').value)
        )
        self.tf_target_frame = self.get_parameter('tf_target_frame').value
        self.tf_source_frame = self.get_parameter('tf_source_frame').value

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.start_time = self.get_clock().now()
        self.next_publish_time = self.start_time + self.delay
        self.sent_count = 0
        self.logged_waiting_for_subscriber = False
        self.logged_waiting_for_tf = False
        self.done = False

        self.timer = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        now = self.get_clock().now()
        if now < self.next_publish_time:
            return

        if self.publisher.get_subscription_count() == 0:
            if not self.logged_waiting_for_subscriber:
                self.get_logger().info('Waiting for /initialpose subscriber...')
                self.logged_waiting_for_subscriber = True
            return

        transform = self.lookup_latest_transform()
        if transform is None:
            if not self.logged_waiting_for_tf:
                self.get_logger().info(
                    f'Waiting for TF {self.tf_target_frame} -> {self.tf_source_frame}...'
                )
                self.logged_waiting_for_tf = True
            return

        self.publisher.publish(self.build_message(transform.header.stamp))
        self.sent_count += 1
        self.get_logger().info(
            f'Published initial pose {self.sent_count}/{self.repeat_count}: '
            f'x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}'
        )

        if self.sent_count >= self.repeat_count:
            self.done = True
            return

        self.next_publish_time = now + self.repeat_interval

    def lookup_latest_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.tf_target_frame,
                self.tf_source_frame,
                Time(),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    def build_message(self, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.pose.covariance = [
            self.covariance_x, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.covariance_y, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.covariance_yaw,
        ]
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
