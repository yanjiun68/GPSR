#!/usr/bin/env python3

import math
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener


def _yaw_from_pose(msg: PoseWithCovarianceStamped) -> float:
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class BootstrapPosePromoter(Node):
    def __init__(self):
        super().__init__('bootstrap_pose_promoter')

        self.declare_parameter('seed_topic', '/initialpose_seed')
        self.declare_parameter('refined_pose_topic', '/lidar_loc_pose')
        self.declare_parameter('output_topic', '/initialpose')
        self.declare_parameter('bootstrap_duration_sec', 2.0)
        self.declare_parameter('output_timestamp_offset_sec', 0.25)
        self.declare_parameter('publish_count', 2)
        self.declare_parameter('publish_interval_sec', 0.3)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_correction_distance', 0.35)
        self.declare_parameter('max_correction_yaw', 0.35)
        self.declare_parameter('tf_stamp_margin_sec', 0.10)

        self.seed_topic = self.get_parameter('seed_topic').value
        self.refined_pose_topic = self.get_parameter('refined_pose_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.bootstrap_duration_sec = float(
            self.get_parameter('bootstrap_duration_sec').value
        )
        self.output_timestamp_offset = Duration(
            nanoseconds=int(
                max(
                    0.0,
                    float(self.get_parameter('output_timestamp_offset_sec').value),
                ) * 1e9
            )
        )
        self.publish_count = max(1, int(self.get_parameter('publish_count').value))
        self.publish_interval_sec = max(
            0.05,
            float(self.get_parameter('publish_interval_sec').value),
        )
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_correction_distance = max(
            0.0,
            float(self.get_parameter('max_correction_distance').value),
        )
        self.max_correction_yaw = max(
            0.0,
            float(self.get_parameter('max_correction_yaw').value),
        )
        self.tf_stamp_margin = Duration(
            nanoseconds=int(
                max(
                    0.0,
                    float(self.get_parameter('tf_stamp_margin_sec').value),
                ) * 1e9
            )
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.output_topic,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.seed_topic,
            self.seed_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.refined_pose_topic,
            self.refined_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.session_start_sec = None
        self.seed_pose = None
        self.latest_refined_pose = None
        self.latest_refined_time_sec = None
        self.pending_publish_pose = None
        self.remaining_publish_count = 0
        self.next_publish_sec = None

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f'Bootstrapping AMCL initial pose from {self.seed_topic} via '
            f'{self.refined_pose_topic} for {self.bootstrap_duration_sec:.1f} s '
            f'before publishing to {self.output_topic}.'
        )

    def seed_callback(self, msg: PoseWithCovarianceStamped):
        self.seed_pose = deepcopy(msg)
        self.latest_refined_pose = None
        self.latest_refined_time_sec = None
        self.pending_publish_pose = None
        self.remaining_publish_count = 0
        self.next_publish_sec = None
        self.session_start_sec = self._now_sec()
        self.get_logger().info(
            'Received bootstrap seed pose from RViz. '
            'Running lidar alignment before seeding AMCL.'
        )

    def refined_callback(self, msg: PoseWithCovarianceStamped):
        if self.session_start_sec is None:
            return
        self.latest_refined_pose = deepcopy(msg)
        self.latest_refined_time_sec = self._now_sec()

    def timer_callback(self):
        now_sec = self._now_sec()

        if self.session_start_sec is not None:
            if (now_sec - self.session_start_sec) >= self.bootstrap_duration_sec:
                chosen_pose = self._choose_pose()
                if chosen_pose is not None:
                    self.pending_publish_pose = chosen_pose
                    self.remaining_publish_count = self.publish_count
                    self.next_publish_sec = now_sec
                self.session_start_sec = None

        if self.pending_publish_pose is None or self.remaining_publish_count <= 0:
            return

        if now_sec < self.next_publish_sec:
            return

        outgoing = deepcopy(self.pending_publish_pose)
        outgoing.header.stamp = self._resolve_output_stamp()
        self.publisher.publish(outgoing)
        self.remaining_publish_count -= 1
        self.next_publish_sec = now_sec + self.publish_interval_sec

        if self.remaining_publish_count == 0:
            self.get_logger().info(
                f'Published refined AMCL initial pose to {self.output_topic}.'
            )
            self.pending_publish_pose = None

    def _choose_pose(self):
        if (
            self.seed_pose is not None
            and self.latest_refined_pose is not None
            and self.latest_refined_time_sec is not None
            and self.latest_refined_time_sec >= self.session_start_sec + 0.2
        ):
            dx = (
                self.latest_refined_pose.pose.pose.position.x
                - self.seed_pose.pose.pose.position.x
            )
            dy = (
                self.latest_refined_pose.pose.pose.position.y
                - self.seed_pose.pose.pose.position.y
            )
            correction_distance = math.hypot(dx, dy)
            yaw_delta = abs(
                _wrap_angle(
                    _yaw_from_pose(self.latest_refined_pose)
                    - _yaw_from_pose(self.seed_pose)
                )
            )

            if (
                correction_distance > self.max_correction_distance
                or yaw_delta > self.max_correction_yaw
            ):
                self.get_logger().warn(
                    'Discarding lidar-refined bootstrap pose because the '
                    f'correction is too large: '
                    f'dxy={correction_distance:.3f} m, dyaw={yaw_delta:.3f} rad.'
                )
                return self.seed_pose

            self.get_logger().info(
                'Using lidar-refined bootstrap pose as the AMCL initial pose.'
            )
            return self.latest_refined_pose

        if self.seed_pose is not None:
            self.get_logger().warn(
                'No lidar-refined pose arrived in time. '
                'Falling back to the manual seed pose.'
            )
            return self.seed_pose

        return None

    def _resolve_output_stamp(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
            )
            stamp_ns = (
                transform.header.stamp.sec * 1_000_000_000
                + transform.header.stamp.nanosec
            )
            if stamp_ns > self.tf_stamp_margin.nanoseconds:
                return (Time.from_msg(transform.header.stamp) - self.tf_stamp_margin).to_msg()
            return transform.header.stamp
        except (LookupException, ConnectivityException, ExtrapolationException):
            return (self.get_clock().now() - self.output_timestamp_offset).to_msg()

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = BootstrapPosePromoter()
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
