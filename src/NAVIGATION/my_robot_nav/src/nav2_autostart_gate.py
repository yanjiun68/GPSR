#!/usr/bin/env python3

import rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class Nav2AutostartGate(Node):
    def __init__(self):
        super().__init__('nav2_autostart_gate')

        self.declare_parameter('service_name', '/lifecycle_manager_navigation/manage_nodes')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('require_initial_pose', True)
        self.declare_parameter('localization_topic', '/amcl_pose')
        self.declare_parameter('require_localization', False)
        self.declare_parameter('covariance_x_threshold', 0.12)
        self.declare_parameter('covariance_y_threshold', 0.12)
        self.declare_parameter('covariance_yaw_threshold', 0.10)
        self.declare_parameter('require_transform', False)
        self.declare_parameter('transform_target_frame', 'map')
        self.declare_parameter('transform_source_frame', 'base_footprint')
        self.declare_parameter('wait_log_interval_sec', 5.0)
        self.declare_parameter('startup_delay', 0.5)

        self.require_initial_pose = bool(self.get_parameter('require_initial_pose').value)
        self.require_localization = bool(self.get_parameter('require_localization').value)
        self.require_transform = bool(self.get_parameter('require_transform').value)
        self.transform_target_frame = self.get_parameter('transform_target_frame').value
        self.transform_source_frame = self.get_parameter('transform_source_frame').value
        self.wait_log_interval_sec = float(
            self.get_parameter('wait_log_interval_sec').value
        )
        self.covariance_x_threshold = float(
            self.get_parameter('covariance_x_threshold').value
        )
        self.covariance_y_threshold = float(
            self.get_parameter('covariance_y_threshold').value
        )
        self.covariance_yaw_threshold = float(
            self.get_parameter('covariance_yaw_threshold').value
        )
        self.startup_delay = float(self.get_parameter('startup_delay').value)

        self.have_odom = False
        self.have_initial_pose = False
        self.have_localization = False
        self.startup_requested = False
        self.ready_since = None
        self.last_wait_log_time = None

        self.tf_buffer = Buffer() if self.require_transform else None
        self.tf_listener = (
            TransformListener(self.tf_buffer, self)
            if self.require_transform
            else None
        )

        self.client = self.create_client(
            ManageLifecycleNodes,
            self.get_parameter('service_name').value,
        )

        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('initial_pose_topic').value,
            self.initial_pose_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('localization_topic').value,
            self.localization_callback,
            10,
        )
        self.timer = self.create_timer(0.2, self.timer_callback)

        wait_conditions = ['odom']
        if self.require_initial_pose:
            wait_conditions.append('initial pose')
        if self.require_localization:
            wait_conditions.append('converged localization')
        if self.require_transform:
            wait_conditions.append(
                f'{self.transform_target_frame} -> '
                f'{self.transform_source_frame} TF'
            )

        self.get_logger().info(
            'Waiting for odom'
            f'{" and initial pose" if self.require_initial_pose else ""}'
            f'{" and converged localization" if self.require_localization else ""}'
            ' before starting Nav2.'
        )

    def odom_callback(self, _msg: Odometry):
        self.have_odom = True

    def initial_pose_callback(self, _msg: PoseWithCovarianceStamped):
        self.have_initial_pose = True

    def localization_callback(self, msg: PoseWithCovarianceStamped):
        covariance = msg.pose.covariance
        self.have_localization = (
            covariance[0] <= self.covariance_x_threshold
            and covariance[7] <= self.covariance_y_threshold
            and covariance[35] <= self.covariance_yaw_threshold
        )

    def timer_callback(self):
        if self.startup_requested:
            return

        if not self.have_odom:
            self.ready_since = None
            return

        if self.require_initial_pose and not self.have_initial_pose:
            self.ready_since = None
            return

        if self.require_localization and not self.have_localization:
            self.ready_since = None
            return

        if self.require_transform and not self.required_transform_available():
            self.ready_since = None
            return

        if not self.client.wait_for_service(timeout_sec=0.0):
            self.ready_since = None
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.ready_since is None:
            self.ready_since = now_sec
            return

        if (now_sec - self.ready_since) < self.startup_delay:
            return

        request = ManageLifecycleNodes.Request()
        request.command = getattr(ManageLifecycleNodes.Request, 'STARTUP', 0)
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)
        self.startup_requested = True
        self.get_logger().info('Conditions satisfied. Requesting Nav2 startup.')

    def required_transform_available(self):
        if self.tf_buffer is None:
            return True

        available = self.tf_buffer.can_transform(
            self.transform_target_frame,
            self.transform_source_frame,
            Time(),
        )
        if not available:
            self.log_waiting(
                f'Waiting for TF {self.transform_target_frame} -> '
                f'{self.transform_source_frame} before starting Nav2.'
            )
        return available

    def log_waiting(self, message):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (
            self.last_wait_log_time is None
            or (now_sec - self.last_wait_log_time) >= self.wait_log_interval_sec
        ):
            self.last_wait_log_time = now_sec
            self.get_logger().info(message)

    def handle_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.startup_requested = False
            self.ready_since = None
            self.get_logger().error(f'Failed to call lifecycle manager: {exc}')
            return

        if response is None or not response.success:
            self.startup_requested = False
            self.ready_since = None
            self.get_logger().error('Lifecycle manager rejected Nav2 startup request.')
            return

        self.get_logger().info('Nav2 startup request accepted.')
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2AutostartGate()
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
