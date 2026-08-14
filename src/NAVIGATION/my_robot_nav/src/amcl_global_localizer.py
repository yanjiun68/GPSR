#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Empty


class AmclGlobalLocalizer(Node):
    def __init__(self):
        super().__init__('amcl_global_localizer')

        self.declare_parameter('service_name', '/reinitialize_global_localization')
        self.declare_parameter('localization_topic', '/amcl_pose')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('enabled', True)
        self.declare_parameter('startup_delay_sec', 2.0)
        self.declare_parameter('retry_interval_sec', 10.0)
        self.declare_parameter('max_retry_count', 3)
        self.declare_parameter('covariance_x_threshold', 0.12)
        self.declare_parameter('covariance_y_threshold', 0.12)
        self.declare_parameter('covariance_yaw_threshold', 0.10)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.startup_delay_sec = float(self.get_parameter('startup_delay_sec').value)
        self.retry_interval_sec = float(self.get_parameter('retry_interval_sec').value)
        self.max_retry_count = int(self.get_parameter('max_retry_count').value)
        self.covariance_x_threshold = float(
            self.get_parameter('covariance_x_threshold').value
        )
        self.covariance_y_threshold = float(
            self.get_parameter('covariance_y_threshold').value
        )
        self.covariance_yaw_threshold = float(
            self.get_parameter('covariance_yaw_threshold').value
        )

        self.have_manual_initial_pose = False
        self.localization_converged = False
        self.call_in_flight = False
        self.retry_count = 0
        self.start_time_sec = self._now_sec()
        self.last_request_time_sec: Optional[float] = None

        self.client = self.create_client(
            Empty,
            self.get_parameter('service_name').value,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('localization_topic').value,
            self.localization_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('initial_pose_topic').value,
            self.initial_pose_callback,
            10,
        )
        self.timer = self.create_timer(0.5, self.timer_callback)

        if self.enabled:
            self.get_logger().info(
                'AMCL global localizer enabled. '
                'If startup pose is unknown, it will request AMCL global '
                'localization and wait for covariance to settle.'
            )
        else:
            self.get_logger().info('AMCL global localizer disabled.')

    def initial_pose_callback(self, _msg: PoseWithCovarianceStamped):
        self.have_manual_initial_pose = True

    def localization_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        x_ok = cov[0] <= self.covariance_x_threshold
        y_ok = cov[7] <= self.covariance_y_threshold
        yaw_ok = cov[35] <= self.covariance_yaw_threshold
        converged = x_ok and y_ok and yaw_ok

        if converged and not self.localization_converged:
            self.get_logger().info(
                'AMCL localization looks converged: '
                f'cov_x={cov[0]:.3f}, cov_y={cov[7]:.3f}, cov_yaw={cov[35]:.3f}'
            )
        self.localization_converged = converged

    def timer_callback(self):
        if not self.enabled or self.localization_converged:
            return

        if self.have_manual_initial_pose:
            return

        if self.call_in_flight or self.retry_count >= self.max_retry_count:
            return

        now_sec = self._now_sec()
        if self.last_request_time_sec is None:
            if (now_sec - self.start_time_sec) < self.startup_delay_sec:
                return
        elif (now_sec - self.last_request_time_sec) < self.retry_interval_sec:
            return

        if not self.client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(
                'Waiting for /reinitialize_global_localization service...',
                throttle_duration_sec=5.0,
            )
            return

        self.retry_count += 1
        self.last_request_time_sec = now_sec
        self.call_in_flight = True

        request = Empty.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)
        self.get_logger().warn(
            'Requested AMCL global localization. '
            'Slowly rotate the robot in place so /amcl_pose can converge.'
        )

    def handle_response(self, future):
        self.call_in_flight = False
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(
                f'AMCL global localization request failed: {exc}'
            )
            return

        self.get_logger().info(
            'AMCL global localization request accepted.'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = AmclGlobalLocalizer()
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
