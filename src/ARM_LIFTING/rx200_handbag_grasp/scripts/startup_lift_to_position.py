#!/usr/bin/env python3
"""Move the lift to a startup height after hardware homing completes."""

import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint


class StartupLiftToPosition(Node):
    """One-shot node: wait for /lift/is_homed, then command the lift upward."""

    def __init__(self) -> None:
        super().__init__('startup_lift_to_position')

        self.declare_parameter('target_position', 0.10)
        self.declare_parameter('joint_name', 'lift_joint')
        self.declare_parameter('action_name', '/lift_controller/follow_joint_trajectory')
        self.declare_parameter('homed_topic', '/lift/is_homed')
        self.declare_parameter('wait_timeout_s', 45.0)
        self.declare_parameter('move_duration_s', 4.0)

        self.target_position = float(self.get_parameter('target_position').value)
        self.joint_name = str(self.get_parameter('joint_name').value)
        self.action_name = str(self.get_parameter('action_name').value)
        self.homed_topic = str(self.get_parameter('homed_topic').value)
        self.wait_timeout_s = float(self.get_parameter('wait_timeout_s').value)
        self.move_duration_s = float(self.get_parameter('move_duration_s').value)

        self._is_homed = False
        self.create_subscription(Bool, self.homed_topic, self._homed_cb, 10)
        self._client = ActionClient(self, FollowJointTrajectory, self.action_name)

    def _homed_cb(self, msg: Bool) -> None:
        self._is_homed = bool(msg.data)

    def run(self) -> bool:
        self.get_logger().info(
            f'Waiting for lift home, then moving {self.joint_name} to '
            f'{self.target_position:.3f}m'
        )

        start = time.monotonic()
        while rclpy.ok() and not self._is_homed:
            if (time.monotonic() - start) > self.wait_timeout_s:
                self.get_logger().error(
                    f'Timed out after {self.wait_timeout_s:.1f}s waiting for {self.homed_topic}'
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info('Lift homed. Waiting for lift action server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Action server unavailable: {self.action_name}')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.joint_name]

        point = JointTrajectoryPoint()
        point.positions = [self.target_position]
        point.time_from_start.sec = int(self.move_duration_s)
        point.time_from_start.nanosec = int(
            (self.move_duration_s - int(self.move_duration_s)) * 1_000_000_000
        )
        goal.trajectory.points = [point]

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Lift startup goal was rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error('Lift startup goal returned no result')
            return False

        error_code = result.result.error_code
        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Lift startup move failed with error_code={error_code}')
            return False

        self.get_logger().info(f'Lift startup position reached: {self.target_position:.3f}m')
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StartupLiftToPosition()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
