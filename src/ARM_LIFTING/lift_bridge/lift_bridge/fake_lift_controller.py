#!/usr/bin/env python3
"""
Fake Lift Controller for simulation mode.
Provides a FollowJointTrajectory action server that simulates lift movement.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.qos import qos_profile_sensor_data

import time
import threading


class FakeLiftController(Node):
    """
    Simulates a lift controller with FollowJointTrajectory action server.
    Accepts trajectory goals and simulates movement by interpolating position.
    """

    def __init__(self):
        super().__init__('fake_lift_controller')
        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('joint_name', 'lift_joint')
        self.declare_parameter('joint_states_topic', '/lift/joint_state')
        self.declare_parameter('action_name', '/lift_controller/follow_joint_trajectory')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('min_m', 0.0)
        self.declare_parameter('max_m', 0.275)
        self.declare_parameter('max_velocity', 0.05)  # m/s

        self.joint_name = self.get_parameter('joint_name').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.action_name = self.get_parameter('action_name').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.min_m = self.get_parameter('min_m').value
        self.max_m = self.get_parameter('max_m').value
        self.max_velocity = self.get_parameter('max_velocity').value

        # State
        self.current_position = 0.0
        self.target_position = 0.0
        self.is_moving = False
        self.position_lock = threading.Lock()

        # Publisher
        self.pub = self.create_publisher(
            JointState, self.joint_states_topic, qos_profile_sensor_data
        )

        # Action Server
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group
        )

        # Timer for publishing joint states
        period = 1.0 / max(1.0, self.publish_rate)
        self.timer = self.create_timer(period, self._publish_state, callback_group=self.cb_group)

        self.get_logger().info(
            f'FakeLiftController started: joint={self.joint_name}, '
            f'action={self.action_name}, limits=[{self.min_m*1000:.0f}, {self.max_m*1000:.0f}]mm'
        )

    def goal_cb(self, goal_request) -> GoalResponse:
        """Accept all goals."""
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle) -> CancelResponse:
        """Accept cancellation."""
        self.is_moving = False
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle) -> FollowJointTrajectory.Result:
        """Execute the trajectory by simulating movement."""
        result = FollowJointTrajectory.Result()
        traj = goal_handle.request.trajectory

        # Find lift_joint index
        if self.joint_name not in traj.joint_names:
            self.get_logger().warn(f'Joint {self.joint_name} not in trajectory')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        idx = traj.joint_names.index(self.joint_name)

        if not traj.points:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        # Get final target position
        final_point = traj.points[-1]
        if idx >= len(final_point.positions):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

        target = max(self.min_m, min(self.max_m, final_point.positions[idx]))

        with self.position_lock:
            start_pos = self.current_position

        self.target_position = target
        self.is_moving = True

        # Calculate movement time
        distance = abs(target - start_pos)
        move_time = distance / self.max_velocity if self.max_velocity > 0 else 1.0
        move_time = max(0.5, move_time)  # Minimum 0.5s

        self.get_logger().info(
            f'Moving lift: {start_pos*1000:.1f}mm -> {target*1000:.1f}mm ({move_time:.1f}s)'
        )

        # Simulate movement
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = [self.joint_name]

        start_time = time.time()
        while rclpy.ok() and self.is_moving:
            if goal_handle.is_cancel_requested:
                self.is_moving = False
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            elapsed = time.time() - start_time
            progress = min(1.0, elapsed / move_time)

            # Linear interpolation
            with self.position_lock:
                self.current_position = start_pos + (target - start_pos) * progress

            # Publish feedback
            feedback.actual.positions = [self.current_position]
            feedback.desired.positions = [target]
            feedback.error.positions = [target - self.current_position]
            goal_handle.publish_feedback(feedback)

            if progress >= 1.0:
                break

            time.sleep(0.02)

        # Final position
        with self.position_lock:
            self.current_position = target

        self.is_moving = False
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL

        self.get_logger().info(f'Lift reached {target*1000:.1f}mm')
        return result

    def _publish_state(self):
        """Publish current joint state."""
        with self.position_lock:
            pos = self.current_position

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [pos]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLiftController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
