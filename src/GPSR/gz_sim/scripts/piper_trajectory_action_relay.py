#!/usr/bin/env python3
"""Expose MoveIt controller actions backed by Gazebo JointTrajectory topics."""

import asyncio
import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class TrajectoryEndpoint:
    def __init__(self, node, action_name, command_topic, joints):
        self.node = node
        self.action_name = action_name
        self.joints = tuple(joints)
        self.publisher = node.create_publisher(JointTrajectory, command_topic, 10)
        self.lock = threading.Lock()
        self.busy = False
        self.server = ActionServer(
            node,
            FollowJointTrajectory,
            action_name,
            execute_callback=self.execute,
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel,
            callback_group=node.callback_group,
        )

    def accept_goal(self, goal):
        if tuple(goal.trajectory.joint_names) != self.joints:
            self.node.get_logger().error(
                f'{self.action_name}: expected joints {self.joints}, '
                f'got {tuple(goal.trajectory.joint_names)}')
            return GoalResponse.REJECT
        if not goal.trajectory.points:
            return GoalResponse.REJECT
        previous_time = -1.0
        for point in goal.trajectory.points:
            point_time = duration_seconds(point.time_from_start)
            if (len(point.positions) != len(self.joints) or
                    not all(math.isfinite(value) for value in point.positions) or
                    point_time <= previous_time):
                return GoalResponse.REJECT
            previous_time = point_time
        with self.lock:
            return GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT

    @staticmethod
    def accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def hold_current_position(self):
        positions = self.node.positions_for(self.joints)
        if positions is None:
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.joints)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.nanosec = 100_000_000
        trajectory.points = [point]
        self.publisher.publish(trajectory)

    async def execute(self, goal_handle):
        with self.lock:
            if self.busy:
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'controller is busy'
                return result
            self.busy = True
        result = FollowJointTrajectory.Result()
        try:
            if not self.node.wait_for_positions(self.joints, 5.0):
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = 'Gazebo joint feedback is unavailable'
                return result

            self.publisher.publish(goal_handle.request.trajectory)
            final_point = goal_handle.request.trajectory.points[-1]
            command_duration = duration_seconds(final_point.time_from_start)
            completion_not_before = time.monotonic() + command_duration
            deadline = completion_not_before + 10.0
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(self.joints)
            feedback.desired = final_point

            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    self.hold_current_position()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'trajectory canceled and joints held'
                    return result
                actual = self.node.positions_for(self.joints)
                if actual is not None:
                    feedback.actual.positions = actual
                    feedback.error.positions = [
                        desired - measured
                        for desired, measured in zip(final_point.positions, actual)
                    ]
                    goal_handle.publish_feedback(feedback)
                    if (time.monotonic() >= completion_not_before and
                            max(abs(value) for value in feedback.error.positions) <= 0.03):
                        goal_handle.succeed()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        return result
                await asyncio.sleep(0.02)

            self.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = 'Gazebo joints did not reach the trajectory goal'
            return result
        finally:
            with self.lock:
                self.busy = False


class PiperTrajectoryActionRelay(Node):
    def __init__(self):
        super().__init__('piper_trajectory_action_relay')
        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()
        self.positions = {}
        self.last_state_time = 0.0
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 20,
            callback_group=self.callback_group)
        self.arm = TrajectoryEndpoint(
            self, '/arm_controller/follow_joint_trajectory',
            '/piper/arm_joint_trajectory',
            [f'joint{index}' for index in range(1, 7)])
        self.gripper = TrajectoryEndpoint(
            self, '/gripper_controller/follow_joint_trajectory',
            '/piper/gripper_joint_trajectory', ['gripper'])

    def on_joint_state(self, message):
        with self.state_lock:
            for name, position in zip(message.name, message.position):
                if math.isfinite(position):
                    self.positions[name] = float(position)
            self.last_state_time = time.monotonic()

    def positions_for(self, joints):
        with self.state_lock:
            if time.monotonic() - self.last_state_time > 1.0:
                return None
            values = [self.positions.get(name) for name in joints]
        return None if any(value is None for value in values) else values

    def wait_for_positions(self, joints, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.positions_for(joints) is not None:
                return True
            time.sleep(0.02)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = PiperTrajectoryActionRelay()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
