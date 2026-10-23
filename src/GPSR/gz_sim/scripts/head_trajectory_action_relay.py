#!/usr/bin/env python3
"""Expose the Gazebo camera head as a FollowJointTrajectory controller."""

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


HEAD_JOINTS = ('head_yaw_joint', 'head_pitch_joint')


def duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def ordered(values, source_names):
    if not values:
        return []
    by_name = dict(zip(source_names, values))
    return [float(by_name[name]) for name in HEAD_JOINTS]


class HeadTrajectoryActionRelay(Node):
    """Translate MoveIt controller actions into the Gazebo trajectory topic."""

    def __init__(self):
        super().__init__('head_trajectory_action_relay')
        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()
        self.goal_lock = threading.Lock()
        self.positions = {}
        self.last_state_time = 0.0
        self.goal_reserved = False
        self.publisher = self.create_publisher(
            JointTrajectory, '/head/joint_trajectory', 10)
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 20,
            callback_group=self.callback_group)
        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            '/head/motor_controller/follow_joint_trajectory',
            execute_callback=self.execute,
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel,
            callback_group=self.callback_group,
        )

    def accept_goal(self, goal):
        names = tuple(goal.trajectory.joint_names)
        if len(names) != len(HEAD_JOINTS) or set(names) != set(HEAD_JOINTS):
            self.get_logger().error(
                f'Expected joints {HEAD_JOINTS}, received {names}')
            return GoalResponse.REJECT
        if not goal.trajectory.points:
            return GoalResponse.REJECT

        previous_time = -1.0
        for point in goal.trajectory.points:
            arrays = (
                point.positions, point.velocities,
                point.accelerations, point.effort)
            point_time = duration_seconds(point.time_from_start)
            if (
                    len(point.positions) != len(HEAD_JOINTS)
                    or any(values and len(values) != len(HEAD_JOINTS)
                           for values in arrays)
                    or any(not math.isfinite(value)
                           for values in arrays for value in values)
                    or point_time <= previous_time):
                return GoalResponse.REJECT
            previous_time = point_time

        with self.goal_lock:
            if self.goal_reserved:
                return GoalResponse.REJECT
            self.goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def on_joint_state(self, message):
        with self.state_lock:
            for name, position in zip(message.name, message.position):
                if math.isfinite(position):
                    self.positions[name] = float(position)
            self.last_state_time = time.monotonic()

    def positions_for_head(self):
        with self.state_lock:
            if time.monotonic() - self.last_state_time > 1.0:
                return None
            values = [self.positions.get(name) for name in HEAD_JOINTS]
        return None if any(value is None for value in values) else values

    def wait_for_feedback(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.positions_for_head() is not None:
                return True
            time.sleep(0.02)
        return False

    def gazebo_trajectory(self, source):
        command = JointTrajectory()
        command.header = source.header
        command.joint_names = list(HEAD_JOINTS)
        for source_point in source.points:
            point = JointTrajectoryPoint()
            point.positions = ordered(source_point.positions, source.joint_names)
            point.velocities = ordered(source_point.velocities, source.joint_names)
            point.accelerations = ordered(
                source_point.accelerations, source.joint_names)
            point.effort = ordered(source_point.effort, source.joint_names)
            point.time_from_start = source_point.time_from_start
            command.points.append(point)
        return command

    def hold_current_position(self):
        positions = self.positions_for_head()
        if positions is None:
            return
        command = JointTrajectory()
        command.joint_names = list(HEAD_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.nanosec = 100_000_000
        command.points = [point]
        self.publisher.publish(command)

    async def execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            if not self.wait_for_feedback(5.0):
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = 'Gazebo head joint feedback is unavailable'
                return result

            source = goal_handle.request.trajectory
            self.publisher.publish(self.gazebo_trajectory(source))
            desired = ordered(source.points[-1].positions, source.joint_names)
            command_duration = duration_seconds(
                source.points[-1].time_from_start)
            finish_sim_time = (
                self.get_clock().now().nanoseconds * 1e-9 + command_duration)
            # A complex headless world can run substantially below real time.
            # Follow the simulation clock for trajectory completion, but retain
            # a generous wall-clock watchdog for a paused or failed simulator.
            wall_deadline = (
                time.monotonic() + max(30.0, command_duration * 20.0))

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(HEAD_JOINTS)
            feedback.desired.positions = desired
            while time.monotonic() < wall_deadline:
                if goal_handle.is_cancel_requested:
                    self.hold_current_position()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'trajectory canceled and head held'
                    return result

                actual = self.positions_for_head()
                if actual is not None:
                    errors = [target - value
                              for target, value in zip(desired, actual)]
                    feedback.actual.positions = actual
                    feedback.error.positions = errors
                    goal_handle.publish_feedback(feedback)
                    if (self.get_clock().now().nanoseconds * 1e-9
                            >= finish_sim_time
                            and max(abs(value) for value in errors) <= 0.02):
                        goal_handle.succeed()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        return result
                # rclpy Humble executes action coroutines without an asyncio
                # event loop. The multithreaded executor keeps subscriptions
                # responsive while this action callback briefly sleeps.
                time.sleep(0.05)

            self.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = 'Gazebo head did not reach the trajectory goal'
            return result
        finally:
            with self.goal_lock:
                self.goal_reserved = False


def main(args=None):
    rclpy.init(args=args)
    node = HeadTrajectoryActionRelay()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
