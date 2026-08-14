#!/usr/bin/env python3
"""
Step 3 chair pointing controller.

Behavior:
1. Move arm + lift to ready state on startup.
2. Wait until a chair has been detected continuously for a configurable duration.
3. Move to point state.
4. Move toward point state.
5. Stay at point for a fixed pause.
6. Move to sleep state.
"""

import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection2DArray


class Step3ChairPointing(Node):
    """Drive the robot through ready -> point -> sleep based on chair detection."""

    def __init__(self):
        super().__init__('step3_chair_pointing')

        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('target_class', 'chair')
        self.declare_parameter('required_detection_duration_sec', 2.0)
        self.declare_parameter('detection_stale_timeout_sec', 0.75)
        self.declare_parameter('no_detection_log_period_sec', 1.0)
        self.declare_parameter('arm_action_name', '/rx200/arm_controller/follow_joint_trajectory')
        self.declare_parameter('lift_action_name', '/lift_controller/follow_joint_trajectory')
        self.declare_parameter('move_time_sec', 2.5)
        self.declare_parameter('point_pause_sec', 10.0)
        default_srdf_path = (
            f"{get_package_share_directory('rx200_handbag_grasp')}/config/combined.srdf.xacro"
        )
        self.declare_parameter('srdf_path', default_srdf_path)
        self.declare_parameter('named_state_group', 'interbotix_arm_lift')
        self.declare_parameter('ready_named_state', 'Ready')
        self.declare_parameter('point_named_state', 'point')
        self.declare_parameter('sleep_named_state', 'Sleep')
        self.declare_parameter('sleep_named_state_fallback_group', 'interbotix_arm')
        self.declare_parameter('sleep_lift_position', 0.0)

        detections_topic = self.get_parameter('detections_topic').value
        self.target_class = str(self.get_parameter('target_class').value).strip().lower()
        self.required_detection_duration_sec = float(
            self.get_parameter('required_detection_duration_sec').value
        )
        self.detection_stale_timeout_sec = float(
            self.get_parameter('detection_stale_timeout_sec').value
        )
        self.no_detection_log_period_sec = float(
            self.get_parameter('no_detection_log_period_sec').value
        )
        self.move_time_sec = float(self.get_parameter('move_time_sec').value)
        self.point_pause_sec = float(self.get_parameter('point_pause_sec').value)

        self.arm_joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        self.lift_joint_names = ['lift_joint']

        self.srdf_path = str(self.get_parameter('srdf_path').value)
        self.named_state_group = str(self.get_parameter('named_state_group').value)
        self.ready_named_state = str(self.get_parameter('ready_named_state').value)
        self.point_named_state = str(self.get_parameter('point_named_state').value)
        self.sleep_named_state = str(self.get_parameter('sleep_named_state').value)
        self.sleep_named_state_fallback_group = str(
            self.get_parameter('sleep_named_state_fallback_group').value
        )
        self.sleep_lift_position = float(self.get_parameter('sleep_lift_position').value)
        self.named_states = self._load_named_states()

        arm_action_name = self.get_parameter('arm_action_name').value
        lift_action_name = self.get_parameter('lift_action_name').value
        self.arm_client = ActionClient(self, FollowJointTrajectory, arm_action_name)
        self.lift_client = ActionClient(self, FollowJointTrajectory, lift_action_name)

        self._latest_chair_seen_time = None
        self._chair_visible_since = None
        self._last_no_chair_log_time = 0.0
        self._hold_until = None
        self._phase = 'startup'
        self._motion_seq = 0
        self._active_motion = None

        self.create_subscription(Detection2DArray, detections_topic, self._detections_cb, 10)
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info('Step3 chair pointing controller started')
        self.get_logger().info(f'  detections_topic: {detections_topic}')
        self.get_logger().info(f'  target_class: {self.target_class}')
        self.get_logger().info(f'  srdf_path: {self.srdf_path}')
        self.get_logger().info(
            f'  named states: ready={self.ready_named_state}, point={self.point_named_state}, sleep={self.sleep_named_state}'
        )

        self._wait_for_action_servers()
        self._start_motion('ready')

    def _wait_for_action_servers(self):
        """Wait for arm and lift trajectory action servers."""
        while rclpy.ok() and not self.arm_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm trajectory action server...')
        while rclpy.ok() and not self.lift_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for lift trajectory action server...')

    def _detections_cb(self, msg: Detection2DArray) -> None:
        for det in msg.detections:
            if not det.results:
                continue
            cls_name = str(det.results[0].hypothesis.class_id).strip().lower().replace('_', ' ')
            if cls_name == self.target_class:
                self._latest_chair_seen_time = time.monotonic()
                return

    def _tick(self) -> None:
        now = time.monotonic()

        if self._phase == 'waiting_for_chair':
            visible = (
                self._latest_chair_seen_time is not None and
                (now - self._latest_chair_seen_time) <= self.detection_stale_timeout_sec
            )

            if visible:
                if self._chair_visible_since is None:
                    self._chair_visible_since = now
                    self.get_logger().info(
                        f'Chair detected, starting {self.required_detection_duration_sec:.0f}-second confirmation...'
                    )
                held = now - self._chair_visible_since
                if held >= self.required_detection_duration_sec:
                    self.get_logger().info(
                        f'Chair confirmed for {self.required_detection_duration_sec:.0f} seconds, moving to point state'
                    )
                    self._start_motion('point')
            else:
                self._chair_visible_since = None
                if (now - self._last_no_chair_log_time) >= self.no_detection_log_period_sec:
                    self.get_logger().info('no chair detected')
                    self._last_no_chair_log_time = now
            return

        if self._phase == 'holding_point' and self._hold_until is not None and now >= self._hold_until:
            self.get_logger().info('Point hold finished, moving to sleep state')
            self._hold_until = None
            self._start_motion('sleep')

    def _start_motion(self, pose_name: str) -> None:
        arm_positions, lift_position = self._pose_targets(pose_name)
        self.get_logger().info(f'Moving to {pose_name} state')

        if pose_name == 'point':
            self._phase = 'holding_point'
            self._hold_until = time.monotonic() + self.point_pause_sec
            self.get_logger().info(
                f'Point command sent; sleep is scheduled in {self.point_pause_sec:.1f} seconds'
            )
        elif pose_name == 'sleep':
            self._phase = 'moving_sleep'
        else:
            self._phase = f'moving_{pose_name}'

        self._motion_seq += 1
        motion_seq = self._motion_seq
        self._active_motion = {
            'seq': motion_seq,
            'pose_name': pose_name,
            'arm_goal_handle': None,
            'lift_goal_handle': None,
            'arm_result': None,
            'lift_result': None,
        }

        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory = self._build_trajectory(
            self.arm_joint_names,
            arm_positions,
            self.move_time_sec,
        )

        lift_goal = FollowJointTrajectory.Goal()
        lift_goal.trajectory = self._build_trajectory(
            self.lift_joint_names,
            [lift_position],
            self.move_time_sec,
        )

        arm_goal_future = self.arm_client.send_goal_async(arm_goal)
        lift_goal_future = self.lift_client.send_goal_async(lift_goal)
        arm_goal_future.add_done_callback(
            lambda future: self._on_goal_response(motion_seq, 'arm', future)
        )
        lift_goal_future.add_done_callback(
            lambda future: self._on_goal_response(motion_seq, 'lift', future)
        )

    def _on_goal_response(self, motion_seq: int, side: str, future) -> None:
        motion = self._current_motion(motion_seq)
        if motion is None:
            return

        pose_name = motion['pose_name']
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_motion(pose_name, f'{side} goal response failed: {exc}')
            return

        if goal_handle is None or not goal_handle.accepted:
            self._fail_motion(pose_name, f'{side} goal for {pose_name} was rejected')
            return

        motion[f'{side}_goal_handle'] = goal_handle

        if pose_name == 'point':
            if motion['arm_goal_handle'] is not None and motion['lift_goal_handle'] is not None:
                self.get_logger().info(
                    f'Point goal accepted; sleep is scheduled in {self.point_pause_sec:.1f} seconds'
                )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result_future: self._on_result(motion_seq, side, result_future)
        )

    def _on_result(self, motion_seq: int, side: str, future) -> None:
        motion = self._current_motion(motion_seq)
        if motion is None:
            return

        pose_name = motion['pose_name']
        try:
            result = future.result()
        except Exception as exc:
            self._fail_motion(pose_name, f'{side} result for {pose_name} failed: {exc}')
            return

        motion[f'{side}_result'] = result
        if motion['arm_result'] is None or motion['lift_result'] is None:
            return

        self._finish_motion(pose_name, motion['arm_result'], motion['lift_result'])

    def _current_motion(self, motion_seq: int):
        if self._active_motion is None:
            return None
        if self._active_motion['seq'] != motion_seq:
            return None
        return self._active_motion

    def _fail_motion(self, pose_name: str, reason: str) -> None:
        self._active_motion = None
        self._phase = 'error'
        raise RuntimeError(f'{pose_name} motion failed: {reason}')

    def _finish_motion(self, pose_name: str, arm_result, lift_result) -> None:
        if self._active_motion is not None and self._active_motion['pose_name'] == pose_name:
            self._active_motion = None

        if arm_result is None or lift_result is None:
            raise RuntimeError(f'No trajectory result returned for {pose_name}')

        if arm_result.result.error_code != 0:
            raise RuntimeError(f'Arm motion to {pose_name} failed with code {arm_result.result.error_code}')
        if lift_result.result.error_code != 0:
            raise RuntimeError(f'Lift motion to {pose_name} failed with code {lift_result.result.error_code}')

        self.get_logger().info(f'{pose_name} state reached')

        if pose_name == 'ready':
            self._phase = 'waiting_for_chair'
            self._chair_visible_since = None
            self._last_no_chair_log_time = 0.0
        elif pose_name == 'point':
            self._phase = 'holding_point'
        elif pose_name == 'sleep':
            self._phase = 'done'
            self.get_logger().info('Sleep state reached, step3 sequence complete')

    def _load_named_states(self) -> dict:
        root = ET.parse(self.srdf_path).getroot()
        named_states = {}
        for group_state in root.findall('group_state'):
            group_name = group_state.attrib.get('group')
            state_name = group_state.attrib.get('name')
            if not group_name or not state_name:
                continue

            joints = {}
            for joint in group_state.findall('joint'):
                joint_name = joint.attrib.get('name')
                joint_value = joint.attrib.get('value')
                if joint_name is None or joint_value is None:
                    continue
                joints[joint_name] = float(joint_value)
            named_states[(group_name, state_name)] = joints

        return named_states

    def _get_named_state(self, group_name: str, state_name: str) -> dict:
        try:
            return self.named_states[(group_name, state_name)]
        except KeyError as exc:
            raise RuntimeError(
                f'Named state "{state_name}" for group "{group_name}" not found in {self.srdf_path}'
            ) from exc

    def _pose_targets(self, pose_name: str):
        if pose_name == 'ready':
            joints = self._get_named_state(self.named_state_group, self.ready_named_state)
            return self._split_arm_lift_joints(joints)
        if pose_name == 'point':
            joints = self._get_named_state(self.named_state_group, self.point_named_state)
            return self._split_arm_lift_joints(joints)
        if pose_name == 'sleep':
            arm_joints = self._get_named_state(
                self.sleep_named_state_fallback_group,
                self.sleep_named_state,
            )
            arm_positions = [arm_joints[joint_name] for joint_name in self.arm_joint_names]
            return arm_positions, self.sleep_lift_position
        raise ValueError(f'Unknown pose name: {pose_name}')

    def _split_arm_lift_joints(self, joints: dict):
        arm_positions = [joints[joint_name] for joint_name in self.arm_joint_names]
        lift_position = joints['lift_joint']
        return arm_positions, lift_position

    def _build_trajectory(self, joint_names, positions, move_time_sec: float) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(seconds=move_time_sec).to_msg()

        trajectory.points.append(point)
        return trajectory


def main(args=None):
    rclpy.init(args=args)
    node = Step3ChairPointing()
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
