#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger

from putrabot_interfaces.action import PersonFollow
from putrabot_interfaces.msg import BehaviorStatus, SafetyStatus, TrackedPose, TrackedPoseArray
from putrabot_interfaces.srv import ClearPerson, GetPersonFollowStatus, SelectPerson, SetFollowParams
from person_following.utils.follow_mode import MODE_MAPPED, normalize_mode


class PersonFollowServer(Node):
    ABORT_STOP_CODES = {
        SafetyStatus.STOP_TRACKER_HEARTBEAT_TIMEOUT,
        SafetyStatus.STOP_TF_UNAVAILABLE_TIMEOUT,
        SafetyStatus.STOP_NAV2_UNAVAILABLE,
        SafetyStatus.STOP_SELECTED_TARGET_STALE_TIMEOUT,
        SafetyStatus.STOP_OBSTACLE_INHIBIT_TIMEOUT,
        SafetyStatus.STOP_BEHAVIOR_ABORTED,
    }
    CANCEL_STOP_CODES = {
        SafetyStatus.STOP_DISABLED_BY_SERVER,
    }
    SUCCESS_STOP_CODES = set()
    TERMINAL_STOP_CODES = ABORT_STOP_CODES | CANCEL_STOP_CODES | SUCCESS_STOP_CODES

    def __init__(self):
        super().__init__('person_follow_server')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('tracks_state_topic', '/person_tracks_state')
        self.declare_parameter('target_topic', '/person_target_locked')
        self.declare_parameter('facing_topic', '/person_target_facing_robot')
        self.declare_parameter('behavior_status_topic', '/person_follow/behavior_status')
        self.declare_parameter('safety_status_topic', '/person_follow/safety_status')
        self.declare_parameter('select_topic', '/person_target_select_id')
        self.declare_parameter('follow_enable_topic', '/person_follow/enable')
        self.declare_parameter('follow_mode_topic', '/person_follow/mode')
        self.declare_parameter('session_active_topic', '/person_follow/session_active')
        self.declare_parameter('tracks_state_timeout_sec', 0.5)
        self.declare_parameter('feedback_hz', 10.0)
        self.declare_parameter('behavior_node_name', '/follow_behavior_manager')
        self.declare_parameter('default_follow_distance_m', 0.8)
        self.declare_parameter('default_face_hold_sec', 10.0)
        self.declare_parameter('default_follow_mode', MODE_MAPPED)

        self.tracks_state_topic = str(self.get_parameter('tracks_state_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.facing_topic = str(self.get_parameter('facing_topic').value)
        self.behavior_status_topic = str(self.get_parameter('behavior_status_topic').value)
        self.safety_status_topic = str(self.get_parameter('safety_status_topic').value)
        self.select_topic = str(self.get_parameter('select_topic').value)
        self.follow_enable_topic = str(self.get_parameter('follow_enable_topic').value)
        self.follow_mode_topic = str(self.get_parameter('follow_mode_topic').value)
        self.session_active_topic = str(self.get_parameter('session_active_topic').value)
        self.tracks_state_timeout_sec = float(self.get_parameter('tracks_state_timeout_sec').value)
        self.feedback_hz = max(5.0, min(10.0, float(self.get_parameter('feedback_hz').value)))
        self.behavior_node_name = str(self.get_parameter('behavior_node_name').value)
        self.follow_distance_m = float(self.get_parameter('default_follow_distance_m').value)
        self.face_hold_sec = float(self.get_parameter('default_face_hold_sec').value)
        self.current_mode = normalize_mode(self.get_parameter('default_follow_mode').value)

        self.last_tracks_state_rx_sec = 0.0
        self.latest_track_ids = set()
        self.latest_track_positions: Dict[int, tuple[float, float]] = {}
        self.latest_target: Optional[TrackedPose] = None
        self.facing_robot = False
        self.session_active = False
        self.follow_enabled = False
        self.selected_track_id = 0
        self.behavior_state = 'IDLE'
        self.safety_state = 'WAITING'
        self.inhibit_active = True
        self.stop_reason = 'disabled_by_server'
        self.stop_reason_code = int(SafetyStatus.STOP_DISABLED_BY_SERVER)
        self._active_goal_handle = None

        self.sub_tracks_state = self.create_subscription(
            TrackedPoseArray, self.tracks_state_topic, self.tracks_state_cb, 10, callback_group=self.cb_group
        )
        self.sub_target = self.create_subscription(
            TrackedPose, self.target_topic, self.target_cb, 10, callback_group=self.cb_group
        )
        self.sub_facing = self.create_subscription(Bool, self.facing_topic, self.facing_cb, 10, callback_group=self.cb_group)
        self.sub_behavior = self.create_subscription(
            BehaviorStatus, self.behavior_status_topic, self.behavior_cb, 10, callback_group=self.cb_group
        )
        self.sub_mode = self.create_subscription(
            String, self.follow_mode_topic, self.mode_cb, 10, callback_group=self.cb_group
        )
        self.sub_safety = self.create_subscription(
            SafetyStatus, self.safety_status_topic, self.safety_cb, 10, callback_group=self.cb_group
        )

        self.pub_select = self.create_publisher(Int32, self.select_topic, 10)
        self.pub_follow_enable = self.create_publisher(Bool, self.follow_enable_topic, 10)
        self.pub_session_active = self.create_publisher(Bool, self.session_active_topic, 10)

        self.srv_select = self.create_service(
            SelectPerson, '/person_follow/select_person', self.handle_select_person, callback_group=self.cb_group
        )
        self.srv_clear = self.create_service(
            ClearPerson, '/person_follow/clear_person', self.handle_clear_person, callback_group=self.cb_group
        )
        self.srv_status = self.create_service(
            GetPersonFollowStatus, '/person_follow/get_status', self.handle_get_status, callback_group=self.cb_group
        )
        self.srv_stop = self.create_service(
            Trigger, '/person_follow/stop', self.handle_stop_follow, callback_group=self.cb_group
        )
        self.srv_set_params = self.create_service(
            SetFollowParams, '/person_follow/set_params', self.handle_set_params, callback_group=self.cb_group
        )

        self.action_server = ActionServer(
            self,
            PersonFollow,
            '/person_follow/control',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
        )

        set_param_service = f'{self.behavior_node_name}/set_parameters'
        self.param_client = self.create_client(SetParameters, set_param_service, callback_group=self.cb_group)

        self.get_logger().info(f'[PersonFollowServer] Started mode={self.current_mode}')
        self._set_follow_enable(False)
        self._publish_session_active()

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_session_active(self):
        msg = Bool()
        msg.data = bool(self.session_active)
        self.pub_session_active.publish(msg)

    def _set_follow_enable(self, enabled: bool):
        self.follow_enabled = bool(enabled)
        msg = Bool()
        msg.data = self.follow_enabled
        self.pub_follow_enable.publish(msg)

    def _publish_select(self, track_id: int):
        msg = Int32()
        msg.data = int(track_id)
        self.pub_select.publish(msg)

    def _is_tracks_state_fresh(self) -> bool:
        if self.last_tracks_state_rx_sec <= 0.0:
            return False
        return (self._now_sec() - self.last_tracks_state_rx_sec) <= self.tracks_state_timeout_sec

    def _choose_nearest_track_id(self) -> int:
        if not self.latest_track_positions:
            return 0
        return int(
            min(
                self.latest_track_positions.keys(),
                key=lambda tid: math.hypot(self.latest_track_positions[tid][0], self.latest_track_positions[tid][1]),
            )
        )

    def _validate_follow_params(self, follow_distance_m: float, face_hold_sec: float) -> tuple[bool, str]:
        if float(follow_distance_m) <= 0.0:
            return False, 'follow_distance_m must be > 0'
        if float(face_hold_sec) <= 0.0:
            return False, 'face_hold_sec must be > 0'
        return True, 'ok'

    def _apply_follow_params(self, follow_distance_m: float, face_hold_sec: float) -> tuple[bool, str]:
        ok, reason = self._validate_follow_params(follow_distance_m, face_hold_sec)
        if not ok:
            return False, reason
        if not self.param_client.wait_for_service(timeout_sec=0.5):
            return False, f'behavior param service not available: {self.behavior_node_name}'

        params = [
            Parameter('follow_distance_m', Parameter.Type.DOUBLE, float(follow_distance_m)),
            Parameter('face_hold_sec', Parameter.Type.DOUBLE, float(face_hold_sec)),
        ]
        req = SetParameters.Request()
        req.parameters = [p.to_parameter_msg() for p in params]
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return False, 'failed to set parameters'
        results = future.result().results
        if any(not result.successful for result in results):
            reasons = [result.reason for result in results if not result.successful and result.reason]
            return False, '; '.join(reasons) if reasons else 'parameter update rejected'
        return True, 'ok'

    def tracks_state_cb(self, msg: TrackedPoseArray):
        self.last_tracks_state_rx_sec = self._now_sec()
        ids = set()
        pos: Dict[int, tuple[float, float]] = {}
        for tr in msg.tracks:
            tid = int(tr.track_id)
            if tid <= 0:
                continue
            ids.add(tid)
            pos[tid] = (float(tr.pose.position.x), float(tr.pose.position.y))
        self.latest_track_ids = ids
        self.latest_track_positions = pos

    def target_cb(self, msg: TrackedPose):
        self.latest_target = msg
        self.selected_track_id = int(msg.track_id) if bool(msg.selected) else 0

    def facing_cb(self, msg: Bool):
        self.facing_robot = bool(msg.data)

    def behavior_cb(self, msg: BehaviorStatus):
        self.behavior_state = str(msg.state_label)

    def mode_cb(self, msg: String):
        next_mode = normalize_mode(msg.data, default='')
        if not next_mode:
            self.get_logger().warn(f'[PersonFollowServer] Ignoring invalid follow mode: {msg.data!r}')
            return
        if next_mode != self.current_mode:
            self.get_logger().info(f'[PersonFollowServer] mode: {self.current_mode} -> {next_mode}')
            self.current_mode = next_mode

    def safety_cb(self, msg: SafetyStatus):
        self.safety_state = str(msg.health_label)
        self.inhibit_active = bool(msg.inhibit_active)
        self.stop_reason = str(msg.stop_reason)
        self.stop_reason_code = int(msg.stop_reason_code)
        if self.session_active and self.stop_reason_code in self.TERMINAL_STOP_CODES:
            self.session_active = False
            self._publish_session_active()

    def handle_select_person(self, req: SelectPerson.Request, res: SelectPerson.Response):
        tid = int(req.track_id)
        if tid <= 0:
            res.success = False
            res.message = 'track_id must be > 0'
            return res
        if not self._is_tracks_state_fresh():
            res.success = False
            res.message = 'tracks_state not fresh'
            return res
        if tid not in self.latest_track_ids:
            res.success = False
            res.message = 'track_id not available'
            return res
        self._publish_select(tid)
        self.selected_track_id = tid
        res.success = True
        res.message = 'ok'
        return res

    def handle_clear_person(self, req: ClearPerson.Request, res: ClearPerson.Response):
        del req
        self._publish_select(0)
        self.selected_track_id = 0
        self.latest_target = None
        res.success = True
        res.message = 'selection_cleared'
        return res

    def _finalize_goal(self, goal_handle, stop_reason_code: int, stop_reason: str) -> Optional[PersonFollow.Result]:
        if goal_handle is None or self._active_goal_handle is not goal_handle:
            return None
        if not getattr(goal_handle, 'is_active', True):
            return None

        result = PersonFollow.Result()
        result.message = f'stopped: {stop_reason}'

        if int(stop_reason_code) in self.CANCEL_STOP_CODES:
            goal_handle.canceled()
            result.success = False
        elif int(stop_reason_code) in self.SUCCESS_STOP_CODES:
            goal_handle.succeed()
            result.success = True
        else:
            goal_handle.abort()
            result.success = False

        self._active_goal_handle = None
        return result

    def handle_stop_follow(self, req: Trigger.Request, res: Trigger.Response):
        del req
        self._set_follow_enable(False)
        self._publish_select(0)
        self.selected_track_id = 0
        self.session_active = False
        self._publish_session_active()
        res.success = True
        res.message = 'follow_stopped'
        return res

    def handle_get_status(self, req: GetPersonFollowStatus.Request, res: GetPersonFollowStatus.Response):
        del req
        target_age = 0.0
        target_visible = False
        if self.latest_target is not None:
            target_visible = bool(self.latest_target.camera_visible and self.latest_target.valid)
            target_age = float(self.latest_target.staleness_sec)

        res.active = bool(self.session_active)
        res.inhibit_active = bool(self.inhibit_active)
        res.selected_track_id = int(self.selected_track_id)
        res.target_visible = bool(target_visible)
        res.target_age_sec = float(target_age)
        res.facing_robot = bool(self.facing_robot)
        res.state = f'{self.behavior_state} ({self.current_mode})'
        res.safety_state = str(self.safety_state)
        res.stop_reason = f'{self.current_mode}:{self.stop_reason}'
        return res

    def handle_set_params(self, req: SetFollowParams.Request, res: SetFollowParams.Response):
        new_follow_distance = float(self.follow_distance_m)
        new_face_hold_sec = float(self.face_hold_sec)
        changed = []
        if float(req.follow_distance_m) > 0.0:
            new_follow_distance = float(req.follow_distance_m)
            changed.append(f'follow_distance_m={new_follow_distance:.3f}')
        if float(req.face_hold_sec) > 0.0:
            new_face_hold_sec = float(req.face_hold_sec)
            changed.append(f'face_hold_sec={new_face_hold_sec:.3f}')
        if not changed:
            res.success = False
            res.message = 'no valid parameter provided'
            return res
        ok, reason = self._apply_follow_params(new_follow_distance, new_face_hold_sec)
        res.success = bool(ok)
        if ok:
            self.follow_distance_m = new_follow_distance
            self.face_hold_sec = new_face_hold_sec
            res.message = 'updated: ' + ', '.join(changed)
        else:
            res.message = f'apply failed: {reason}'
        return res

    def goal_callback(self, goal_request: PersonFollow.Goal):
        del goal_request
        if self._active_goal_handle is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._active_goal_handle = goal_handle
        goal = goal_handle.request
        new_follow_distance = float(self.follow_distance_m)
        new_face_hold_sec = float(self.face_hold_sec)
        if float(goal.follow_distance_m) > 0.0:
            new_follow_distance = float(goal.follow_distance_m)
        if float(goal.face_hold_sec) > 0.0:
            new_face_hold_sec = float(goal.face_hold_sec)
        ok, reason = self._apply_follow_params(new_follow_distance, new_face_hold_sec)
        if not ok:
            self._set_follow_enable(False)
            self.session_active = False
            self._publish_session_active()
            goal_handle.abort()
            result = PersonFollow.Result()
            result.success = False
            result.message = f'failed to apply follow params: {reason}'
            self._active_goal_handle = None
            return result
        self.follow_distance_m = new_follow_distance
        self.face_hold_sec = new_face_hold_sec

        if bool(goal.auto_select_if_needed) and self.selected_track_id <= 0 and self._is_tracks_state_fresh():
            nearest = self._choose_nearest_track_id()
            if nearest > 0:
                self._publish_select(nearest)
                self.selected_track_id = nearest

        self.session_active = True
        self._publish_session_active()
        self._set_follow_enable(True)

        period = 1.0 / self.feedback_hz
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._set_follow_enable(False)
                    self.session_active = False
                    self._publish_session_active()
                    goal_handle.canceled()
                    result = PersonFollow.Result()
                    result.success = False
                    result.message = 'canceled'
                    self._active_goal_handle = None
                    return result

                target_age = 0.0
                target_visible = False
                if self.latest_target is not None:
                    target_visible = bool(self.latest_target.camera_visible and self.latest_target.valid)
                    target_age = float(self.latest_target.staleness_sec)

                fb = PersonFollow.Feedback()
                fb.active = bool(self.session_active)
                fb.selected_track_id = int(self.selected_track_id)
                fb.target_visible = bool(target_visible)
                fb.target_age_sec = float(target_age)
                fb.facing_robot = bool(self.facing_robot)
                fb.stop_reason = str(self.stop_reason)
                goal_handle.publish_feedback(fb)

                if (not self.session_active) and self.stop_reason_code in self.TERMINAL_STOP_CODES:
                    self._set_follow_enable(False)
                    result = self._finalize_goal(goal_handle, self.stop_reason_code, self.stop_reason)
                    if result is not None:
                        return result
                    fallback = PersonFollow.Result()
                    fallback.success = False
                    fallback.message = f'stopped: {self.stop_reason}'
                    return fallback

                time.sleep(period)
        finally:
            self._active_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
