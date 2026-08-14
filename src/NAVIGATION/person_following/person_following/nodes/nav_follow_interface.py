#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String

from putrabot_interfaces.msg import MotionIntent, SafetyStatus, TrackedPose
from person_following.utils.follow_mode import MODE_MAPPED, normalize_mode


def _yaw_from_quat_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class NavFollowInterface(Node):
    def __init__(self):
        super().__init__('nav_follow_interface')

        self.declare_parameter('target_topic', '/person_target_locked')
        self.declare_parameter('motion_intent_topic', '/person_follow/motion_intent')
        self.declare_parameter('safety_status_topic', '/person_follow/safety_status')
        self.declare_parameter('follow_enable_topic', '/person_follow/enable')
        self.declare_parameter('follow_mode_topic', '/person_follow/mode')
        self.declare_parameter('nav_available_topic', '/person_follow/nav2_available')
        self.declare_parameter('tf_ok_topic', '/person_follow/tf_ok')
        self.declare_parameter('nav_backend_status_topic', '/person_follow/nav_backend_status')
        self.declare_parameter('local_backend_status_topic', '/person_follow/local_backend_status')
        self.declare_parameter('track_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('nav2_action_name', '/navigate_to_pose')
        self.declare_parameter('cmd_vel_raw_topic', '/cmd_vel_follow')
        self.declare_parameter('goal_update_hz', 2.0)
        self.declare_parameter('min_goal_step_m', 0.30)
        self.declare_parameter('goal_resend_sec', 2.5)
        self.declare_parameter('goal_resend_cooldown_sec', 1.25)
        self.declare_parameter('goal_cancel_cooldown_sec', 1.0)
        self.declare_parameter('max_goal_distance_m', 2.5)
        self.declare_parameter('yaw_deadband_deg', 7.0)
        self.declare_parameter('angular_speed_min_rad_s', 0.12)
        self.declare_parameter('angular_speed_max_rad_s', 0.55)
        self.declare_parameter('search_goal_resend_sec', 3.5)
        self.declare_parameter('default_follow_mode', MODE_MAPPED)

        self.target_topic = str(self.get_parameter('target_topic').value)
        self.motion_intent_topic = str(self.get_parameter('motion_intent_topic').value)
        self.safety_status_topic = str(self.get_parameter('safety_status_topic').value)
        self.follow_enable_topic = str(self.get_parameter('follow_enable_topic').value)
        self.follow_mode_topic = str(self.get_parameter('follow_mode_topic').value)
        self.nav_available_topic = str(self.get_parameter('nav_available_topic').value)
        self.tf_ok_topic = str(self.get_parameter('tf_ok_topic').value)
        self.nav_backend_status_topic = str(self.get_parameter('nav_backend_status_topic').value)
        self.local_backend_status_topic = str(self.get_parameter('local_backend_status_topic').value)
        self.track_frame = str(self.get_parameter('track_frame').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.nav2_action_name = str(self.get_parameter('nav2_action_name').value)
        self.cmd_vel_raw_topic = str(self.get_parameter('cmd_vel_raw_topic').value)
        self.goal_update_hz = max(0.5, float(self.get_parameter('goal_update_hz').value))
        self.min_goal_step_m = max(0.0, float(self.get_parameter('min_goal_step_m').value))
        self.goal_resend_sec = max(0.1, float(self.get_parameter('goal_resend_sec').value))
        self.goal_resend_cooldown_sec = max(0.0, float(self.get_parameter('goal_resend_cooldown_sec').value))
        self.goal_cancel_cooldown_sec = max(0.0, float(self.get_parameter('goal_cancel_cooldown_sec').value))
        self.max_goal_distance_m = max(0.1, float(self.get_parameter('max_goal_distance_m').value))
        self.yaw_deadband_deg = float(self.get_parameter('yaw_deadband_deg').value)
        self.angular_speed_min_rad_s = max(0.0, float(self.get_parameter('angular_speed_min_rad_s').value))
        self.angular_speed_max_rad_s = max(self.angular_speed_min_rad_s, float(self.get_parameter('angular_speed_max_rad_s').value))
        self.search_goal_resend_sec = max(0.1, float(self.get_parameter('search_goal_resend_sec').value))
        self.current_mode = normalize_mode(self.get_parameter('default_follow_mode').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav2_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        self.latest_target: Optional[TrackedPose] = None
        self.latest_motion: Optional[MotionIntent] = None
        self.latest_safety: Optional[SafetyStatus] = None
        self.follow_enabled = False
        self.goal_handle: Optional[rclpy.action.client.ClientGoalHandle] = None
        self.last_goal_sent_xy: Optional[Tuple[float, float]] = None
        self.last_goal_send_sec = 0.0
        self.last_goal_cancel_sec = 0.0
        self.last_target_map_xy: Optional[Tuple[float, float]] = None
        self.last_search_goal_sec = 0.0
        self.goal_reject_count = 0
        self.goal_send_fail_count = 0
        self.goal_cancel_fail_count = 0
        self._active = False
        self._backend_state = 'IDLE'
        self._backend_reason = 'startup'
        self._last_backend_status = ''
        self._local_backend_state = 'IDLE'
        self._local_has_authority = False

        self.create_subscription(TrackedPose, self.target_topic, self._target_cb, 10)
        self.create_subscription(MotionIntent, self.motion_intent_topic, self._motion_cb, 10)
        self.create_subscription(SafetyStatus, self.safety_status_topic, self._safety_cb, 10)
        self.create_subscription(Bool, self.follow_enable_topic, self._follow_enable_cb, 10)
        self.create_subscription(String, self.follow_mode_topic, self._mode_cb, 10)
        self.create_subscription(String, self.local_backend_status_topic, self._local_backend_status_cb, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_raw_topic, 10)
        self.pub_nav_available = self.create_publisher(Bool, self.nav_available_topic, 10)
        self.pub_tf_ok = self.create_publisher(Bool, self.tf_ok_topic, 10)
        self.pub_backend_status = self.create_publisher(String, self.nav_backend_status_topic, 10)

        self.timer = self.create_timer(1.0 / self.goal_update_hz, self._timer_cb)

        self.get_logger().info('[NavFollowInterface] Started')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _target_cb(self, msg: TrackedPose):
        self.latest_target = msg
        if bool(msg.valid):
            target_map = self._point_in_map(float(msg.pose.position.x), float(msg.pose.position.y))
            if target_map is not None:
                self.last_target_map_xy = target_map

    def _motion_cb(self, msg: MotionIntent):
        self.latest_motion = msg

    def _safety_cb(self, msg: SafetyStatus):
        self.latest_safety = msg

    def _follow_enable_cb(self, msg: Bool):
        self.follow_enabled = bool(msg.data)

    def _mode_cb(self, msg: String):
        next_mode = normalize_mode(msg.data, default='')
        if not next_mode:
            self.get_logger().warn(f'[NavFollowInterface] Ignoring invalid follow mode: {msg.data!r}')
            return
        if next_mode != self.current_mode:
            self.get_logger().info(f'[NavFollowInterface] mode: {self.current_mode} -> {next_mode}')
            self.current_mode = next_mode

    def _local_backend_status_cb(self, msg: String):
        status = self._parse_backend_status(msg.data)
        self._local_backend_state = status['state']
        self._local_has_authority = status['authority']

    def _timer_cb(self):
        nav_server_ready = bool(self.nav2_client.wait_for_server(timeout_sec=0.0))
        nav_available = Bool()
        nav_available.data = nav_server_ready
        self.pub_nav_available.publish(nav_available)

        base_pose_ok = self._base_pose_in_map() is not None
        tf_ok = Bool()
        tf_ok.data = bool(base_pose_ok)
        self.pub_tf_ok.publish(tf_ok)

        if self.current_mode != MODE_MAPPED:
            self._handoff_inactive('mode_not_mapped')
            return

        if not nav_server_ready or not base_pose_ok:
            self._stop_motion('backend_not_ready')
            self._set_backend_state('FAILED', authority=False, ready=False, fault=True, reason='nav_or_tf_unavailable')
            return

        if self._local_has_authority:
            self._stop_motion('waiting_local_release')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='waiting_local_release')
            return

        self._active = True
        if not self.follow_enabled or self.latest_motion is None:
            self._stop_motion('follow_disabled_or_no_motion')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='idle_waiting')
            return
        if self.latest_safety is None or bool(self.latest_safety.inhibit_active):
            self._stop_motion('safety_inhibit')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='safety_inhibit')
            return

        mode = int(self.latest_motion.mode)
        if mode == MotionIntent.MODE_HOLD:
            self._stop_motion('motion_hold')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='hold')
            return
        if mode == MotionIntent.MODE_ROTATE:
            self._run_rotate(self.latest_motion)
            return
        if mode == MotionIntent.MODE_SEARCH:
            self._run_search()
            return
        self._run_follow_pose(self.latest_motion)

    def _run_follow_pose(self, motion: MotionIntent):
        target = self.latest_target
        if target is None or not bool(target.valid) or not bool(target.publishable):
            self._stop_motion('stale_or_unpublishable_target')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='target_unpublishable')
            return

        px = float(target.pose.position.x)
        py = float(target.pose.position.y)
        d = math.hypot(px, py)
        if d < 1e-6:
            self._stop_motion('target_too_close')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='target_too_close')
            return

        ux = px / d
        uy = py / d
        follow_distance = float(motion.follow_distance_m) if float(motion.follow_distance_m) > 0.0 else 0.8
        goal_bx = px - follow_distance * ux
        goal_by = py - follow_distance * uy
        goal_norm = math.hypot(goal_bx, goal_by)
        if goal_norm > self.max_goal_distance_m:
            scale = self.max_goal_distance_m / goal_norm
            goal_bx *= scale
            goal_by *= scale

        goal_map = self._goal_in_map(goal_bx, goal_by, px, py)
        if goal_map is None:
            self._stop_motion('goal_transform_failed')
            self._set_backend_state('FAILED', authority=False, ready=False, fault=True, reason='goal_transform_failed')
            return

        gx, gy, yaw = goal_map
        now = self._now_sec()
        if not self._should_send_goal(now, gx, gy):
            self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='holding_goal')
            return
        self._send_goal(gx, gy, yaw, now)
        self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='following_goal')

    def _run_rotate(self, motion: MotionIntent):
        self._cancel_goal_if_any(self._now_sec())
        turn = self._scaled_turn(float(motion.desired_yaw_rad))
        t = Twist()
        t.angular.z = float(turn)
        self.pub_cmd_vel.publish(t)
        self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='rotate')

    def _run_search(self):
        now = self._now_sec()
        if self.last_target_map_xy is None:
            t = Twist()
            t.angular.z = 0.45
            self.pub_cmd_vel.publish(t)
            self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='search_spin')
            return
        if (now - self.last_search_goal_sec) < self.search_goal_resend_sec and self.goal_handle is not None:
            self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='search_goal_active')
            return
        base = self._base_pose_in_map()
        if base is None:
            self._stop_motion('search_tf_missing')
            self._set_backend_state('FAILED', authority=False, ready=False, fault=True, reason='search_tf_missing')
            return
        gx, gy = self.last_target_map_xy
        _, _, yaw = base
        self._send_goal(gx, gy, yaw, now)
        self.last_search_goal_sec = now
        self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='search_goal_sent')

    def _stop_motion(self, reason: str):
        self._cancel_goal_if_any(self._now_sec())
        self.pub_cmd_vel.publish(Twist())
        self._active = False
        self._backend_reason = reason

    def _handoff_inactive(self, reason: str):
        if self.goal_handle is not None:
            self._set_backend_state('PAUSING', authority=False, ready=False, fault=False, reason=reason)
            self._cancel_goal_if_any(self._now_sec())
        if self._active:
            self.pub_cmd_vel.publish(Twist())
        self._active = False
        self._set_backend_state('IDLE', authority=False, ready=True, fault=False, reason=reason)

    def _cancel_goal_if_any(self, now_sec: float):
        if self.goal_handle is None:
            return
        if self.goal_cancel_cooldown_sec > 0.0 and (now_sec - self.last_goal_cancel_sec) < self.goal_cancel_cooldown_sec:
            return
        try:
            self.goal_handle.cancel_goal_async()
        except Exception:
            self.goal_cancel_fail_count += 1
        self.goal_handle = None
        self.last_goal_cancel_sec = now_sec

    def _should_send_goal(self, now_sec: float, gx: float, gy: float) -> bool:
        if self.last_goal_sent_xy is None:
            return True
        dt = now_sec - self.last_goal_send_sec
        if dt >= self.goal_resend_sec:
            return True
        if dt < self.goal_resend_cooldown_sec:
            return False
        dx = gx - self.last_goal_sent_xy[0]
        dy = gy - self.last_goal_sent_xy[1]
        return math.hypot(dx, dy) >= self.min_goal_step_m

    def _send_goal(self, gx: float, gy: float, yaw_map: float, now_sec: float):
        self._cancel_goal_if_any(now_sec)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        qx, qy, qz, qw = _quat_from_yaw(yaw_map)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        try:
            future = self.nav2_client.send_goal_async(goal)
            future.add_done_callback(self._on_goal_response)
            self.last_goal_send_sec = now_sec
            self.last_goal_sent_xy = (float(gx), float(gy))
        except Exception:
            self.goal_send_fail_count += 1

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception:
            self.goal_send_fail_count += 1
            self.goal_handle = None
            return
        if not goal_handle.accepted:
            self.goal_reject_count += 1
            self.goal_handle = None
            return
        self.goal_handle = goal_handle
        self.goal_reject_count = 0

    def _goal_in_map(self, goal_bx: float, goal_by: float, target_bx: float, target_by: float) -> Optional[Tuple[float, float, float]]:
        base = self._base_pose_in_map()
        if base is None:
            return None
        tx, ty, yaw = base
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        gx = tx + cy * goal_bx - sy * goal_by
        gy = ty + sy * goal_bx + cy * goal_by
        txm = tx + cy * target_bx - sy * target_by
        tym = ty + sy * target_bx + cy * target_by
        goal_yaw = math.atan2(tym - gy, txm - gx)
        return (gx, gy, goal_yaw)

    def _point_in_map(self, px_b: float, py_b: float) -> Optional[Tuple[float, float]]:
        base = self._base_pose_in_map()
        if base is None:
            return None
        tx, ty, yaw = base
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return (tx + cy * px_b - sy * py_b, ty + sy * px_b + cy * py_b)

    def _base_pose_in_map(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except Exception:
            return None
        q = tf.transform.rotation
        return (
            float(tf.transform.translation.x),
            float(tf.transform.translation.y),
            _yaw_from_quat_xyzw(float(q.x), float(q.y), float(q.z), float(q.w)),
        )

    def _scaled_turn(self, desired_yaw_rad: float) -> float:
        if abs(math.degrees(desired_yaw_rad)) <= self.yaw_deadband_deg:
            return 0.0
        turn = max(-self.angular_speed_max_rad_s, min(self.angular_speed_max_rad_s, desired_yaw_rad))
        if 0.0 < abs(turn) < self.angular_speed_min_rad_s:
            turn = math.copysign(self.angular_speed_min_rad_s, turn)
        return turn

    def _set_backend_state(self, state: str, authority: bool, ready: bool, fault: bool, reason: str):
        self._backend_state = state
        payload = f'state={state};authority={int(authority)};ready={int(ready)};fault={int(fault)};reason={reason}'
        if payload == self._last_backend_status:
            return
        msg = String()
        msg.data = payload
        self.pub_backend_status.publish(msg)
        self._last_backend_status = payload

    @staticmethod
    def _parse_backend_status(raw: str):
        result = {'state': 'IDLE', 'authority': False, 'ready': False, 'fault': False, 'reason': ''}
        for part in raw.split(';'):
            if '=' not in part:
                continue
            key, value = part.split('=', 1)
            key = key.strip()
            value = value.strip()
            if key == 'state':
                result['state'] = value or 'IDLE'
            elif key in ('authority', 'ready', 'fault'):
                result[key] = value == '1'
            elif key == 'reason':
                result['reason'] = value
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavFollowInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
