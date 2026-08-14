#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Tuple

import numpy as np

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from putrabot_interfaces.msg import MotionIntent, SafetyStatus, TrackedPose
from person_following.utils.follow_mode import MODE_MAPPED, MODE_UNMAPPED, normalize_mode


def _quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class LocalFollowInterface(Node):
    def __init__(self):
        super().__init__('local_follow_interface')

        self.declare_parameter('target_topic', '/person_target_locked')
        self.declare_parameter('motion_intent_topic', '/person_follow/motion_intent')
        self.declare_parameter('safety_status_topic', '/person_follow/safety_status')
        self.declare_parameter('follow_enable_topic', '/person_follow/enable')
        self.declare_parameter('follow_mode_topic', '/person_follow/mode')
        self.declare_parameter('obstacle_inhibit_topic', '/person_follow/obstacle_inhibit')
        self.declare_parameter('local_sensing_ok_topic', '/person_follow/local_sensing_ok')
        self.declare_parameter('local_backend_status_topic', '/person_follow/local_backend_status')
        self.declare_parameter('nav_backend_status_topic', '/person_follow/nav_backend_status')
        self.declare_parameter('cmd_vel_raw_topic', '/cmd_vel_follow')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('default_follow_mode', MODE_MAPPED)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('track_frame', 'base_footprint')
        self.declare_parameter('local_goal_frame', 'odom')
        self.declare_parameter('nav2_action_name', '/navigate_to_pose')
        self.declare_parameter('unmapped_backend', 'mppi')
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('goal_update_hz', 4.0)
        self.declare_parameter('goal_resend_sec', 2.0)
        self.declare_parameter('goal_min_step_m', 0.15)
        self.declare_parameter('goal_smoothing_alpha', 0.35)
        self.declare_parameter('max_local_goal_distance_m', 1.6)
        self.declare_parameter('goal_cancel_cooldown_sec', 0.4)
        self.declare_parameter('max_linear_speed_mps', 0.45)
        self.declare_parameter('max_angular_speed_rad_s', 0.75)
        self.declare_parameter('min_linear_speed_mps', 0.05)
        self.declare_parameter('distance_kp', 0.65)
        self.declare_parameter('yaw_kp', 1.2)
        self.declare_parameter('rotate_yaw_enter_deg', 8.0)
        self.declare_parameter('rotate_yaw_exit_deg', 5.0)
        self.declare_parameter('rotate_min_angular_speed_rad_s', 0.0)
        self.declare_parameter('follow_distance_default_m', 0.8)
        self.declare_parameter('target_stale_timeout_sec', 1.2)
        self.declare_parameter('scan_timeout_sec', 0.6)
        self.declare_parameter('depth_timeout_sec', 0.8)
        self.declare_parameter('corridor_half_width_m', 0.45)
        self.declare_parameter('corridor_lookahead_m', 1.4)
        self.declare_parameter('critical_stop_distance_m', 0.42)
        self.declare_parameter('slowdown_distance_m', 0.90)
        self.declare_parameter('side_clearance_m', 0.55)
        self.declare_parameter('bypass_trigger_sec', 1.2)
        self.declare_parameter('bypass_duration_sec', 1.4)
        self.declare_parameter('bypass_forward_speed_mps', 0.12)
        self.declare_parameter('bypass_turn_speed_rad_s', 0.45)
        self.declare_parameter('wait_before_bypass_sec', 0.6)
        self.declare_parameter('depth_stop_distance_m', 0.6)
        self.declare_parameter('depth_critical_distance_m', 0.38)
        self.declare_parameter('depth_center_ratio', 0.35)
        self.declare_parameter('depth_min_valid_pixels', 150)
        self.declare_parameter('target_mask_enabled', True)
        self.declare_parameter('target_mask_half_length_m', 0.35)
        self.declare_parameter('target_mask_half_width_m', 0.28)
        self.declare_parameter('depth_target_exclusion_margin_m', 0.18)
        self.declare_parameter('depth_min_obstacle_pixels', 60)
        self.declare_parameter('publish_debug_markers', True)
        self.declare_parameter('debug_markers_topic', '/person_follow/unmapped_debug_markers')
        self.declare_parameter('debug_status_topic', '/person_follow/local_debug')

        self.target_topic = str(self.get_parameter('target_topic').value)
        self.motion_intent_topic = str(self.get_parameter('motion_intent_topic').value)
        self.safety_status_topic = str(self.get_parameter('safety_status_topic').value)
        self.follow_enable_topic = str(self.get_parameter('follow_enable_topic').value)
        self.follow_mode_topic = str(self.get_parameter('follow_mode_topic').value)
        self.obstacle_inhibit_topic = str(self.get_parameter('obstacle_inhibit_topic').value)
        self.local_sensing_ok_topic = str(self.get_parameter('local_sensing_ok_topic').value)
        self.local_backend_status_topic = str(self.get_parameter('local_backend_status_topic').value)
        self.nav_backend_status_topic = str(self.get_parameter('nav_backend_status_topic').value)
        self.cmd_vel_raw_topic = str(self.get_parameter('cmd_vel_raw_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.current_mode = normalize_mode(self.get_parameter('default_follow_mode').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.track_frame = str(self.get_parameter('track_frame').value)
        self.local_goal_frame = str(self.get_parameter('local_goal_frame').value)
        self.nav2_action_name = str(self.get_parameter('nav2_action_name').value)
        self.unmapped_backend = str(self.get_parameter('unmapped_backend').value).strip().lower() or 'mppi'
        if self.unmapped_backend not in ('reactive', 'mppi'):
            self.unmapped_backend = 'mppi'

        self.control_hz = max(2.0, float(self.get_parameter('control_hz').value))
        self.goal_update_hz = max(0.5, float(self.get_parameter('goal_update_hz').value))
        self.goal_resend_sec = max(0.2, float(self.get_parameter('goal_resend_sec').value))
        self.goal_min_step_m = max(0.01, float(self.get_parameter('goal_min_step_m').value))
        self.goal_smoothing_alpha = min(1.0, max(0.0, float(self.get_parameter('goal_smoothing_alpha').value)))
        self.max_local_goal_distance_m = max(0.1, float(self.get_parameter('max_local_goal_distance_m').value))
        self.goal_cancel_cooldown_sec = max(0.0, float(self.get_parameter('goal_cancel_cooldown_sec').value))
        self.max_linear_speed = max(0.05, float(self.get_parameter('max_linear_speed_mps').value))
        self.max_angular_speed = max(0.1, float(self.get_parameter('max_angular_speed_rad_s').value))
        self.min_linear_speed = max(0.0, float(self.get_parameter('min_linear_speed_mps').value))
        self.distance_kp = max(0.0, float(self.get_parameter('distance_kp').value))
        self.yaw_kp = max(0.0, float(self.get_parameter('yaw_kp').value))
        self.rotate_yaw_enter_deg = max(0.0, float(self.get_parameter('rotate_yaw_enter_deg').value))
        self.rotate_yaw_exit_deg = max(0.0, min(self.rotate_yaw_enter_deg, float(self.get_parameter('rotate_yaw_exit_deg').value)))
        self.rotate_min_angular_speed = max(0.0, float(self.get_parameter('rotate_min_angular_speed_rad_s').value))
        self.follow_distance_default = max(0.1, float(self.get_parameter('follow_distance_default_m').value))
        self.target_stale_timeout_sec = max(0.1, float(self.get_parameter('target_stale_timeout_sec').value))
        self.scan_timeout_sec = max(0.1, float(self.get_parameter('scan_timeout_sec').value))
        self.depth_timeout_sec = max(0.1, float(self.get_parameter('depth_timeout_sec').value))
        self.corridor_half_width_m = max(0.1, float(self.get_parameter('corridor_half_width_m').value))
        self.corridor_lookahead_m = max(0.2, float(self.get_parameter('corridor_lookahead_m').value))
        self.critical_stop_distance_m = max(0.1, float(self.get_parameter('critical_stop_distance_m').value))
        self.slowdown_distance_m = max(self.critical_stop_distance_m, float(self.get_parameter('slowdown_distance_m').value))
        self.side_clearance_m = max(0.1, float(self.get_parameter('side_clearance_m').value))
        self.bypass_trigger_sec = max(0.1, float(self.get_parameter('bypass_trigger_sec').value))
        self.bypass_duration_sec = max(0.1, float(self.get_parameter('bypass_duration_sec').value))
        self.bypass_forward_speed = max(0.0, float(self.get_parameter('bypass_forward_speed_mps').value))
        self.bypass_turn_speed = max(0.1, float(self.get_parameter('bypass_turn_speed_rad_s').value))
        self.wait_before_bypass_sec = max(0.0, float(self.get_parameter('wait_before_bypass_sec').value))
        self.depth_stop_distance_m = max(0.1, float(self.get_parameter('depth_stop_distance_m').value))
        self.depth_critical_distance_m = max(0.1, float(self.get_parameter('depth_critical_distance_m').value))
        self.depth_center_ratio = min(0.9, max(0.1, float(self.get_parameter('depth_center_ratio').value)))
        self.depth_min_valid_pixels = max(1, int(self.get_parameter('depth_min_valid_pixels').value))
        self.target_mask_enabled = bool(self.get_parameter('target_mask_enabled').value)
        self.target_mask_half_length_m = max(0.05, float(self.get_parameter('target_mask_half_length_m').value))
        self.target_mask_half_width_m = max(0.05, float(self.get_parameter('target_mask_half_width_m').value))
        self.depth_target_exclusion_margin_m = max(0.05, float(self.get_parameter('depth_target_exclusion_margin_m').value))
        self.depth_min_obstacle_pixels = max(1, int(self.get_parameter('depth_min_obstacle_pixels').value))
        self.publish_debug_markers = bool(self.get_parameter('publish_debug_markers').value)
        self.debug_markers_topic = str(self.get_parameter('debug_markers_topic').value)
        self.debug_status_topic = str(self.get_parameter('debug_status_topic').value)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav2_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        self.latest_target: Optional[TrackedPose] = None
        self.latest_motion: Optional[MotionIntent] = None
        self.latest_safety: Optional[SafetyStatus] = None
        self.follow_enabled = False
        self.last_scan_sec = 0.0
        self.last_depth_sec = 0.0
        self.last_scan: Optional[LaserScan] = None
        self.last_depth_min_m: Optional[float] = None
        self.last_depth_critical = False
        self.last_depth_blocked = False
        self._active = False
        self._blocked_since_sec = 0.0
        self._bypass_until_sec = 0.0
        self._bypass_direction = 0.0
        self._last_inhibit = False
        self._last_sensing_ok = False
        self._backend_state = 'IDLE'
        self._backend_reason = 'startup'
        self._last_backend_status = ''
        self._nav_backend_state = 'IDLE'
        self._nav_has_authority = False
        self._rotate_active = False

        self.goal_handle = None
        self.last_goal_sent_xy: Optional[Tuple[float, float]] = None
        self.last_goal_send_sec = 0.0
        self.last_goal_cancel_sec = 0.0
        self.smoothed_goal_xy: Optional[np.ndarray] = None
        self.last_goal_frame = self.local_goal_frame
        self.last_debug_publish_sec = 0.0
        self._debug_raw_goal_xy: Optional[Tuple[float, float]] = None
        self._debug_target_xy: Optional[Tuple[float, float]] = None
        self._debug_goal_yaw: float = 0.0
        self._goal_valid = False
        self._goal_age_sec = 0.0
        self._last_goal_status_reason = 'idle'
        self._goal_send_fail_count = 0
        self._goal_reject_count = 0

        self.create_subscription(TrackedPose, self.target_topic, self._target_cb, 10)
        self.create_subscription(MotionIntent, self.motion_intent_topic, self._motion_cb, 10)
        self.create_subscription(SafetyStatus, self.safety_status_topic, self._safety_cb, 10)
        self.create_subscription(Bool, self.follow_enable_topic, self._follow_enable_cb, 10)
        self.create_subscription(String, self.follow_mode_topic, self._mode_cb, 10)
        self.create_subscription(String, self.nav_backend_status_topic, self._nav_backend_status_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_raw_topic, 10)
        self.pub_obstacle = self.create_publisher(Bool, self.obstacle_inhibit_topic, 10)
        self.pub_sensing_ok = self.create_publisher(Bool, self.local_sensing_ok_topic, 10)
        self.pub_backend_status = self.create_publisher(String, self.local_backend_status_topic, 10)
        self.pub_debug_status = self.create_publisher(String, self.debug_status_topic, 10)
        self.pub_debug_markers = self.create_publisher(MarkerArray, self.debug_markers_topic, 10)

        self.timer = self.create_timer(1.0 / self.control_hz, self._timer_cb)
        self.get_logger().info(
            f'[LocalFollowInterface] Started mode={self.current_mode} backend={self.unmapped_backend}'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _target_cb(self, msg: TrackedPose):
        self.latest_target = msg

    def _motion_cb(self, msg: MotionIntent):
        self.latest_motion = msg

    def _safety_cb(self, msg: SafetyStatus):
        self.latest_safety = msg

    def _follow_enable_cb(self, msg: Bool):
        self.follow_enabled = bool(msg.data)

    def _mode_cb(self, msg: String):
        next_mode = normalize_mode(msg.data, default='')
        if not next_mode:
            self.get_logger().warn(f'[LocalFollowInterface] Ignoring invalid follow mode: {msg.data!r}')
            return
        if next_mode != self.current_mode:
            self.get_logger().info(f'[LocalFollowInterface] mode: {self.current_mode} -> {next_mode}')
            self.current_mode = next_mode

    def _nav_backend_status_cb(self, msg: String):
        status = self._parse_backend_status(msg.data)
        self._nav_backend_state = status['state']
        self._nav_has_authority = status['authority']

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_sec = self._now_sec()

    def _depth_cb(self, msg: Image):
        self.last_depth_sec = self._now_sec()
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'[LocalFollowInterface] Depth conversion failed: {exc}')
            self.last_depth_min_m = None
            self.last_depth_blocked = False
            self.last_depth_critical = False
            return

        arr = np.asarray(img)
        if arr.size == 0:
            self.last_depth_min_m = None
            self.last_depth_blocked = False
            self.last_depth_critical = False
            return
        if arr.dtype.kind in ('u', 'i'):
            arr = arr.astype(np.float32) * 0.001
        else:
            arr = arr.astype(np.float32)

        h, w = arr.shape[:2]
        cx_half = max(1, int(0.5 * w * self.depth_center_ratio))
        x0 = max(0, w // 2 - cx_half)
        x1 = min(w, w // 2 + cx_half)
        roi = arr[:, x0:x1]
        valid = np.isfinite(roi) & (roi > 0.05)
        if int(np.count_nonzero(valid)) < self.depth_min_valid_pixels:
            self.last_depth_min_m = None
            self.last_depth_blocked = False
            self.last_depth_critical = False
            return
        vals = roi[valid]
        target_xy = self._selected_target_xy()
        depth_vals = vals
        if target_xy is not None:
            target_dist = math.hypot(target_xy[0], target_xy[1])
            cutoff = max(0.05, target_dist - self.depth_target_exclusion_margin_m)
            obstacle_vals = vals[vals < cutoff]
            if obstacle_vals.size >= self.depth_min_obstacle_pixels:
                depth_vals = obstacle_vals
            else:
                self.last_depth_min_m = None
                self.last_depth_blocked = False
                self.last_depth_critical = False
                return
        depth_min = float(np.percentile(depth_vals, 10))
        self.last_depth_min_m = depth_min
        self.last_depth_blocked = depth_min <= self.depth_stop_distance_m
        self.last_depth_critical = depth_min <= self.depth_critical_distance_m

    def _timer_cb(self):
        now = self._now_sec()
        sensing_ok = self._sensing_ok(now)
        self._publish_bool(self.pub_sensing_ok, sensing_ok, '_last_sensing_ok')

        if self.current_mode != MODE_UNMAPPED:
            self._handoff_inactive('mode_not_unmapped')
            self._publish_debug(now)
            return

        if self._nav_has_authority:
            self._cancel_goal_if_any(now, reason='waiting_nav_release')
            self.pub_cmd_vel.publish(Twist())
            self._active = False
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='waiting_nav_release')
            self._publish_debug(now)
            return

        self._active = True
        cmd_to_publish: Optional[Twist]
        if self.unmapped_backend == 'reactive':
            cmd_to_publish = self._build_reactive_command(now, sensing_ok)
        else:
            cmd_to_publish = self._run_mppi_backend(now, sensing_ok)

        if cmd_to_publish is not None:
            self.pub_cmd_vel.publish(cmd_to_publish)

        self._publish_debug(now)

    def _run_mppi_backend(self, now: float, sensing_ok: bool) -> Optional[Twist]:
        stop_cmd = Twist()
        if not self.follow_enabled or self.latest_motion is None:
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='idle_waiting')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='idle_waiting')
            return stop_cmd
        if not sensing_ok:
            self._publish_bool(self.pub_obstacle, True, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='local_sensing_not_ready')
            self._set_backend_state('FAILED', authority=False, ready=False, fault=True, reason='local_sensing_not_ready')
            return stop_cmd
        if self._blocked_by_external_safety():
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='safety_inhibit')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='safety_inhibit')
            return stop_cmd

        motion_mode = int(self.latest_motion.mode)
        if motion_mode == MotionIntent.MODE_HOLD:
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='hold')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='hold')
            return stop_cmd
        if motion_mode == MotionIntent.MODE_ROTATE:
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='rotate')
            cmd = self._build_rotate_command(self.latest_motion)
            self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='rotate')
            return cmd
        if motion_mode == MotionIntent.MODE_SEARCH:
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='search')
            cmd = Twist()
            cmd.angular.z = float(
                self._clamp(self.latest_motion.rotate_speed_rad_s, -self.max_angular_speed, self.max_angular_speed)
            )
            self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='search')
            return cmd

        target = self.latest_target
        if not self._target_ok(target, require_publishable=True):
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='target_unpublishable')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='target_unpublishable')
            return stop_cmd

        corridor = self._corridor_observation(target_xy=(float(target.pose.position.x), float(target.pose.position.y)))
        critical = corridor['critical'] or self.last_depth_critical
        self._publish_bool(self.pub_obstacle, critical, '_last_inhibit')
        if critical:
            self._cancel_goal_if_any(now, reason='critical_obstacle')
            self._set_backend_state('BLOCKED', authority=False, ready=True, fault=False, reason='critical_obstacle')
            return stop_cmd

        if not self.nav2_client.wait_for_server(timeout_sec=0.0):
            self._cancel_goal_if_any(now, reason='nav2_unavailable')
            self._set_backend_state('FAILED', authority=False, ready=False, fault=True, reason='nav2_unavailable')
            return stop_cmd

        goal = self._compute_follow_goal(target, self.latest_motion)
        if goal is None:
            self._cancel_goal_if_any(now, reason='goal_generation_invalid')
            self._set_backend_state('FAILED', authority=False, ready=False, fault=True, reason='goal_generation_invalid')
            return stop_cmd

        gx, gy, yaw = goal
        self._goal_valid = True
        self._goal_age_sec = max(0.0, now - self.last_goal_send_sec) if self.last_goal_send_sec > 0.0 else 0.0
        if not self._should_send_goal(now, gx, gy):
            self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='holding_goal')
            self._last_goal_status_reason = 'holding_goal'
            return None

        self._send_goal(gx, gy, yaw, now)
        self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='following_goal')
        self._last_goal_status_reason = 'following_goal'
        return None

    def _compute_follow_goal(self, target: TrackedPose, motion: MotionIntent) -> Optional[Tuple[float, float, float]]:
        px = float(target.pose.position.x)
        py = float(target.pose.position.y)
        distance = math.hypot(px, py)
        self._debug_target_xy = (px, py)
        if distance < 1e-6:
            return None

        follow_distance = float(motion.follow_distance_m) if float(motion.follow_distance_m) > 0.0 else self.follow_distance_default
        ux = px / distance
        uy = py / distance
        goal_bx = px - follow_distance * ux
        goal_by = py - follow_distance * uy
        goal_norm = math.hypot(goal_bx, goal_by)
        if goal_norm > self.max_local_goal_distance_m:
            scale = self.max_local_goal_distance_m / max(goal_norm, 1e-6)
            goal_bx *= scale
            goal_by *= scale
        self._debug_raw_goal_xy = (goal_bx, goal_by)

        base = self._pose_in_frame(self.local_goal_frame, self.base_frame)
        if base is None:
            return None
        bx, by, base_yaw = base
        cy = math.cos(base_yaw)
        sy = math.sin(base_yaw)
        raw_goal = np.array([
            bx + cy * goal_bx - sy * goal_by,
            by + sy * goal_bx + cy * goal_by,
        ], dtype=np.float64)
        target_in_goal = np.array([
            bx + cy * px - sy * py,
            by + sy * px + cy * py,
        ], dtype=np.float64)
        if self.smoothed_goal_xy is None:
            self.smoothed_goal_xy = raw_goal.copy()
        else:
            self.smoothed_goal_xy = (
                self.goal_smoothing_alpha * raw_goal
                + (1.0 - self.goal_smoothing_alpha) * self.smoothed_goal_xy
            )
        goal_yaw = math.atan2(target_in_goal[1] - self.smoothed_goal_xy[1], target_in_goal[0] - self.smoothed_goal_xy[0])
        self._debug_goal_yaw = float(goal_yaw)
        return (float(self.smoothed_goal_xy[0]), float(self.smoothed_goal_xy[1]), float(goal_yaw))

    def _pose_in_frame(self, frame_id: str, child_frame: str) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(frame_id, child_frame, Time())
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
            1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z)),
        )
        return (
            float(tf.transform.translation.x),
            float(tf.transform.translation.y),
            float(yaw),
        )

    def _send_goal(self, gx: float, gy: float, yaw: float, now_sec: float):
        self._cancel_goal_if_any(now_sec, reason='goal_refresh')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.local_goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        try:
            future = self.nav2_client.send_goal_async(goal)
            future.add_done_callback(self._on_goal_response)
            self.last_goal_send_sec = now_sec
            self.last_goal_sent_xy = (float(gx), float(gy))
            self.last_goal_frame = self.local_goal_frame
        except Exception as exc:
            self._goal_send_fail_count += 1
            self.get_logger().warn(f'[LocalFollowInterface] Failed to send local goal: {exc}')

    def _cancel_goal_if_any(self, now_sec: float, reason: str):
        if self.goal_handle is None:
            self._last_goal_status_reason = reason
            return
        if self.goal_cancel_cooldown_sec > 0.0 and (now_sec - self.last_goal_cancel_sec) < self.goal_cancel_cooldown_sec:
            return
        try:
            self.goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warn(f'[LocalFollowInterface] Failed to cancel goal: {exc}')
        self.goal_handle = None
        self.last_goal_cancel_sec = now_sec
        self._last_goal_status_reason = reason

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._goal_send_fail_count += 1
            self.goal_handle = None
            self.get_logger().warn(f'[LocalFollowInterface] Goal response failed: {exc}')
            return
        if not goal_handle.accepted:
            self._goal_reject_count += 1
            self.goal_handle = None
            self.get_logger().warn('[LocalFollowInterface] Local goal rejected by Nav2')
            return
        self.goal_handle = goal_handle

    def _should_send_goal(self, now_sec: float, gx: float, gy: float) -> bool:
        if self.last_goal_sent_xy is None:
            return True
        dt = now_sec - self.last_goal_send_sec
        if dt < (1.0 / self.goal_update_hz):
            return False
        if dt >= self.goal_resend_sec:
            return True
        dx = gx - self.last_goal_sent_xy[0]
        dy = gy - self.last_goal_sent_xy[1]
        return math.hypot(dx, dy) >= self.goal_min_step_m

    def _build_reactive_command(self, now: float, sensing_ok: bool) -> Optional[Twist]:
        cmd = Twist()
        if not self.follow_enabled or self.latest_motion is None:
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='idle_waiting')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='reactive_idle_waiting')
            return cmd
        if not sensing_ok:
            self._publish_bool(self.pub_obstacle, True, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='local_sensing_not_ready')
            self._set_backend_state('FAILED', authority=False, ready=False, fault=True, reason='local_sensing_not_ready')
            return cmd
        if self._blocked_by_external_safety():
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='safety_inhibit')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='safety_inhibit')
            return cmd

        motion_mode = int(self.latest_motion.mode)
        if motion_mode == MotionIntent.MODE_HOLD:
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='hold')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='hold')
            return cmd
        if motion_mode == MotionIntent.MODE_ROTATE:
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='rotate')
            rotate_cmd = self._build_rotate_command(self.latest_motion)
            self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='reactive_rotate')
            return rotate_cmd
        if motion_mode == MotionIntent.MODE_SEARCH:
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='search')
            cmd.angular.z = float(self._clamp(
                self.latest_motion.rotate_speed_rad_s,
                -self.max_angular_speed,
                self.max_angular_speed,
            ))
            self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='reactive_search')
            return cmd

        target = self.latest_target
        if not self._target_ok(target, require_publishable=True):
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._cancel_goal_if_any(now, reason='target_unpublishable')
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='target_unpublishable')
            return cmd

        px = float(target.pose.position.x)
        py = float(target.pose.position.y)
        distance = math.hypot(px, py)
        bearing = math.atan2(py, px)
        follow_distance = float(self.latest_motion.follow_distance_m) if float(self.latest_motion.follow_distance_m) > 0.0 else self.follow_distance_default

        self._debug_target_xy = (px, py)
        self._debug_raw_goal_xy = (px, py)
        self.smoothed_goal_xy = np.array([px, py], dtype=np.float64)
        self._debug_goal_yaw = float(bearing)
        self._goal_valid = True
        self._goal_age_sec = 0.0
        self._last_goal_status_reason = 'reactive_direct'

        corridor = self._corridor_observation(target_xy=(px, py))
        critical = corridor['critical'] or self.last_depth_critical
        direct_blocked = corridor['blocked'] or self.last_depth_blocked
        left_clearance = min(float(corridor['left_clearance']), self.last_depth_min_m or float('inf'))
        right_clearance = min(float(corridor['right_clearance']), self.last_depth_min_m or float('inf'))

        if critical:
            self._blocked_since_sec = now if self._blocked_since_sec <= 0.0 else self._blocked_since_sec
            self._bypass_until_sec = 0.0
            self._bypass_direction = 0.0
            self._publish_bool(self.pub_obstacle, True, '_last_inhibit')
            self._set_backend_state('BLOCKED', authority=False, ready=True, fault=False, reason='critical_obstacle')
            return cmd

        if self._bypass_until_sec > now:
            if not self._target_ok(target, require_publishable=True):
                self._bypass_until_sec = 0.0
                self._bypass_direction = 0.0
                self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
                self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='bypass_cancelled')
                return cmd
            cmd.linear.x = min(self.bypass_forward_speed, self.max_linear_speed)
            cmd.angular.z = self._bypass_direction * self.bypass_turn_speed
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='reactive_bypass')
            return cmd

        if direct_blocked:
            if self._blocked_since_sec <= 0.0:
                self._blocked_since_sec = now
            blocked_dt = now - self._blocked_since_sec
            if blocked_dt < self.wait_before_bypass_sec:
                self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
                self._set_backend_state('PAUSING', authority=False, ready=True, fault=False, reason='waiting_bypass')
                return cmd
            direction = self._choose_bypass_direction(left_clearance, right_clearance)
            if blocked_dt >= self.bypass_trigger_sec and direction != 0.0 and self._target_ok(target, require_publishable=True):
                self._bypass_direction = direction
                self._bypass_until_sec = now + self.bypass_duration_sec
                cmd.linear.x = min(self.bypass_forward_speed, self.max_linear_speed)
                cmd.angular.z = direction * self.bypass_turn_speed
                self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
                self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='bypass_start')
                return cmd
            cmd.angular.z = self._clamp(self.yaw_kp * bearing, -self.max_angular_speed, self.max_angular_speed)
            self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
            self._set_backend_state('PAUSING', authority=False, ready=True, fault=False, reason='blocked_rotate_only')
            return cmd

        self._blocked_since_sec = 0.0
        self._bypass_direction = 0.0
        self._bypass_until_sec = 0.0
        self._publish_bool(self.pub_obstacle, False, '_last_inhibit')

        distance_error = max(0.0, distance - follow_distance)
        linear = self.distance_kp * distance_error
        angular = self.yaw_kp * bearing
        if abs(bearing) > math.radians(35.0):
            linear *= 0.35
        elif abs(bearing) > math.radians(20.0):
            linear *= 0.65

        nearest_forward = min(float(corridor['forward_clearance']), self.last_depth_min_m or float('inf'))
        if nearest_forward < self.slowdown_distance_m:
            scale = max(
                0.0,
                min(
                    1.0,
                    (nearest_forward - self.critical_stop_distance_m)
                    / max(1e-3, self.slowdown_distance_m - self.critical_stop_distance_m),
                ),
            )
            linear *= scale

        cmd.linear.x = self._clamp(
            linear,
            0.0,
            min(self.max_linear_speed, float(self.latest_motion.speed_limit_mps) or self.max_linear_speed),
        )
        if 0.0 < cmd.linear.x < self.min_linear_speed:
            cmd.linear.x = self.min_linear_speed
        cmd.angular.z = self._clamp(angular, -self.max_angular_speed, self.max_angular_speed)
        if cmd.linear.x > 0.0 or abs(cmd.angular.z) > 0.0:
            self._set_backend_state('ACTIVE', authority=True, ready=True, fault=False, reason='reactive_follow')
        else:
            self._set_backend_state('READY', authority=False, ready=True, fault=False, reason='zero_command')
        return cmd

    def _handoff_inactive(self, reason: str):
        self._cancel_goal_if_any(self._now_sec(), reason=reason)
        if self._active:
            self.pub_cmd_vel.publish(Twist())
        self._active = False
        self._rotate_active = False
        self._blocked_since_sec = 0.0
        self._bypass_until_sec = 0.0
        self._bypass_direction = 0.0
        self._publish_bool(self.pub_obstacle, False, '_last_inhibit')
        self._set_backend_state('IDLE', authority=False, ready=True, fault=False, reason=reason)
        self._goal_valid = False
        self._goal_age_sec = 0.0

    def _blocked_by_external_safety(self) -> bool:
        if self.latest_safety is None or not bool(self.latest_safety.inhibit_active):
            return False
        code = int(self.latest_safety.stop_reason_code)
        return code not in (
            SafetyStatus.STOP_WAITING_FOR_TARGET,
            SafetyStatus.STOP_SELECTED_TARGET_PENDING,
            SafetyStatus.STOP_OBSTACLE_INHIBIT,
        )

    def _target_ok(self, target: Optional[TrackedPose], require_publishable: bool = False) -> bool:
        if target is None:
            return False
        if not bool(target.selected) or not bool(target.valid):
            return False
        if require_publishable and not bool(target.publishable):
            return False
        if float(target.staleness_sec) > self.target_stale_timeout_sec:
            return False
        if not math.isfinite(float(target.pose.position.x)) or not math.isfinite(float(target.pose.position.y)):
            return False
        return True

    def _selected_target_xy(self) -> Optional[Tuple[float, float]]:
        target = self.latest_target
        if not self._target_ok(target, require_publishable=True):
            return None
        return (float(target.pose.position.x), float(target.pose.position.y))

    def _scan_point_is_selected_target(
        self,
        x: float,
        y: float,
        target_xy: Optional[Tuple[float, float]],
    ) -> bool:
        if not self.target_mask_enabled or target_xy is None:
            return False
        tx, ty = target_xy
        return (
            abs(x - tx) <= self.target_mask_half_length_m
            and abs(y - ty) <= self.target_mask_half_width_m
        )

    def _sensing_ok(self, now: float) -> bool:
        scan_ok = self.last_scan_sec > 0.0 and (now - self.last_scan_sec) <= self.scan_timeout_sec
        depth_ok = self.last_depth_sec > 0.0 and (now - self.last_depth_sec) <= self.depth_timeout_sec
        return scan_ok and depth_ok

    def _build_rotate_command(self, motion: MotionIntent) -> Twist:
        cmd = Twist()
        yaw_error = float(motion.desired_yaw_rad)
        abs_deg = abs(math.degrees(yaw_error))
        enter_deg = self.rotate_yaw_enter_deg
        exit_deg = min(self.rotate_yaw_exit_deg, enter_deg)

        if self._rotate_active:
            if abs_deg <= exit_deg:
                self._rotate_active = False
        else:
            self._rotate_active = abs_deg >= enter_deg

        if not self._rotate_active:
            return cmd

        angular = self._clamp(self.yaw_kp * yaw_error, -self.max_angular_speed, self.max_angular_speed)
        if self.rotate_min_angular_speed > 0.0 and abs(angular) > 1e-6:
            angular = math.copysign(max(abs(angular), self.rotate_min_angular_speed), angular)
            angular = self._clamp(angular, -self.max_angular_speed, self.max_angular_speed)
        cmd.angular.z = float(angular)
        return cmd

    def _corridor_observation(self, target_xy: Optional[Tuple[float, float]] = None):
        forward_clearance = float('inf')
        left_clearance = float('inf')
        right_clearance = float('inf')
        blocked = False
        critical = False

        scan = self.last_scan
        if scan is None:
            return {
                'forward_clearance': forward_clearance,
                'left_clearance': left_clearance,
                'right_clearance': right_clearance,
                'blocked': True,
                'critical': True,
            }

        angle = float(scan.angle_min)
        for rng in scan.ranges:
            if not math.isfinite(rng) or rng <= 0.05:
                angle += float(scan.angle_increment)
                continue
            x = float(rng) * math.cos(angle)
            y = float(rng) * math.sin(angle)
            angle += float(scan.angle_increment)
            if x <= 0.0:
                continue
            if self._scan_point_is_selected_target(x, y, target_xy):
                continue
            if abs(y) <= self.corridor_half_width_m and x <= self.corridor_lookahead_m:
                forward_clearance = min(forward_clearance, x)
                blocked = blocked or x <= self.slowdown_distance_m
                critical = critical or x <= self.critical_stop_distance_m
            if 0.0 < y <= self.side_clearance_m and x <= self.corridor_lookahead_m:
                left_clearance = min(left_clearance, math.hypot(x, y))
            if -self.side_clearance_m <= y < 0.0 and x <= self.corridor_lookahead_m:
                right_clearance = min(right_clearance, math.hypot(x, y))

        return {
            'forward_clearance': forward_clearance,
            'left_clearance': left_clearance,
            'right_clearance': right_clearance,
            'blocked': blocked,
            'critical': critical,
        }

    def _choose_bypass_direction(self, left_clearance: float, right_clearance: float) -> float:
        if not math.isfinite(left_clearance):
            left_clearance = self.corridor_lookahead_m + self.side_clearance_m
        if not math.isfinite(right_clearance):
            right_clearance = self.corridor_lookahead_m + self.side_clearance_m
        margin = 0.20
        if left_clearance >= self.side_clearance_m and left_clearance > (right_clearance + margin):
            return 1.0
        if right_clearance >= self.side_clearance_m and right_clearance > (left_clearance + margin):
            return -1.0
        return 0.0

    def _publish_bool(self, publisher, value: bool, attr_name: str):
        last = getattr(self, attr_name)
        if bool(last) == bool(value):
            return
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)
        setattr(self, attr_name, bool(value))

    def _set_backend_state(self, state: str, authority: bool, ready: bool, fault: bool, reason: str):
        self._backend_state = state
        self._backend_reason = reason
        payload = f'state={state};authority={int(authority)};ready={int(ready)};fault={int(fault)};reason={reason}'
        if payload == self._last_backend_status:
            return
        msg = String()
        msg.data = payload
        self.pub_backend_status.publish(msg)
        self._last_backend_status = payload

    def _publish_debug(self, now_sec: float):
        msg = String()
        goal_x = float(self.smoothed_goal_xy[0]) if self.smoothed_goal_xy is not None else 0.0
        goal_y = float(self.smoothed_goal_xy[1]) if self.smoothed_goal_xy is not None else 0.0
        target_x = float(self._debug_target_xy[0]) if self._debug_target_xy is not None else 0.0
        target_y = float(self._debug_target_xy[1]) if self._debug_target_xy is not None else 0.0
        msg.data = (
            f'backend={self.unmapped_backend};state={self._backend_state};reason={self._backend_reason};'
            f'goal_valid={int(self._goal_valid)};goal_age={self._goal_age_sec:.2f};'
            f'target_x={target_x:.2f};target_y={target_y:.2f};'
            f'goal_x={goal_x:.2f};goal_y={goal_y:.2f};goal_status={self._last_goal_status_reason};'
            f'goal_send_fail={self._goal_send_fail_count};goal_reject={self._goal_reject_count}'
        )
        self.pub_debug_status.publish(msg)

        if not self.publish_debug_markers:
            return
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        markers.markers.append(self._make_delete_marker('local_debug', 1000, self.track_frame, stamp))
        markers.markers.append(self._make_delete_marker('local_debug', 1001, self.track_frame, stamp))
        markers.markers.append(self._make_delete_marker('local_debug', 1002, self.local_goal_frame, stamp))
        markers.markers.append(self._make_delete_marker('local_debug', 1003, self.track_frame, stamp))

        if self._debug_target_xy is not None:
            markers.markers.append(self._make_sphere_marker(
                frame_id=self.track_frame,
                stamp=stamp,
                marker_id=1000,
                x=self._debug_target_xy[0],
                y=self._debug_target_xy[1],
                z=0.05,
                rgb=(0.1, 0.7, 0.9),
                scale=0.16,
            ))
        if self._debug_raw_goal_xy is not None:
            markers.markers.append(self._make_sphere_marker(
                frame_id=self.track_frame,
                stamp=stamp,
                marker_id=1001,
                x=self._debug_raw_goal_xy[0],
                y=self._debug_raw_goal_xy[1],
                z=0.05,
                rgb=(0.2, 0.9, 0.2),
                scale=0.12,
            ))
        if self.smoothed_goal_xy is not None:
            markers.markers.append(self._make_sphere_marker(
                frame_id=self.local_goal_frame,
                stamp=stamp,
                marker_id=1002,
                x=float(self.smoothed_goal_xy[0]),
                y=float(self.smoothed_goal_xy[1]),
                z=0.05,
                rgb=(0.95, 0.4, 0.1),
                scale=0.14,
            ))
        if self._debug_target_xy is not None and self._debug_raw_goal_xy is not None:
            markers.markers.append(self._make_line_marker(
                frame_id=self.track_frame,
                stamp=stamp,
                marker_id=1003,
                start=(0.0, 0.0, 0.06),
                end=(self._debug_target_xy[0], self._debug_target_xy[1], 0.06),
                rgb=(0.9, 0.9, 0.2),
            ))

        self.pub_debug_markers.publish(markers)
        self.last_debug_publish_sec = now_sec

    @staticmethod
    def _make_delete_marker(ns: str, marker_id: int, frame_id: str, stamp):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    @staticmethod
    def _make_sphere_marker(frame_id: str, stamp, marker_id: int, x: float, y: float, z: float, rgb, scale: float):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'local_debug'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale)
        marker.scale.y = float(scale)
        marker.scale.z = float(scale)
        marker.color.r = float(rgb[0])
        marker.color.g = float(rgb[1])
        marker.color.b = float(rgb[2])
        marker.color.a = 0.95
        return marker

    @staticmethod
    def _make_line_marker(frame_id: str, stamp, marker_id: int, start, end, rgb):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'local_debug'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = float(rgb[0])
        marker.color.g = float(rgb[1])
        marker.color.b = float(rgb[2])
        marker.color.a = 0.9

        start_pt = PoseStamped().pose.position
        end_pt = PoseStamped().pose.position
        start_pt.x, start_pt.y, start_pt.z = float(start[0]), float(start[1]), float(start[2])
        end_pt.x, end_pt.y, end_pt.z = float(end[0]), float(end[1]), float(end[2])
        marker.points.append(start_pt)
        marker.points.append(end_pt)
        return marker

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, float(value)))

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
    node = LocalFollowInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
