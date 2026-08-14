#!/usr/bin/env python3

import math
import os
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformBroadcaster,
    TransformListener,
)


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)


def pose_inverse(pose: Pose2D) -> Pose2D:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    x = -(c * pose.x + s * pose.y)
    y = -(-s * pose.x + c * pose.y)
    return Pose2D(x=x, y=y, yaw=wrap_angle(-pose.yaw))


def pose_compose(a: Pose2D, b: Pose2D) -> Pose2D:
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    x = a.x + c * b.x - s * b.y
    y = a.y + s * b.x + c * b.y
    return Pose2D(x=x, y=y, yaw=wrap_angle(a.yaw + b.yaw))


def pose_delta(old_pose: Pose2D, new_pose: Pose2D) -> Pose2D:
    return pose_compose(pose_inverse(old_pose), new_pose)


def pose_distance(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def pose_to_dict(pose: Pose2D) -> dict:
    return {'x': float(pose.x), 'y': float(pose.y), 'yaw': float(pose.yaw)}


def dict_to_pose(data: Optional[dict]) -> Optional[Pose2D]:
    if not isinstance(data, dict):
        return None
    return Pose2D(
        x=float(data.get('x', 0.0)),
        y=float(data.get('y', 0.0)),
        yaw=float(data.get('yaw', 0.0)),
    )


class BreadcrumbTeachRepeat(Node):
    def __init__(self):
        super().__init__('breadcrumb_teach_repeat')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan_filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('pose_topic', '/breadcrumb_loc_pose')
        self.declare_parameter('path_topic', '/breadcrumb_path')
        self.declare_parameter('repeat_path_topic', '/breadcrumb_repeat_path')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('route_frame', 'breadcrumb_map')
        self.declare_parameter('route_file', '/tmp/my_robot_nav_breadcrumb_route.yaml')
        self.declare_parameter('autoload_route', True)
        self.declare_parameter('autosave_route', True)
        self.declare_parameter('publish_start_pose_on_finish', True)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('heading_mode', 'path_tangent')
        self.declare_parameter('allow_reverse', False)

        self.declare_parameter('record_distance', 0.30)
        self.declare_parameter('record_yaw', 0.35)
        self.declare_parameter('max_route_breadcrumbs', 4000)
        self.declare_parameter('grid_resolution', 0.05)

        self.declare_parameter('score_radius', 0.20)
        self.declare_parameter('beam_stride', 4)
        self.declare_parameter('max_beams', 90)
        self.declare_parameter('min_valid_beams', 18)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('coarse_xy_window', 0.16)
        self.declare_parameter('coarse_xy_step', 0.03)
        self.declare_parameter('coarse_yaw_window', 0.20)
        self.declare_parameter('coarse_yaw_step', 0.04)
        self.declare_parameter('fine_xy_window', 0.05)
        self.declare_parameter('fine_xy_step', 0.01)
        self.declare_parameter('fine_yaw_window', 0.08)
        self.declare_parameter('fine_yaw_step', 0.015)
        self.declare_parameter('translation_penalty', 3.0)
        self.declare_parameter('rotation_penalty', 1.2)

        self.declare_parameter('target_reach_distance', 0.22)
        self.declare_parameter('final_goal_tolerance', 0.16)
        self.declare_parameter('lookahead_distance', 0.45)
        self.declare_parameter('slowdown_radius', 0.55)
        self.declare_parameter('rotate_in_place_angle', 0.65)
        self.declare_parameter('max_linear_speed', 0.18)
        self.declare_parameter('max_lateral_speed', 0.12)
        self.declare_parameter('max_angular_speed', 0.70)
        self.declare_parameter('xy_gain', 0.95)
        self.declare_parameter('lateral_gain', 1.35)
        self.declare_parameter('yaw_gain', 1.8)
        # Scan-match-noise rejection for the strafe channel. On a mecanum base,
        # lateral_gain × max_lateral_speed amplifies per-tick xy jitter
        # (observed ~5 cm in /breadcrumb_loc_pose) into full-speed vy sign
        # flips, producing visible side-to-side zig-zag while driving. Reject
        # |error_y| below this threshold. Set to 0.0 to restore the old
        # noise-sensitive behavior.
        self.declare_parameter('lateral_deadband', 0.10)

        # Yaw low-pass for control. On-hardware diagnosis shows scan-match
        # produces per-tick yaw jumps of 0.10-0.20 rad during the initial
        # π-rotation at return start, which (combined with heading_target
        # also near ±π) flips heading_error sign tick-to-tick, flipping wz
        # between ±max_angular_speed — the dominant cause of the observed
        # RETURN snake. Low-pass yaw at alpha*raw + (1-alpha)*prev to reject
        # these jumps while still tracking real rotation. 1.0 = no filter.
        self.declare_parameter('yaw_filter_alpha', 0.35)
        # Hysteretic rotate-in-place band. Enter rotation-scaled mode when
        # |he| exceeds the larger threshold; only exit after |he| stays
        # below the smaller threshold for dwell_ticks consecutive ticks.
        # Prevents mode flip-flop at the boundary when yaw is noisy.
        self.declare_parameter('rotate_enter_angle', 0.85)
        self.declare_parameter('rotate_exit_angle', 0.55)
        self.declare_parameter('rotate_exit_dwell_ticks', 3)
        # During rotate-in-place, latch wz sign at entry. Block sign
        # reversal until |he_filt| shrinks below this release angle — this
        # stops "halfway through π-turn, a scan-match jump flips wz,
        # robot bounces back" bang-bang. If yaw_gain*he_filt would have
        # opposite sign to the latch, wz is clamped to 0 for that tick.
        self.declare_parameter('wz_sign_latch_release_angle', 0.35)

        # --- Rotation shim (Layer 2, Nav2-style pre-tracking rotate stage) ---
        # When a RETURN begins with the robot pointing away from home
        # (|heading_error| near ±π), pure-pursuit tied to scan-match yaw
        # bang-bangs because scan-match yaw flickers across the ±π
        # discontinuity. Instead we open-loop rotate under ODOMETRY yaw
        # feedback until aligned, THEN hand off to pure-pursuit. Odom yaw
        # is continuously integrated and never wraps at ±π at the control
        # layer, so this structurally removes the singularity.
        #
        # On entry we lock (a) the odom yaw at that instant and (b) the
        # required signed rotation derived from ONE scan-match sample of
        # pose + target. During shim, wz = gain * wrap(required_delta -
        # odom_yaw_delta); vx, vy forced to 0. Exit when |remaining| <
        # exit_angle for dwell_ticks consecutive ticks, after which normal
        # pure-pursuit resumes (now with tiny heading error — no ±π risk).
        self.declare_parameter('rotation_shim_enabled', True)
        self.declare_parameter('rotation_shim_entry_angle', 0.85)
        self.declare_parameter('rotation_shim_exit_angle', 0.30)
        self.declare_parameter('rotation_shim_exit_dwell_ticks', 3)
        self.declare_parameter('rotation_shim_wz_gain', 1.2)

        # Pose source for the repeat controller:
        #   'odom'       — dead-reckoning. current_route_pose is recomputed
        #                  every control tick as anchor_route ⊕
        #                  pose_delta(anchor_odom, current_odom). Scan-match
        #                  is skipped during repeat. Recommended for mecanum
        #                  indoor routes < ~20 m: eliminates scan-match yaw
        #                  jumps across ±π that were feeding snake behavior.
        #   'scan_match' — legacy. update_repeat_localization writes
        #                  current_route_pose every scan, with odom delta
        #                  only interpolating between scans.
        self.declare_parameter('replay_pose_source', 'odom')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.repeat_path_topic = self.get_parameter('repeat_path_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.route_frame = self.get_parameter('route_frame').value
        self.route_file = self.get_parameter('route_file').value
        self.autoload_route = bool(self.get_parameter('autoload_route').value)
        self.autosave_route = bool(self.get_parameter('autosave_route').value)
        self.publish_start_pose_on_finish = bool(
            self.get_parameter('publish_start_pose_on_finish').value
        )
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.heading_mode = str(self.get_parameter('heading_mode').value).strip().lower()
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)

        self.record_distance = float(self.get_parameter('record_distance').value)
        self.record_yaw = float(self.get_parameter('record_yaw').value)
        self.max_route_breadcrumbs = int(self.get_parameter('max_route_breadcrumbs').value)
        self.grid_resolution = float(self.get_parameter('grid_resolution').value)

        self.beam_stride = max(1, int(self.get_parameter('beam_stride').value))
        self.max_beams = max(10, int(self.get_parameter('max_beams').value))
        self.min_valid_beams = int(self.get_parameter('min_valid_beams').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.coarse_xy_window = float(self.get_parameter('coarse_xy_window').value)
        self.coarse_xy_step = float(self.get_parameter('coarse_xy_step').value)
        self.coarse_yaw_window = float(self.get_parameter('coarse_yaw_window').value)
        self.coarse_yaw_step = float(self.get_parameter('coarse_yaw_step').value)
        self.fine_xy_window = float(self.get_parameter('fine_xy_window').value)
        self.fine_xy_step = float(self.get_parameter('fine_xy_step').value)
        self.fine_yaw_window = float(self.get_parameter('fine_yaw_window').value)
        self.fine_yaw_step = float(self.get_parameter('fine_yaw_step').value)
        self.translation_penalty = float(self.get_parameter('translation_penalty').value)
        self.rotation_penalty = float(self.get_parameter('rotation_penalty').value)

        self.target_reach_distance = float(self.get_parameter('target_reach_distance').value)
        self.final_goal_tolerance = float(self.get_parameter('final_goal_tolerance').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.slowdown_radius = float(self.get_parameter('slowdown_radius').value)
        self.rotate_in_place_angle = float(self.get_parameter('rotate_in_place_angle').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.xy_gain = float(self.get_parameter('xy_gain').value)
        self.lateral_gain = float(self.get_parameter('lateral_gain').value)
        self.yaw_gain = float(self.get_parameter('yaw_gain').value)
        self.lateral_deadband = max(0.0, float(self.get_parameter('lateral_deadband').value))
        self.yaw_filter_alpha = clamp(
            float(self.get_parameter('yaw_filter_alpha').value), 1e-3, 1.0)
        self.rotate_enter_angle = float(self.get_parameter('rotate_enter_angle').value)
        self.rotate_exit_angle = float(self.get_parameter('rotate_exit_angle').value)
        # Guard against mis-configuration: exit must be strictly below enter,
        # else hysteresis degenerates into the old bang-bang threshold.
        if self.rotate_exit_angle >= self.rotate_enter_angle:
            self.rotate_exit_angle = max(0.0, self.rotate_enter_angle - 0.10)
        self.rotate_exit_dwell_ticks = max(
            1, int(self.get_parameter('rotate_exit_dwell_ticks').value))
        self.wz_sign_latch_release_angle = max(
            0.0, float(self.get_parameter('wz_sign_latch_release_angle').value))

        self.rotation_shim_enabled = bool(
            self.get_parameter('rotation_shim_enabled').value)
        self.rotation_shim_entry_angle = max(
            0.0, float(self.get_parameter('rotation_shim_entry_angle').value))
        self.rotation_shim_exit_angle = max(
            0.0, float(self.get_parameter('rotation_shim_exit_angle').value))
        # Guard against mis-configuration: exit must be strictly below entry.
        if self.rotation_shim_exit_angle >= self.rotation_shim_entry_angle:
            self.rotation_shim_exit_angle = max(
                0.0, self.rotation_shim_entry_angle - 0.2)
        self.rotation_shim_exit_dwell_ticks = max(
            1, int(self.get_parameter('rotation_shim_exit_dwell_ticks').value))
        self.rotation_shim_wz_gain = max(
            1e-3, float(self.get_parameter('rotation_shim_wz_gain').value))

        raw_source = str(
            self.get_parameter('replay_pose_source').value).strip().lower()
        if raw_source not in ('odom', 'scan_match'):
            self.get_logger().warn(
                f"replay_pose_source='{raw_source}' unrecognized; "
                f"falling back to 'odom'.")
            raw_source = 'odom'
        self.replay_pose_source = raw_source

        self._yaw_filt: Optional[float] = None
        self._rotate_mode: bool = False
        self._rotate_exit_dwell: int = 0
        self._wz_latched_sign: int = 0
        self._shim_active: bool = False
        self._shim_odom_yaw_start: Optional[float] = None
        self._shim_required_delta: float = 0.0
        self._shim_exit_dwell: int = 0

        # Dead-reckoning anchors captured at repeat start. While the repeat
        # is active, current_route_pose = anchor_route ⊕
        # pose_delta(anchor_odom, current_odom).  Reset on stop/clear.
        self._repeat_anchor_route: Optional[Pose2D] = None
        self._repeat_anchor_odom: Optional[Pose2D] = None

        self.weight_map = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None

        self.state = 'idle'
        self.current_odom_pose: Optional[Pose2D] = None
        self.route_start_odom_pose: Optional[Pose2D] = None
        self.last_repeat_odom_pose: Optional[Pose2D] = None
        self.current_route_pose: Optional[Pose2D] = None
        self.route_start_map_pose: Optional[Pose2D] = None
        self.latest_scan_msg: Optional[LaserScan] = None
        self.repeat_target_index = 0

        self.breadcrumbs: List[Pose2D] = []
        self.route_points: List[Tuple[float, float]] = []
        self.route_cumulative: List[float] = []
        self._scan_frame_cache = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.repeat_path_pub = self.create_publisher(Path, self.repeat_path_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            10,
        )

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.create_service(Trigger, '~/start_teach', self.handle_start_teach)
        self.create_service(Trigger, '~/finish_teach', self.handle_finish_teach)
        self.create_service(Trigger, '~/start_repeat', self.handle_start_repeat)
        self.create_service(Trigger, '~/stop', self.handle_stop)
        self.create_service(Trigger, '~/save_route', self.handle_save_route)
        self.create_service(Trigger, '~/load_route', self.handle_load_route)

        self.create_timer(0.05, self.control_timer_callback)
        if self.publish_tf:
            self.create_timer(0.05, self.republish_transform)

        if self.autoload_route:
            self.load_route_from_disk(log_missing=False)

        self.get_logger().info(
            'Breadcrumb teach&repeat ready. '
            f'scan_topic={self.scan_topic}, odom_topic={self.odom_topic}, cmd_vel_topic={self.cmd_vel_topic}, '
            f'heading_mode={self.heading_mode}, route_file={self.route_file}'
        )

    def odom_callback(self, msg: Odometry):
        yaw = quat_to_yaw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.current_odom_pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw,
        )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan_msg = msg
        if self.state == 'teach':
            self.record_breadcrumb(msg)
        elif self.state == 'repeat':
            self.update_repeat_localization(msg)

    def handle_start_teach(self, _request, response):
        if self.current_odom_pose is None:
            response.success = False
            response.message = 'No odometry available yet.'
            return response

        self.clear_route()
        self.route_start_odom_pose = self.current_odom_pose
        self.route_start_map_pose = self.lookup_map_pose()
        self.current_route_pose = Pose2D(0.0, 0.0, 0.0)
        self.last_repeat_odom_pose = None
        self.repeat_target_index = 0
        self.state = 'teach'

        recorded = False
        if self.latest_scan_msg is not None:
            recorded = self.record_breadcrumb(self.latest_scan_msg, force=True)

        map_pose_text = 'captured start pose in map frame' if self.route_start_map_pose else 'no map pose captured'
        response.success = True
        response.message = (
            'Teach started. '
            f'{map_pose_text}. '
            f'Initial breadcrumb recorded={recorded}.'
        )
        self.get_logger().info(response.message)
        return response

    def handle_finish_teach(self, _request, response):
        success, message = self.finalize_teach()
        response.success = success
        response.message = message
        return response

    def handle_start_repeat(self, _request, response):
        if self.state == 'teach':
            success, message = self.finalize_teach()
            if not success:
                response.success = False
                response.message = message
                return response

        if len(self.breadcrumbs) < 2 or self.weight_map is None:
            response.success = False
            response.message = 'No breadcrumb route is ready. Teach or load a route first.'
            return response

        if self.current_odom_pose is None:
            response.success = False
            response.message = 'No odometry available yet.'
            return response

        self.state = 'repeat'
        self.current_route_pose = self.breadcrumbs[-1]
        self.last_repeat_odom_pose = self.current_odom_pose
        self._repeat_anchor_route = self.breadcrumbs[-1]
        self._repeat_anchor_odom = self.current_odom_pose
        self.repeat_target_index = max(0, len(self.breadcrumbs) - 2)
        self._yaw_filt = None
        self._rotate_mode = False
        self._rotate_exit_dwell = 0
        self._wz_latched_sign = 0
        self._shim_active = False
        self._shim_odom_yaw_start = None
        self._shim_required_delta = 0.0
        self._shim_exit_dwell = 0

        if (self.latest_scan_msg is not None
                and self.replay_pose_source == 'scan_match'):
            self.update_repeat_localization(self.latest_scan_msg)

        # Arm the rotation shim if the taught path requires a large initial
        # turn. Using the just-refreshed scan-match pose + the first
        # lookahead target + current odom yaw, lock the required signed
        # rotation once so the shim is immune to subsequent scan-match
        # noise. If scan-match gave no usable pose or the initial turn is
        # small, skip the shim and let normal control take over.
        self.arm_rotation_shim()

        self.publish_repeat_path()
        response.success = True
        response.message = (
            f'Repeat started with {len(self.breadcrumbs)} breadcrumbs. '
            f'Target index={self.repeat_target_index}. '
            f'source={self.replay_pose_source} '
            f'shim_active={self._shim_active} '
            f'required_delta={self._shim_required_delta:+.2f}'
        )
        self.get_logger().info(response.message)
        return response

    def arm_rotation_shim(self):
        """Decide at RETURN start whether to enter the odom-only rotation
        shim. Safe to call when preconditions are missing — it just leaves
        the shim disarmed."""
        self._shim_active = False
        self._shim_odom_yaw_start = None
        self._shim_required_delta = 0.0
        self._shim_exit_dwell = 0

        if not self.rotation_shim_enabled:
            return
        if self.current_odom_pose is None or self.current_route_pose is None:
            return
        if not self.breadcrumbs:
            return

        target_pose = self.get_lookahead_target()
        dx = target_pose.x - self.current_route_pose.x
        dy = target_pose.y - self.current_route_pose.y
        distance = math.hypot(dx, dy)
        heading_target = self.compute_heading_target(
            target_pose, dx, dy, distance)
        required_delta = wrap_angle(
            heading_target - self.current_route_pose.yaw)

        if abs(required_delta) <= self.rotation_shim_entry_angle:
            return

        self._shim_active = True
        self._shim_odom_yaw_start = self.current_odom_pose.yaw
        self._shim_required_delta = required_delta
        self._shim_exit_dwell = 0

    def handle_stop(self, _request, response):
        if self.state == 'teach':
            self.state = 'idle'
        elif self.state == 'repeat':
            self.stop_robot()
            self.state = 'ready' if self.breadcrumbs else 'idle'
        else:
            self.state = 'ready' if self.breadcrumbs else 'idle'

        self._shim_active = False
        self._shim_odom_yaw_start = None
        self._shim_required_delta = 0.0
        self._shim_exit_dwell = 0
        self._repeat_anchor_route = None
        self._repeat_anchor_odom = None

        response.success = True
        response.message = f'Breadcrumb node stopped. New state={self.state}.'
        self.get_logger().info(response.message)
        return response

    def handle_save_route(self, _request, response):
        success, message = self.save_route_to_disk()
        response.success = success
        response.message = message
        return response

    def handle_load_route(self, _request, response):
        success, message = self.load_route_from_disk(log_missing=True)
        response.success = success
        response.message = message
        return response

    def finalize_teach(self) -> Tuple[bool, str]:
        if self.state != 'teach':
            return False, f'Cannot finish teach while state={self.state}.'

        if len(self.breadcrumbs) < 2:
            return False, 'Teach route is too short. Need at least 2 breadcrumbs.'

        if not self.rebuild_route_map():
            return False, 'Failed to build breadcrumb laser map from recorded scans.'

        self.state = 'ready'
        self.publish_taught_path()
        self.publish_repeat_path()

        if self.autosave_route:
            saved, save_message = self.save_route_to_disk()
            if not saved:
                self.get_logger().warn(save_message)

        message = (
            f'Teach finished with {len(self.breadcrumbs)} breadcrumbs and '
            f'{len(self.route_points)} laser points.'
        )
        self.get_logger().info(message)
        return True, message

    def clear_route(self):
        self.state = 'idle'
        self.weight_map = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None
        self.route_start_odom_pose = None
        self.last_repeat_odom_pose = None
        self.current_route_pose = None
        self.route_start_map_pose = None
        self.repeat_target_index = 0
        self.breadcrumbs = []
        self.route_points = []
        self.route_cumulative = []
        self._shim_active = False
        self._shim_odom_yaw_start = None
        self._shim_required_delta = 0.0
        self._shim_exit_dwell = 0
        self._repeat_anchor_route = None
        self._repeat_anchor_odom = None
        self.publish_empty_path(self.path_pub)
        self.publish_empty_path(self.repeat_path_pub)

    def record_breadcrumb(self, msg: LaserScan, force: bool = False) -> bool:
        if self.route_start_odom_pose is None or self.current_odom_pose is None:
            return False

        if len(self.breadcrumbs) >= self.max_route_breadcrumbs:
            self.get_logger().warn(
                f'Max breadcrumb count reached ({self.max_route_breadcrumbs}), ignoring extra scans.',
                throttle_duration_sec=2.0,
            )
            return False

        route_pose = pose_delta(self.route_start_odom_pose, self.current_odom_pose)
        if not force and self.breadcrumbs:
            last_pose = self.breadcrumbs[-1]
            travel = pose_distance(route_pose, last_pose)
            yaw_change = abs(wrap_angle(route_pose.yaw - last_pose.yaw))
            if travel < self.record_distance and yaw_change < self.record_yaw:
                return False

        beams = self.scan_to_points(msg)
        if beams is None:
            return False

        world_points = self.transform_points(beams, route_pose)
        self.breadcrumbs.append(route_pose)
        self.route_points.extend((float(x), float(y)) for x, y in world_points.tolist())
        self.recompute_cumulative_distances()
        self.publish_taught_path()

        self.get_logger().info(
            f'Recorded breadcrumb #{len(self.breadcrumbs)} '
            f'at x={route_pose.x:.2f}, y={route_pose.y:.2f}, yaw={route_pose.yaw:.2f}',
            throttle_duration_sec=0.5,
        )
        return True

    def update_repeat_localization(self, msg: LaserScan):
        # In dead-reckoning mode, control never reads scan-match output —
        # avoid the cost of search_pose entirely and do not overwrite
        # current_route_pose. refresh_route_pose_from_odom() is the only
        # writer in odom mode.
        if self.replay_pose_source == 'odom':
            return
        if self.weight_map is None or self.current_odom_pose is None or not self.breadcrumbs:
            return

        beams = self.scan_to_points(msg)
        if beams is None:
            return

        predicted_pose = self.predict_repeat_pose()
        coarse_pose, _ = self.search_pose(
            predicted_pose,
            beams,
            self.coarse_xy_window,
            self.coarse_xy_step,
            self.coarse_yaw_window,
            self.coarse_yaw_step,
        )
        fine_pose, fine_score = self.search_pose(
            coarse_pose,
            beams,
            self.fine_xy_window,
            self.fine_xy_step,
            self.fine_yaw_window,
            self.fine_yaw_step,
        )

        if fine_score <= -1e8:
            self.get_logger().warn(
                'Breadcrumb laser localization could not find a valid pose candidate.',
                throttle_duration_sec=2.0,
            )
            return

        self.current_route_pose = fine_pose
        self.last_repeat_odom_pose = self.current_odom_pose
        self.publish_pose(msg.header.stamp, fine_score)
        if self.publish_tf:
            self.publish_transform(msg.header.stamp)

    def predict_repeat_pose(self) -> Pose2D:
        if self.current_route_pose is None:
            self.current_route_pose = self.breadcrumbs[-1]
            self.last_repeat_odom_pose = self.current_odom_pose
            return self.current_route_pose

        if self.last_repeat_odom_pose is None:
            self.last_repeat_odom_pose = self.current_odom_pose
            return self.current_route_pose

        delta = pose_delta(self.last_repeat_odom_pose, self.current_odom_pose)
        return pose_compose(self.current_route_pose, delta)

    def refresh_route_pose_from_odom(self) -> None:
        """Dead-reckoning pose update. Recomputes current_route_pose as
        anchor_route ⊕ pose_delta(anchor_odom, current_odom). No-op if
        anchors or odom are missing."""
        if (self._repeat_anchor_route is None
                or self._repeat_anchor_odom is None
                or self.current_odom_pose is None):
            return
        delta = pose_delta(self._repeat_anchor_odom, self.current_odom_pose)
        self.current_route_pose = pose_compose(
            self._repeat_anchor_route, delta)

    def control_timer_callback(self):
        if self.state != 'repeat' or self.current_route_pose is None or not self.breadcrumbs:
            return

        if self.replay_pose_source == 'odom':
            self.refresh_route_pose_from_odom()
            # In scan-match mode, publish_pose runs from
            # update_repeat_localization. In odom mode that path is
            # disabled, so we publish here — session_manager needs
            # /breadcrumb_loc_pose to detect arrival.
            self.publish_pose(self.get_clock().now().to_msg(), 100.0)

        while (
            self.repeat_target_index > 0
            and pose_distance(self.current_route_pose, self.breadcrumbs[self.repeat_target_index])
            <= self.target_reach_distance
        ):
            self.repeat_target_index -= 1
            self.get_logger().info(
                f'Reached breadcrumb, next target index={self.repeat_target_index}'
            )
            self.publish_repeat_path()

        final_pose = self.breadcrumbs[0]
        final_distance = pose_distance(self.current_route_pose, final_pose)
        final_yaw_error = abs(wrap_angle(final_pose.yaw - self.current_route_pose.yaw))
        if final_distance <= self.final_goal_tolerance and final_yaw_error <= 0.35:
            self.finish_repeat()
            return

        target_pose = self.get_lookahead_target()
        cmd = self.compute_cmd(target_pose)
        self.cmd_pub.publish(cmd)

    def get_lookahead_target(self) -> Pose2D:
        if self.repeat_target_index <= 0:
            return self.breadcrumbs[0]

        lookahead_index = self.repeat_target_index
        current_arc = self.route_cumulative[self.repeat_target_index]
        while lookahead_index > 0:
            candidate = lookahead_index - 1
            if current_arc - self.route_cumulative[candidate] >= self.lookahead_distance:
                lookahead_index = candidate
                break
            lookahead_index = candidate
        return self.breadcrumbs[lookahead_index]

    def compute_cmd(self, target_pose: Pose2D) -> Twist:
        # Rotation-shim phase: open-loop rotate under ODOM yaw feedback.
        # Scan-match yaw is ignored — during the initial π-rotation it
        # emits per-tick jumps that cross ±π, which is exactly what
        # produces the RETURN snake. Odom yaw is continuously integrated
        # (never wraps at ±π at the control layer), so tracking a locked
        # signed required_delta against it is singularity-free. vx/vy
        # are held at 0 — no translation during the initial turn.
        if self._shim_active:
            if (self.current_odom_pose is None
                    or self._shim_odom_yaw_start is None):
                self._shim_active = False
            else:
                achieved = wrap_angle(
                    self.current_odom_pose.yaw - self._shim_odom_yaw_start)
                remaining = wrap_angle(self._shim_required_delta - achieved)
                if abs(remaining) < self.rotation_shim_exit_angle:
                    self._shim_exit_dwell += 1
                    if self._shim_exit_dwell >= self.rotation_shim_exit_dwell_ticks:
                        self._shim_active = False
                        self._shim_exit_dwell = 0
                else:
                    self._shim_exit_dwell = 0
                if self._shim_active:
                    wz_cmd = clamp(
                        self.rotation_shim_wz_gain * remaining,
                        -self.max_angular_speed,
                        self.max_angular_speed,
                    )
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0
                    cmd.angular.z = wz_cmd
                    return cmd
                # Shim just finished: reset the Layer-1 yaw filter so the
                # next tick re-initializes on the now-stable scan-match
                # yaw rather than carrying a stale value from pre-rotate.
                self._yaw_filt = None
                self._rotate_mode = False
                self._rotate_exit_dwell = 0
                self._wz_latched_sign = 0

        dx = target_pose.x - self.current_route_pose.x
        dy = target_pose.y - self.current_route_pose.y
        distance = math.hypot(dx, dy)

        # Low-pass the scan-match yaw for control. Delta-through-wrap so the
        # filter tracks ±π correctly; without unwrap, a jump from +3.1 to
        # -3.1 (essentially no rotation) would drag the filter across zero.
        raw_yaw = self.current_route_pose.yaw
        if self._yaw_filt is None:
            self._yaw_filt = raw_yaw
        else:
            delta = wrap_angle(raw_yaw - self._yaw_filt)
            self._yaw_filt = wrap_angle(
                self._yaw_filt + self.yaw_filter_alpha * delta)
        yaw_ctrl = self._yaw_filt

        cos_yaw = math.cos(yaw_ctrl)
        sin_yaw = math.sin(yaw_ctrl)
        error_x = cos_yaw * dx + sin_yaw * dy
        error_y = -sin_yaw * dx + cos_yaw * dy

        heading_target = self.compute_heading_target(target_pose, dx, dy, distance)
        heading_error = wrap_angle(heading_target - yaw_ctrl)
        abs_he = abs(heading_error)

        # Hysteretic rotate-in-place mode. Enter when |he| > enter_angle.
        # Exit only after |he| < exit_angle for dwell_ticks consecutive
        # ticks (prevents flip-flop at boundary when yaw is noisy). On
        # entry, latch wz sign to whichever direction we'd rotate right
        # now — blocks mid-turn sign reversal from scan-match jumps.
        if self._rotate_mode:
            if abs_he < self.rotate_exit_angle:
                self._rotate_exit_dwell += 1
                if self._rotate_exit_dwell >= self.rotate_exit_dwell_ticks:
                    self._rotate_mode = False
                    self._rotate_exit_dwell = 0
            else:
                self._rotate_exit_dwell = 0
        else:
            if abs_he > self.rotate_enter_angle:
                self._rotate_mode = True
                self._rotate_exit_dwell = 0
                self._wz_latched_sign = 1 if heading_error > 0.0 else -1

        scale = 1.0
        if self.slowdown_radius > 1e-6:
            scale = clamp(distance / self.slowdown_radius, 0.20, 1.0)
        if self._rotate_mode:
            scale *= 0.25

        vx = clamp(self.xy_gain * error_x, -self.max_linear_speed, self.max_linear_speed)
        # Reject scan-match lateral jitter before it hits the strafe channel.
        # Without this, |error_y| of a few cm (pure measurement noise) was
        # amplified into saturated ±max_lateral_speed sign-flipping at 30 Hz.
        ey = error_y
        if self.lateral_deadband > 0.0:
            if abs(ey) <= self.lateral_deadband:
                ey = 0.0
            elif ey > 0.0:
                ey -= self.lateral_deadband
            else:
                ey += self.lateral_deadband
        vy = clamp(self.lateral_gain * ey, -self.max_lateral_speed, self.max_lateral_speed)
        if not self.allow_reverse:
            vx = max(0.0, vx)
        vx *= scale
        vy *= scale

        wz_raw = clamp(
            self.yaw_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        # wz sign-latch: while in rotate-in-place mode, do not allow sign
        # reversal until the heading has substantially closed. If the
        # proportional law would flip sign because scan-match yaw jumped,
        # hold wz at 0 for this tick — the filter will recover next tick.
        if self._wz_latched_sign != 0:
            if abs_he < self.wz_sign_latch_release_angle:
                self._wz_latched_sign = 0
                wz = wz_raw
            elif wz_raw * self._wz_latched_sign < 0.0:
                wz = 0.0
            else:
                wz = wz_raw
        else:
            wz = wz_raw

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        return cmd

    def compute_heading_target(
        self,
        target_pose: Pose2D,
        dx: float,
        dy: float,
        distance: float,
    ) -> float:
        if self.heading_mode == 'teach':
            return target_pose.yaw
        if distance < 0.05:
            return target_pose.yaw
        return math.atan2(dy, dx)

    def finish_repeat(self):
        self.stop_robot()
        self.state = 'ready'
        self.repeat_target_index = 0
        self._yaw_filt = None
        self._rotate_mode = False
        self._rotate_exit_dwell = 0
        self._wz_latched_sign = 0
        self._shim_active = False
        self._shim_odom_yaw_start = None
        self._shim_required_delta = 0.0
        self._shim_exit_dwell = 0
        self._repeat_anchor_route = None
        self._repeat_anchor_odom = None
        self.publish_repeat_path()
        self.get_logger().info('Breadcrumb repeat finished, robot is back at the taught start.')
        if self.publish_start_pose_on_finish:
            self.publish_start_pose_seed()

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def scan_to_points(self, msg: LaserScan):
        laser_offset = self.lookup_laser_offset(msg.header.frame_id)
        if laser_offset is None:
            return None

        offset_x, offset_y, offset_yaw = laser_offset
        cos_offset = math.cos(offset_yaw)
        sin_offset = math.sin(offset_yaw)
        range_cap = min(self.max_range, float(msg.range_max))
        valid_points = []

        for index in range(0, len(msg.ranges), self.beam_stride):
            distance = float(msg.ranges[index])
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > range_cap:
                continue

            angle = msg.angle_min + index * msg.angle_increment
            laser_x = distance * math.cos(angle)
            laser_y = distance * math.sin(angle)
            base_x = offset_x + cos_offset * laser_x - sin_offset * laser_y
            base_y = offset_y + sin_offset * laser_x + cos_offset * laser_y
            valid_points.append((base_x, base_y))
            if len(valid_points) >= self.max_beams:
                break

        if len(valid_points) < self.min_valid_beams:
            return None

        return np.asarray(valid_points, dtype=np.float32)

    def lookup_laser_offset(self, scan_frame: str):
        if scan_frame in self._scan_frame_cache:
            return self._scan_frame_cache[scan_frame]

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                scan_frame,
                Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        yaw = quat_to_yaw(
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        offset = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            yaw,
        )
        self._scan_frame_cache[scan_frame] = offset
        self.get_logger().info(
            f'Using scan frame offset {scan_frame} -> {self.base_frame}: '
            f'x={offset[0]:.3f}, y={offset[1]:.3f}, yaw={offset[2]:.3f}'
        )
        return offset

    @staticmethod
    def transform_points(points: np.ndarray, pose: Pose2D) -> np.ndarray:
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        out = np.empty_like(points)
        out[:, 0] = pose.x + cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
        out[:, 1] = pose.y + sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
        return out

    def rebuild_route_map(self) -> bool:
        if not self.route_points:
            return False

        points = np.asarray(self.route_points, dtype=np.float32)
        padding = max(0.60, 2.0 * float(self.get_parameter('score_radius').value))
        min_x = float(points[:, 0].min()) - padding
        max_x = float(points[:, 0].max()) + padding
        min_y = float(points[:, 1].min()) - padding
        max_y = float(points[:, 1].max()) + padding

        width = int(math.ceil((max_x - min_x) / self.grid_resolution)) + 1
        height = int(math.ceil((max_y - min_y) / self.grid_resolution)) + 1
        if width <= 0 or height <= 0:
            return False

        occupied = np.zeros((height, width), dtype=bool)
        map_x = ((points[:, 0] - min_x) / self.grid_resolution).astype(np.int32)
        map_y = ((points[:, 1] - min_y) / self.grid_resolution).astype(np.int32)
        inside = (
            (map_x >= 0) & (map_x < width) &
            (map_y >= 0) & (map_y < height)
        )
        occupied[map_y[inside], map_x[inside]] = True

        self.map_origin_x = min_x
        self.map_origin_y = min_y
        self.map_resolution = self.grid_resolution
        self.map_width = width
        self.map_height = height
        self.weight_map = self.build_weight_map(occupied, self.grid_resolution)
        self.get_logger().info(
            f'Built breadcrumb laser map: {width}x{height} @ {self.grid_resolution:.3f} m'
        )
        return True

    def build_weight_map(self, occupied: np.ndarray, resolution: float) -> np.ndarray:
        radius_m = float(self.get_parameter('score_radius').value)
        radius_cells = max(1, int(math.ceil(radius_m / resolution)))
        kernel = np.zeros((radius_cells * 2 + 1, radius_cells * 2 + 1), dtype=np.float32)

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                distance = math.hypot(dx, dy)
                if distance > radius_cells:
                    continue
                kernel[dy + radius_cells, dx + radius_cells] = (
                    1.0 - distance / max(1.0, radius_cells)
                )

        weight_map = np.zeros_like(occupied, dtype=np.float32)
        ys, xs = np.nonzero(occupied)
        for y, x in zip(ys, xs):
            y_min = max(0, y - radius_cells)
            y_max = min(weight_map.shape[0], y + radius_cells + 1)
            x_min = max(0, x - radius_cells)
            x_max = min(weight_map.shape[1], x + radius_cells + 1)

            kernel_y_min = y_min - (y - radius_cells)
            kernel_y_max = kernel_y_min + (y_max - y_min)
            kernel_x_min = x_min - (x - radius_cells)
            kernel_x_max = kernel_x_min + (x_max - x_min)

            region = weight_map[y_min:y_max, x_min:x_max]
            region_kernel = kernel[kernel_y_min:kernel_y_max, kernel_x_min:kernel_x_max]
            np.maximum(region, region_kernel, out=region)

        if not np.any(weight_map):
            weight_map = occupied.astype(np.float32)
        return weight_map

    def search_pose(
        self,
        seed_pose: Pose2D,
        beams: np.ndarray,
        xy_window: float,
        xy_step: float,
        yaw_window: float,
        yaw_step: float,
    ):
        best_pose = seed_pose
        best_score = -1e9
        xy_offsets = np.arange(-xy_window, xy_window + 0.5 * xy_step, xy_step)
        yaw_offsets = np.arange(-yaw_window, yaw_window + 0.5 * yaw_step, yaw_step)

        for dyaw in yaw_offsets:
            yaw = wrap_angle(seed_pose.yaw + dyaw)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            rot_x = cos_yaw * beams[:, 0] - sin_yaw * beams[:, 1]
            rot_y = sin_yaw * beams[:, 0] + cos_yaw * beams[:, 1]

            for dx in xy_offsets:
                world_x = seed_pose.x + dx + rot_x
                for dy in xy_offsets:
                    world_y = seed_pose.y + dy + rot_y
                    score = self.score_points(world_x, world_y)
                    score -= self.translation_penalty * (dx * dx + dy * dy)
                    score -= self.rotation_penalty * (dyaw * dyaw)
                    if score > best_score:
                        best_score = score
                        best_pose = Pose2D(
                            x=seed_pose.x + dx,
                            y=seed_pose.y + dy,
                            yaw=yaw,
                        )

        return best_pose, best_score

    def score_points(self, world_x: np.ndarray, world_y: np.ndarray) -> float:
        map_x = ((world_x - self.map_origin_x) / self.map_resolution).astype(np.int32)
        map_y = ((world_y - self.map_origin_y) / self.map_resolution).astype(np.int32)
        inside = (
            (map_x >= 0)
            & (map_x < self.map_width)
            & (map_y >= 0)
            & (map_y < self.map_height)
        )
        if not np.any(inside):
            return -1e9

        weights = self.weight_map[map_y[inside], map_x[inside]]
        return float(weights.sum() - 0.2 * (len(world_x) - np.count_nonzero(inside)))

    def publish_transform(self, stamp):
        if self.current_odom_pose is None or self.current_route_pose is None:
            return

        route_to_odom = pose_compose(self.current_route_pose, pose_inverse(self.current_odom_pose))
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.route_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = route_to_odom.x
        transform.transform.translation.y = route_to_odom.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(route_to_odom.yaw / 2.0)
        transform.transform.rotation.w = math.cos(route_to_odom.yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def republish_transform(self):
        if self.state != 'repeat':
            return
        self.publish_transform(self.get_clock().now().to_msg())

    def publish_pose(self, stamp, score: float):
        if self.current_route_pose is None:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.route_frame
        msg.pose.pose.position.x = self.current_route_pose.x
        msg.pose.pose.position.y = self.current_route_pose.y
        msg.pose.pose.orientation.z = math.sin(self.current_route_pose.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.current_route_pose.yaw / 2.0)
        covariance_xy = max(0.02, 0.20 - min(0.18, 0.001 * max(score, 0.0)))
        covariance_yaw = max(0.02, 0.10 - min(0.08, 0.0005 * max(score, 0.0)))
        msg.pose.covariance = [
            covariance_xy, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, covariance_xy, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, covariance_yaw,
        ]
        self.pose_pub.publish(msg)

    def publish_taught_path(self):
        self.publish_path(self.path_pub, self.breadcrumbs)

    def publish_repeat_path(self):
        if not self.breadcrumbs:
            self.publish_empty_path(self.repeat_path_pub)
            return
        if self.state == 'repeat':
            poses = list(reversed(self.breadcrumbs[: self.repeat_target_index + 1]))
        else:
            poses = list(reversed(self.breadcrumbs))
        self.publish_path(self.repeat_path_pub, poses)

    def publish_path(self, publisher, poses: Sequence[Pose2D]):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.route_frame
        for pose in poses:
            stamped = PoseStamped()
            stamped.header = path.header
            stamped.pose.position.x = pose.x
            stamped.pose.position.y = pose.y
            stamped.pose.orientation.z = math.sin(pose.yaw / 2.0)
            stamped.pose.orientation.w = math.cos(pose.yaw / 2.0)
            path.poses.append(stamped)
        publisher.publish(path)

    def publish_empty_path(self, publisher):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.route_frame
        publisher.publish(path)

    def recompute_cumulative_distances(self):
        self.route_cumulative = []
        total = 0.0
        for index, pose in enumerate(self.breadcrumbs):
            if index > 0:
                total += pose_distance(self.breadcrumbs[index - 1], pose)
            self.route_cumulative.append(total)

    def publish_start_pose_seed(self):
        if self.route_start_map_pose is None:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.route_start_map_pose.x
        msg.pose.pose.position.y = self.route_start_map_pose.y
        msg.pose.pose.orientation.z = math.sin(self.route_start_map_pose.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.route_start_map_pose.yaw / 2.0)
        msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
        ]

        for _ in range(3):
            self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            'Republished taught start pose to /initialpose so AMCL can relock in the known map.'
        )

    def lookup_map_pose(self) -> Optional[Pose2D]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        yaw = quat_to_yaw(
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        return Pose2D(
            x=float(transform.transform.translation.x),
            y=float(transform.transform.translation.y),
            yaw=yaw,
        )

    def save_route_to_disk(self) -> Tuple[bool, str]:
        if len(self.breadcrumbs) < 2 or not self.route_points:
            return False, 'No breadcrumb route available to save.'

        payload = {
            'version': 1,
            'route_frame': self.route_frame,
            'base_frame': self.base_frame,
            'grid_resolution': float(self.grid_resolution),
            'heading_mode': self.heading_mode,
            'breadcrumbs': [pose_to_dict(pose) for pose in self.breadcrumbs],
            'map_points': [[float(x), float(y)] for x, y in self.route_points],
            'start_map_pose': pose_to_dict(self.route_start_map_pose)
            if self.route_start_map_pose is not None else None,
        }

        route_dir = os.path.dirname(self.route_file)
        if route_dir:
            os.makedirs(route_dir, exist_ok=True)

        try:
            with open(self.route_file, 'w', encoding='utf-8') as handle:
                yaml.safe_dump(payload, handle, sort_keys=False)
        except OSError as exc:
            return False, f'Failed to save breadcrumb route: {exc}'

        message = f'Saved breadcrumb route to {self.route_file}'
        self.get_logger().info(message)
        return True, message

    def load_route_from_disk(self, log_missing: bool) -> Tuple[bool, str]:
        if not self.route_file:
            return False, 'route_file parameter is empty.'

        if not os.path.exists(self.route_file):
            message = f'Breadcrumb route file does not exist: {self.route_file}'
            if log_missing:
                self.get_logger().warn(message)
            return False, message

        try:
            with open(self.route_file, 'r', encoding='utf-8') as handle:
                payload = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            return False, f'Failed to read breadcrumb route: {exc}'

        breadcrumb_data = payload.get('breadcrumbs', [])
        point_data = payload.get('map_points', [])
        if len(breadcrumb_data) < 2 or not point_data:
            return False, 'Breadcrumb route file is missing breadcrumbs or laser points.'

        self.clear_route()
        self.breadcrumbs = [dict_to_pose(item) for item in breadcrumb_data]
        self.breadcrumbs = [pose for pose in self.breadcrumbs if pose is not None]
        self.route_points = [(float(item[0]), float(item[1])) for item in point_data]
        self.route_start_map_pose = dict_to_pose(payload.get('start_map_pose'))
        self.grid_resolution = float(payload.get('grid_resolution', self.grid_resolution))
        self.heading_mode = str(payload.get('heading_mode', self.heading_mode)).strip().lower()

        self.recompute_cumulative_distances()
        if not self.rebuild_route_map():
            self.clear_route()
            return False, 'Loaded route file, but failed to rebuild its breadcrumb map.'

        self.state = 'ready'
        self.publish_taught_path()
        self.publish_repeat_path()
        message = (
            f'Loaded breadcrumb route from {self.route_file} '
            f'with {len(self.breadcrumbs)} breadcrumbs.'
        )
        self.get_logger().info(message)
        return True, message


def main(args=None):
    rclpy.init(args=args)
    node = BreadcrumbTeachRepeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
