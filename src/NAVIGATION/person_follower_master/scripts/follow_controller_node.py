#!/usr/bin/env python3
"""Follow controller node.

Consumes /master_target + /scan, produces /cmd_vel at 20 Hz with:
  - PD on (range, bearing)
  - In-place rotation when bearing > 45°
  - Dead-zone near target distance
  - Forward-cone lidar safety cutoff
  - Acceleration clamp
  - Watchdog (no /master_target for > 0.5 s → stop)
  - Loss-age behaviour:
      age < 2s      → hold
      2s ≤ age < 10s → scan-rotate, flip sign every 3s
      age ≥ 10s     → stop + alarm log
"""

from __future__ import annotations

import math
import os
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from person_follower_master.msg import MasterTarget


STATE_IDLE = "IDLE"
STATE_ENROLLING = "ENROLLING"
STATE_FOLLOWING = "FOLLOWING"
STATE_LOST = "LOST"


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class FollowControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("follow_controller_node")

        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("d_target", 1.20)
        self.declare_parameter("d_deadzone", 0.15)
        self.declare_parameter("kp_v", 0.80)
        self.declare_parameter("kp_w", 1.80)
        self.declare_parameter("v_max", 0.40)
        self.declare_parameter("w_max", 0.80)
        self.declare_parameter("accel_v_max", 1.50)
        self.declare_parameter("accel_w_max", 3.00)
        self.declare_parameter("bearing_turn_in_place_deg", 45.0)
        self.declare_parameter("bearing_deadzone_deg", 5.0)  # suppress w jitter when master is centered
        self.declare_parameter("obstacle_cone_deg", 30.0)
        self.declare_parameter("obstacle_min_range", 0.30)
        self.declare_parameter("lost_hold_sec", 2.0)
        # Once age exceeds lost_hold_sec the controller enters a
        # face-back-then-scan behaviour that persists indefinitely —
        # we never give up looking for the master.
        self.declare_parameter("lost_face_back_deg", 10.0)
        self.declare_parameter("lost_face_back_max_sec", 2.0)
        # Scan rotation is capped by what YOLO@~10Hz can tolerate without
        # motion blur killing recall. 0.25 rad/s ≈ 14° per frame — detector
        # still sees a person on two consecutive frames even at frame edge.
        self.declare_parameter("scan_omega", 0.25)
        self.declare_parameter("scan_flip_period_sec", 12.0)
        # Every `scan_pause_period` the robot stops for `scan_pause_sec` so
        # the detector gets a few clean (unblurred) frames to latch onto
        # anyone in view. Set pause_sec <= 0 to disable.
        self.declare_parameter("scan_pause_period_sec", 3.0)
        self.declare_parameter("scan_pause_sec", 0.8)
        self.declare_parameter("master_watchdog_sec", 0.5)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("master_topic", "/master_target")

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.d_target = float(self.get_parameter("d_target").value)
        self.d_deadzone = float(self.get_parameter("d_deadzone").value)
        self.kp_v = float(self.get_parameter("kp_v").value)
        self.kp_w = float(self.get_parameter("kp_w").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.accel_v_max = float(self.get_parameter("accel_v_max").value)
        self.accel_w_max = float(self.get_parameter("accel_w_max").value)
        self.bearing_turn_in_place_rad = math.radians(
            float(self.get_parameter("bearing_turn_in_place_deg").value))
        self.bearing_deadzone_rad = math.radians(
            float(self.get_parameter("bearing_deadzone_deg").value))
        self.obstacle_cone_rad = math.radians(
            float(self.get_parameter("obstacle_cone_deg").value))
        self.obstacle_min_range = float(self.get_parameter("obstacle_min_range").value)
        self.lost_hold_sec = float(self.get_parameter("lost_hold_sec").value)
        self.lost_face_back_rad = math.radians(
            float(self.get_parameter("lost_face_back_deg").value))
        self.lost_face_back_max_sec = float(self.get_parameter("lost_face_back_max_sec").value)
        self.scan_omega = float(self.get_parameter("scan_omega").value)
        self.scan_flip_period = float(self.get_parameter("scan_flip_period_sec").value)
        self.scan_pause_period = float(self.get_parameter("scan_pause_period_sec").value)
        self.scan_pause_sec = float(self.get_parameter("scan_pause_sec").value)
        self.master_watchdog_sec = float(self.get_parameter("master_watchdog_sec").value)

        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        master_topic = str(self.get_parameter("master_topic").value)

        # State
        self._last_master: Optional[MasterTarget] = None
        self._last_master_stamp_sec: float = 0.0
        self._last_scan: Optional[LaserScan] = None
        self._scan_phase_start_sec: float = 0.0
        self._scan_phase_sign: float = 1.0
        self._last_cmd_v: float = 0.0
        self._last_cmd_w: float = 0.0
        self._last_tick_sec: Optional[float] = None
        self._last_safety_log_sec: float = 0.0
        self._last_timeout_log_sec: float = 0.0

        # QoS
        qos_master = QoSProfile(depth=1)
        qos_master.reliability = ReliabilityPolicy.RELIABLE
        qos_master.history = HistoryPolicy.KEEP_LAST
        qos_master.durability = DurabilityPolicy.TRANSIENT_LOCAL

        qos_sensor = QoSProfile(depth=5)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_sensor.history = HistoryPolicy.KEEP_LAST

        qos_cmd = QoSProfile(depth=10)
        qos_cmd.reliability = ReliabilityPolicy.RELIABLE
        qos_cmd.history = HistoryPolicy.KEEP_LAST

        self.sub_master = self.create_subscription(
            MasterTarget, master_topic, self._on_master, qos_master)
        self.sub_scan = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_sensor)
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, qos_cmd)

        period = 1.0 / max(1.0, self.control_rate_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"follow_controller ready. cmd_vel={cmd_topic} "
            f"d_target={self.d_target} v_max={self.v_max} w_max={self.w_max}"
        )

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_master(self, msg: MasterTarget) -> None:
        self._last_master = msg
        self._last_master_stamp_sec = self._now_sec()

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _min_forward_range(self) -> float:
        scan = self._last_scan
        if scan is None or not scan.ranges:
            return math.inf
        n = len(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        rmin = math.inf
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            theta = angle_min + i * angle_inc
            # Normalize into [-pi, pi]
            theta = (theta + math.pi) % (2.0 * math.pi) - math.pi
            if abs(theta) <= self.obstacle_cone_rad:
                if r < rmin:
                    rmin = r
        return rmin

    # ------------------------------------------------------------
    # Control tick
    # ------------------------------------------------------------
    def _tick(self) -> None:
        now = self._now_sec()
        dt = 1.0 / self.control_rate_hz
        if self._last_tick_sec is not None:
            dt = max(1e-3, min(0.5, now - self._last_tick_sec))
        self._last_tick_sec = now

        v_cmd = 0.0
        w_cmd = 0.0

        # Watchdog on master stream.
        if (self._last_master is None
                or (now - self._last_master_stamp_sec) > self.master_watchdog_sec):
            if (now - self._last_safety_log_sec) > 1.0:
                self.get_logger().warn(
                    "watchdog: no /master_target within {:.2f}s — stopping".format(
                        self.master_watchdog_sec))
                self._last_safety_log_sec = now
            self._publish_with_clamp(0.0, 0.0, dt)
            return

        mt = self._last_master
        state = mt.state

        if state == STATE_IDLE or state == STATE_ENROLLING:
            v_cmd = 0.0
            w_cmd = 0.0
        elif state == STATE_FOLLOWING and mt.is_locked:
            v_cmd, w_cmd = self._compute_following(mt)
        elif state == STATE_LOST:
            v_cmd, w_cmd = self._compute_lost(mt, now)
        else:
            v_cmd = 0.0
            w_cmd = 0.0

        # Lidar front-cone cutoff (only when we're commanding forward motion).
        if v_cmd > 0.02:
            fwd = self._min_forward_range()
            if fwd < self.obstacle_min_range:
                if (now - self._last_safety_log_sec) > 1.0:
                    self.get_logger().warn(
                        f"safety: forward obstacle at {fwd:.2f}m — zeroing v")
                    self._last_safety_log_sec = now
                v_cmd = 0.0

        self._publish_with_clamp(v_cmd, w_cmd, dt)

    # ------------------------------------------------------------
    # Per-state control laws
    # ------------------------------------------------------------
    def _compute_following(self, mt: MasterTarget) -> tuple:
        d = float(mt.distance)
        bearing = float(mt.bearing)

        # Turn in place when bearing is large.
        if abs(bearing) > self.bearing_turn_in_place_rad:
            return 0.0, _clamp(self.kp_w * bearing, -self.w_max, self.w_max)

        range_err = d - self.d_target
        if abs(range_err) < self.d_deadzone:
            v = 0.0
        else:
            v = _clamp(self.kp_v * range_err, -self.v_max, self.v_max)
        # Bearing deadzone: when the master is essentially centred,
        # suppress ω entirely so small detection jitter doesn't make
        # the robot shimmy left/right while standing still.
        if abs(bearing) < self.bearing_deadzone_rad:
            w = 0.0
        else:
            w = _clamp(self.kp_w * bearing, -self.w_max, self.w_max)
        return v, w

    def _compute_lost(self, mt: MasterTarget, now: float) -> tuple:
        """Three-phase search that never gives up:

           1. HOLD       : do nothing briefly; handles momentary occlusion.
           2. FACE-BACK  : rotate toward the last-known bearing so the
                            camera points back where the master was seen.
           3. SCAN       : rotate continuously, flipping direction every
                            scan_flip_period, indefinitely.
        """
        age = float(mt.time_since_last_seen)
        if age < self.lost_hold_sec:
            return 0.0, 0.0

        face_back_age = age - self.lost_hold_sec
        bearing_to_last = float(mt.bearing)

        # Phase 2: rotate toward the last-known bearing. Exit once the
        # camera is roughly pointed back (|bearing| < face_back threshold)
        # OR we've spent enough time trying (hard cap so we never stall
        # if TF is flaky and bearing never converges).
        if (face_back_age < self.lost_face_back_max_sec
                and abs(bearing_to_last) > self.lost_face_back_rad):
            w = _clamp(self.kp_w * bearing_to_last, -self.w_max, self.w_max)
            return 0.0, w

        # Phase 3: persistent scan-rotate with optional pauses so the
        # detector gets unblurred frames between sweeps.
        if (now - self._scan_phase_start_sec) > self.scan_flip_period:
            self._scan_phase_sign = -self._scan_phase_sign
            self._scan_phase_start_sec = now

        in_pause = False
        if self.scan_pause_sec > 0.0 and self.scan_pause_period > 0.0:
            phase_in_cycle = (now - self._scan_phase_start_sec) % self.scan_pause_period
            if phase_in_cycle < self.scan_pause_sec:
                in_pause = True

        # Periodic status log (not alarming — we're just still searching).
        if (now - self._last_timeout_log_sec) > 5.0:
            self.get_logger().info(
                f"searching for master (lost {age:.1f}s, "
                f"ω={0.0 if in_pause else self._scan_phase_sign * self.scan_omega:+.2f})")
            self._last_timeout_log_sec = now

        if in_pause:
            return 0.0, 0.0
        return 0.0, self._scan_phase_sign * self.scan_omega

    # ------------------------------------------------------------
    # Publish with acceleration clamp
    # ------------------------------------------------------------
    def _publish_with_clamp(self, v: float, w: float, dt: float) -> None:
        dv_max = self.accel_v_max * dt
        dw_max = self.accel_w_max * dt
        v_out = _clamp(v, self._last_cmd_v - dv_max, self._last_cmd_v + dv_max)
        w_out = _clamp(w, self._last_cmd_w - dw_max, self._last_cmd_w + dw_max)
        v_out = _clamp(v_out, -self.v_max, self.v_max)
        w_out = _clamp(w_out, -self.w_max, self.w_max)

        twist = Twist()
        twist.linear.x = float(v_out)
        twist.angular.z = float(w_out)
        self.pub_cmd.publish(twist)

        self._last_cmd_v = v_out
        self._last_cmd_w = w_out


def main() -> None:
    rclpy.init()
    node = FollowControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Best-effort stop on shutdown.
    try:
        twist = Twist()
        node.pub_cmd.publish(twist)
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
