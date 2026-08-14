#!/usr/bin/env python3
"""Safety wrapper for the return-home path follower.

Sits between breadcrumb_teach_repeat's raw /cmd_vel_return_raw output and
the cmd_vel_mux, adding a reactive obstacle-avoidance layer so the robot
never blindly drives the taught path into a person or a new obstacle.

Two-tier policy:

  Tier 1 — stop-and-wait (AGV default):
    * Compute min lidar range within a forward cone (default ±35°).
    * slow_start_range >= near > stop_range   : scale linear velocity linearly
    * near < stop_range                        : zero all linear, keep angular
    * near < emergency_range (any direction)   : zero all velocities

  Tier 2 — EVADE (static-obstacle bypass, omnidirectional only):
    * If STOP persists longer than evade_after_sec and a side is provably
      clearer than the other, strafe (vy) that way + tiny forward creep (vx)
      to slip past the obstacle while the breadcrumb controller continues
      to regulate heading on its own path.
    * Exit EVADE when the forward cone clears with hysteresis
      (evade_clear_margin_m > stop_range).
    * Abort EVADE if the chosen side closes in (< evade_side_abort_m), or
      cumulative lateral distance exceeds evade_max_distance_m, or duration
      exceeds evade_max_duration_sec. ABORT escalates to session_manager.

  Global:
    * blocked continuously > abort_timeout_sec → publish ABORT.

Inputs:
  /cmd_vel_return_raw  (geometry_msgs/Twist)   - from breadcrumb node
  /scan_filtered       (sensor_msgs/LaserScan) - self-filtered lidar
  /session_manager/mode (std_msgs/String)       - gate: only filter during RETURNING

Outputs:
  /cmd_vel_return          (geometry_msgs/Twist) - filtered, to cmd_vel_mux
  /return_safety/blocked_sec (std_msgs/Float32)  - seconds we've been blocked
  /return_safety/status      (std_msgs/String)   - CLEAR | SLOW | STOP | EVADE
                                                   | EMERGENCY | ABORT
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


MODE_RETURNING = "RETURNING"


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ReturnSafetyWrapperNode(Node):
    def __init__(self) -> None:
        super().__init__("return_safety_wrapper")

        self.declare_parameter("input_topic", "/cmd_vel_return_raw")
        self.declare_parameter("output_topic", "/cmd_vel_return")
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("mode_topic", "/session_manager/mode")
        self.declare_parameter("blocked_sec_topic", "/return_safety/blocked_sec")
        self.declare_parameter("status_topic", "/return_safety/status")
        self.declare_parameter("publish_rate_hz", 20.0)

        # Safety geometry.
        self.declare_parameter("obstacle_cone_deg", 35.0)
        self.declare_parameter("slow_start_range", 1.20)
        self.declare_parameter("stop_range", 0.35)
        self.declare_parameter("emergency_range", 0.25)
        self.declare_parameter("min_linear_scale", 0.15)

        # Watchdogs / abort.
        self.declare_parameter("cmd_watchdog_sec", 0.4)
        self.declare_parameter("scan_watchdog_sec", 0.5)
        self.declare_parameter("abort_timeout_sec", 60.0)
        self.declare_parameter("clear_hysteresis_sec", 0.5)

        # EVADE (static-obstacle bypass via lateral strafe + forward creep).
        # Only meaningful on omnidirectional bases (this robot is X-drive).
        self.declare_parameter("enable_evade", True)
        # How long to stop-and-wait before attempting EVADE. Gives dynamic
        # obstacles (people) time to move out of the way first.
        self.declare_parameter("evade_after_sec", 3.0)
        self.declare_parameter("evade_speed", 0.12)
        self.declare_parameter("evade_forward_creep", 0.04)
        # Side sampling cone: ±half-width of the ray centered on the side.
        self.declare_parameter("evade_side_cone_deg", 35.0)
        # A side is "viable" only if min range in its cone exceeds this.
        self.declare_parameter("evade_side_clearance_m", 0.45)
        # If, during EVADE, the chosen side closes in below this we abort.
        self.declare_parameter("evade_side_abort_m", 0.25)
        # Forward cone must clear above this (hysteresis vs stop_range) to
        # exit EVADE back to CLEAR/SLOW.
        self.declare_parameter("evade_clear_margin_m", 0.55)
        # Give up if we've strafed this far without clearing.
        self.declare_parameter("evade_max_distance_m", 0.60)
        self.declare_parameter("evade_max_duration_sec", 8.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.blocked_sec_topic = str(self.get_parameter("blocked_sec_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.obstacle_cone_rad = math.radians(
            float(self.get_parameter("obstacle_cone_deg").value))
        self.slow_start_range = float(self.get_parameter("slow_start_range").value)
        self.stop_range = float(self.get_parameter("stop_range").value)
        self.emergency_range = float(self.get_parameter("emergency_range").value)
        self.min_linear_scale = float(self.get_parameter("min_linear_scale").value)

        self.cmd_watchdog_sec = float(self.get_parameter("cmd_watchdog_sec").value)
        self.scan_watchdog_sec = float(self.get_parameter("scan_watchdog_sec").value)
        self.abort_timeout_sec = float(self.get_parameter("abort_timeout_sec").value)
        self.clear_hysteresis_sec = float(self.get_parameter("clear_hysteresis_sec").value)

        self.enable_evade = bool(self.get_parameter("enable_evade").value)
        self.evade_after_sec = float(self.get_parameter("evade_after_sec").value)
        self.evade_speed = float(self.get_parameter("evade_speed").value)
        self.evade_forward_creep = float(self.get_parameter("evade_forward_creep").value)
        self.evade_side_cone_rad = math.radians(
            float(self.get_parameter("evade_side_cone_deg").value))
        self.evade_side_clearance_m = float(self.get_parameter("evade_side_clearance_m").value)
        self.evade_side_abort_m = float(self.get_parameter("evade_side_abort_m").value)
        self.evade_clear_margin_m = float(self.get_parameter("evade_clear_margin_m").value)
        self.evade_max_distance_m = float(self.get_parameter("evade_max_distance_m").value)
        self.evade_max_duration_sec = float(self.get_parameter("evade_max_duration_sec").value)

        if self.stop_range <= self.emergency_range:
            self.stop_range = self.emergency_range + 0.05
        if self.slow_start_range <= self.stop_range:
            self.slow_start_range = self.stop_range + 0.05
        if self.evade_clear_margin_m <= self.stop_range:
            self.evade_clear_margin_m = self.stop_range + 0.10
        if self.evade_side_abort_m >= self.evade_side_clearance_m:
            self.evade_side_abort_m = max(0.10, self.evade_side_clearance_m - 0.10)

        self._last_cmd: Optional[Twist] = None
        self._last_cmd_sec: float = 0.0
        self._last_scan: Optional[LaserScan] = None
        self._last_scan_sec: float = 0.0
        self._mode: str = ""
        self._last_mode_sec: float = 0.0

        self._blocked_start_sec: float = 0.0
        self._last_clear_sec: float = 0.0
        self._last_published_status: str = ""

        # EVADE state.
        self._stop_start_sec: float = 0.0          # when STOP first asserted (0 = not in STOP)
        self._evade_dir: int = 0                   # +1 = strafe left, -1 = strafe right
        self._evade_start_sec: float = 0.0
        self._evade_last_tick_sec: float = 0.0
        self._evade_traveled_m: float = 0.0

        qos_cmd = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_mode = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_status = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.sub_cmd = self.create_subscription(
            Twist, self.input_topic, self._on_cmd, qos_cmd)
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.sub_mode = self.create_subscription(
            String, self.mode_topic, self._on_mode, qos_mode)

        self.pub_cmd = self.create_publisher(Twist, self.output_topic, qos_cmd)
        self.pub_blocked = self.create_publisher(Float32, self.blocked_sec_topic, qos_status)
        self.pub_status = self.create_publisher(String, self.status_topic, qos_status)

        self.create_timer(1.0 / max(1.0, self.publish_rate_hz), self._tick)

        self.get_logger().info(
            f"return_safety_wrapper ready. in={self.input_topic} out={self.output_topic} "
            f"scan={self.scan_topic} slow_start={self.slow_start_range:.2f} "
            f"stop={self.stop_range:.2f} emergency={self.emergency_range:.2f} "
            f"evade={'on' if self.enable_evade else 'off'} "
            f"evade_after={self.evade_after_sec:.1f}s"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_sec = self._now()

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._last_scan_sec = self._now()

    def _on_mode(self, msg: String) -> None:
        self._mode = (msg.data or "").strip().upper()
        self._last_mode_sec = self._now()

    def _min_range_in_cone(
        self, center_rad: Optional[float], half_rad: float
    ) -> float:
        """Min valid lidar range within a cone.

        center_rad=None → full 360° scan.
        center_rad=0    → forward cone of ±half_rad.
        center_rad=+π/2 → left-side cone.
        center_rad=-π/2 → right-side cone.
        """
        scan = self._last_scan
        if scan is None or not scan.ranges:
            return math.inf
        rmin = math.inf
        angle = scan.angle_min
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment
            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            if center_rad is not None:
                if abs(_wrap_pi(a - center_rad)) > half_rad:
                    continue
            if r < rmin:
                rmin = r
        return rmin

    def _min_range(self, cone_rad: Optional[float]) -> float:
        """Back-compat helper: centered at forward if cone given, else 360°."""
        if cone_rad is None:
            return self._min_range_in_cone(None, 0.0)
        return self._min_range_in_cone(0.0, cone_rad)

    def _pick_evade_side(self) -> int:
        """Return +1 (left) or -1 (right) if a viable side exists, else 0.

        A side is viable only if its min range in the side cone exceeds
        evade_side_clearance_m AND exceeds the other side by a margin,
        so we never flip-flop.
        """
        left = self._min_range_in_cone(+math.pi / 2.0, self.evade_side_cone_rad)
        right = self._min_range_in_cone(-math.pi / 2.0, self.evade_side_cone_rad)
        left_ok = left >= self.evade_side_clearance_m
        right_ok = right >= self.evade_side_clearance_m
        if not left_ok and not right_ok:
            return 0
        if left_ok and not right_ok:
            return +1
        if right_ok and not left_ok:
            return -1
        # Both clear: take the more open side with a small bias margin.
        if left >= right + 0.05:
            return +1
        if right >= left + 0.05:
            return -1
        # Basically tied: default to left (arbitrary but stable).
        return +1

    def _enter_evade(self, now: float, side: int) -> None:
        self._evade_dir = side
        self._evade_start_sec = now
        self._evade_last_tick_sec = now
        self._evade_traveled_m = 0.0
        dirname = "LEFT" if side > 0 else "RIGHT"
        self.get_logger().warn(
            f"return_safety: entering EVADE strafe={dirname} "
            f"(stopped {now - self._stop_start_sec:.1f}s)")

    def _reset_evade(self) -> None:
        self._evade_dir = 0
        self._evade_start_sec = 0.0
        self._evade_last_tick_sec = 0.0
        self._evade_traveled_m = 0.0

    def _tick(self) -> None:
        now = self._now()
        out = Twist()
        status = "CLEAR"

        # Bypass gate: only filter during RETURNING; otherwise emit zero and
        # let cmd_vel_mux handle mode selection (it ignores us unless RETURNING).
        if self._mode != MODE_RETURNING:
            self.pub_cmd.publish(out)
            self._blocked_start_sec = 0.0
            self._stop_start_sec = 0.0
            self._reset_evade()
            self._publish_status("IDLE", blocked=0.0)
            return

        cmd_stale = (self._last_cmd is None
                     or (now - self._last_cmd_sec) > self.cmd_watchdog_sec)
        scan_stale = (self._last_scan is None
                      or (now - self._last_scan_sec) > self.scan_watchdog_sec)

        if scan_stale:
            # No trustworthy obstacle data — refuse to move.
            out = Twist()
            status = "EMERGENCY"
            self._register_blocked(now)
            self._reset_evade()
            self._emit(out, status, now)
            return

        if cmd_stale:
            # breadcrumb not publishing — emit zero, but don't treat as blocked
            # (robot is simply not being commanded; mux's watchdog also catches this).
            out = Twist()
            status = "IDLE"
            self._blocked_start_sec = 0.0
            self._stop_start_sec = 0.0
            self._reset_evade()
            self._emit(out, status, now)
            return

        raw = self._last_cmd
        fwd_range = self._min_range_in_cone(0.0, self.obstacle_cone_rad)
        near_range = self._min_range_in_cone(None, 0.0)

        vx = float(raw.linear.x)
        vy = float(raw.linear.y)
        wz = float(raw.angular.z)

        # Emergency: anything within emergency_range in any direction → freeze.
        if near_range < self.emergency_range:
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.angular.z = 0.0
            status = "EMERGENCY"
            self._register_blocked(now)
            self._reset_evade()
            self._emit(out, status, now)
            return

        # --- EVADE continuation -------------------------------------------------
        # If we're already in EVADE, check exit/abort first before re-evaluating
        # STOP policy, otherwise forward clearance during EVADE would be
        # mis-classified as plain CLEAR before we finish the strafe commit.
        if self._evade_dir != 0:
            exit_ok = fwd_range >= self.evade_clear_margin_m
            side_center = (math.pi / 2.0) * self._evade_dir
            side_range = self._min_range_in_cone(side_center, self.evade_side_cone_rad)
            duration = now - self._evade_start_sec

            if exit_ok:
                self.get_logger().info(
                    f"return_safety: EVADE cleared (fwd={fwd_range:.2f}m, "
                    f"traveled={self._evade_traveled_m:.2f}m, "
                    f"duration={duration:.1f}s) → CLEAR")
                self._reset_evade()
                self._blocked_start_sec = 0.0
                self._stop_start_sec = 0.0
                self._last_clear_sec = now
                # Fall through to normal CLEAR/SLOW handling below.
            elif side_range < self.evade_side_abort_m:
                # Chosen side is closing in — the gap isn't real. Give up.
                out.linear.x = 0.0
                out.linear.y = 0.0
                out.angular.z = 0.0
                status = "EMERGENCY"
                self.get_logger().warn(
                    f"return_safety: EVADE side closed (side={side_range:.2f}m "
                    f"< {self.evade_side_abort_m:.2f}m) — aborting")
                self._reset_evade()
                self._register_blocked(now)
                self._emit(out, status, now)
                return
            elif (self._evade_traveled_m >= self.evade_max_distance_m
                  or duration >= self.evade_max_duration_sec):
                # Gave it a fair shot; didn't clear. Fall back to STOP + blocked
                # timer so abort_timeout_sec eventually escalates to ABORT.
                out.linear.x = 0.0
                out.linear.y = 0.0
                out.angular.z = _clamp(wz, -1.0, 1.0)
                status = "STOP"
                self.get_logger().warn(
                    f"return_safety: EVADE exhausted "
                    f"(traveled={self._evade_traveled_m:.2f}m, "
                    f"duration={duration:.1f}s) — reverting to STOP")
                self._reset_evade()
                # Keep _stop_start_sec but prevent immediate re-enter of EVADE
                # by pushing it just now (evade_after_sec timer restarts).
                self._stop_start_sec = now
                self._register_blocked(now)
                self._emit(out, status, now)
                return
            else:
                # Continue strafing.
                dt = max(0.0, now - self._evade_last_tick_sec)
                self._evade_last_tick_sec = now
                self._evade_traveled_m += self.evade_speed * dt
                out.linear.x = self.evade_forward_creep
                out.linear.y = self.evade_speed * float(self._evade_dir)
                # Pass through angular so breadcrumb can still regulate heading
                # toward the next lookahead point, but clamp it.
                out.angular.z = _clamp(wz, -0.6, 0.6)
                status = "EVADE"
                # EVADE is a form of being blocked: don't reset the blocked timer.
                self._register_blocked(now)
                self._emit(out, status, now)
                return

        # Forward-cone policy applies only when commanding forward motion.
        cmd_forward = vx > 0.02
        if cmd_forward and fwd_range < self.stop_range:
            # Track continuous STOP duration so we can promote to EVADE.
            if self._stop_start_sec <= 0.0:
                self._stop_start_sec = now
            stop_dur = now - self._stop_start_sec

            if (self.enable_evade
                    and stop_dur >= self.evade_after_sec):
                side = self._pick_evade_side()
                if side != 0:
                    self._enter_evade(now, side)
                    # Publish one EVADE tick immediately so status reflects it.
                    dt = 0.0
                    self._evade_last_tick_sec = now
                    self._evade_traveled_m = 0.0
                    out.linear.x = self.evade_forward_creep
                    out.linear.y = self.evade_speed * float(side)
                    out.angular.z = _clamp(wz, -0.6, 0.6)
                    status = "EVADE"
                    self._register_blocked(now)
                    self._emit(out, status, now)
                    return
                # Both sides blocked — stay STOP, blocked_sec continues to climb.
                # abort_timeout_sec is the eventual ABORT.

            # Block: keep angular so breadcrumb can still rotate to re-align,
            # but no translation. Strafing (vy) is also forward-ish on omni —
            # kill it too to be safe.
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.angular.z = _clamp(wz, -1.0, 1.0)
            status = "STOP"
            self._register_blocked(now)
            self._emit(out, status, now)
            return

        if cmd_forward and fwd_range < self.slow_start_range:
            # Linear slow-down band.
            span = max(1e-3, self.slow_start_range - self.stop_range)
            scale = (fwd_range - self.stop_range) / span
            scale = _clamp(scale, self.min_linear_scale, 1.0)
            out.linear.x = vx * scale
            out.linear.y = vy * scale
            out.angular.z = wz
            status = "SLOW"
            # Slow is not "blocked" — reset blocked timer but remember recent clear.
            self._blocked_start_sec = 0.0
            self._stop_start_sec = 0.0
            self._last_clear_sec = now
            self._emit(out, status, now)
            return

        # Clear path.
        out.linear.x = vx
        out.linear.y = vy
        out.angular.z = wz
        status = "CLEAR"
        self._blocked_start_sec = 0.0
        self._stop_start_sec = 0.0
        self._last_clear_sec = now
        self._emit(out, status, now)

    def _register_blocked(self, now: float) -> None:
        if self._blocked_start_sec <= 0.0:
            self._blocked_start_sec = now

    def _emit(self, out: Twist, status: str, now: float) -> None:
        blocked = 0.0
        if self._blocked_start_sec > 0.0:
            blocked = now - self._blocked_start_sec

        # Escalate to ABORT if blocked too long. EVADE counts: if we've been
        # fighting the same obstacle for abort_timeout_sec across STOP+EVADE
        # the user wants us to give up and let session_manager latch STOPPED.
        if blocked >= self.abort_timeout_sec and status in ("STOP", "EMERGENCY", "EVADE"):
            status = "ABORT"

        self.pub_cmd.publish(out)
        self._publish_status(status, blocked=blocked)

    def _publish_status(self, status: str, blocked: float) -> None:
        b = Float32()
        b.data = float(blocked)
        self.pub_blocked.publish(b)

        if status != self._last_published_status:
            level = "warn" if status in ("STOP", "EMERGENCY", "ABORT", "EVADE") else "info"
            getattr(self.get_logger(), level)(
                f"return_safety: {self._last_published_status or 'INIT'} -> {status} "
                f"(blocked={blocked:.1f}s)")
            self._last_published_status = status
        s = String()
        s.data = status
        self.pub_status.publish(s)


def main() -> None:
    rclpy.init()
    node = ReturnSafetyWrapperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    try:
        node.pub_cmd.publish(Twist())
    except Exception:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
