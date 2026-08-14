#!/usr/bin/env python3
"""cmd_vel multiplexer for the follow / return-home stack.

Subscribes to two upstream Twist topics and selects exactly one to
forward to the downstream /cmd_vel consumed by cmd_vel_to_targets.

Sources:
  /cmd_vel_follow   -- follow_controller_node.py output (human-follow mode)
  /cmd_vel_return   -- return_safety_wrapper output (home-return mode, already
                       obstacle-scaled)

Mode is chosen by /session_manager/mode (std_msgs/String):
  IDLE      -> zero
  TEACHING  -> forward /cmd_vel_follow
  RETURNING -> forward /cmd_vel_return
  STOPPED   -> zero (E-stop latch)

Safety behaviour (independent of upstream):
  * Per-source watchdog: if the selected source has not published within
    source_watchdog_sec, output zero (better stopped than stale).
  * Mode watchdog: if /session_manager/mode has not been heard within
    mode_watchdog_sec, output zero. Protects against session_manager dying.
  * Stopped latch: once mode STOPPED is observed, output zero until a
    non-STOPPED mode is seen again (session_manager is authoritative).
  * Publishes /cmd_vel_mux/active_source (std_msgs/String) so rosbags can
    replay which source was driving the robot at every instant.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from person_follower_master.msg import MasterTarget


MODE_IDLE = "IDLE"
MODE_TEACHING = "TEACHING"
MODE_RETURNING = "RETURNING"
MODE_STOPPED = "STOPPED"


class CmdVelMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")

        self.declare_parameter("follow_topic", "/cmd_vel_follow")
        self.declare_parameter("return_topic", "/cmd_vel_return")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("mode_topic", "/session_manager/mode")
        self.declare_parameter("master_topic", "/master_target")
        self.declare_parameter("active_source_topic", "/cmd_vel_mux/active_source")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("source_watchdog_sec", 0.25)
        self.declare_parameter("mode_watchdog_sec", 3.0)
        # TEACHING-only: if master has been unhealthy (LOST or stream silent)
        # continuously for this many seconds, the mux swallows /cmd_vel_follow
        # and outputs zero. Purpose: prevent follow_controller's scan-rotate
        # "searching for master" spins from poisoning the breadcrumb path.
        # Grace window lets brief per-frame dropouts pass through unchanged.
        # Keep slightly below follow_controller's scan-rotate threshold (2 s).
        self.declare_parameter("master_lost_grace_sec", 1.8)
        # After master stream silence this long, count the master as lost
        # regardless of last-known state.
        self.declare_parameter("master_stream_silence_sec", 1.0)

        self.follow_topic = str(self.get_parameter("follow_topic").value)
        self.return_topic = str(self.get_parameter("return_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.master_topic = str(self.get_parameter("master_topic").value)
        self.active_source_topic = str(self.get_parameter("active_source_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.source_watchdog_sec = float(self.get_parameter("source_watchdog_sec").value)
        self.mode_watchdog_sec = float(self.get_parameter("mode_watchdog_sec").value)
        self.master_lost_grace_sec = float(self.get_parameter("master_lost_grace_sec").value)
        self.master_stream_silence_sec = float(
            self.get_parameter("master_stream_silence_sec").value)

        self._last_follow: Twist = Twist()
        self._last_return: Twist = Twist()
        self._last_follow_sec: float = 0.0
        self._last_return_sec: float = 0.0

        self._mode: str = MODE_IDLE
        self._last_mode_sec: float = 0.0
        self._last_published_source: str = "ZERO"

        # Master-loss tracking. "Unhealthy" = !is_locked OR state==LOST OR
        # stream silent > master_stream_silence_sec. We gate only after
        # master_lost_grace_sec of *continuous* unhealthiness so single-frame
        # detection misses pass through unaffected.
        self._last_master_sec: float = 0.0
        self._master_locked: bool = False
        self._master_state: str = ""
        self._master_unhealthy_since_sec: float = 0.0

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
        qos_master = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub_follow = self.create_subscription(
            Twist, self.follow_topic, self._on_follow, qos_cmd)
        self.sub_return = self.create_subscription(
            Twist, self.return_topic, self._on_return, qos_cmd)
        self.sub_mode = self.create_subscription(
            String, self.mode_topic, self._on_mode, qos_mode)
        self.sub_master = self.create_subscription(
            MasterTarget, self.master_topic, self._on_master, qos_master)

        self.pub_cmd = self.create_publisher(Twist, self.output_topic, qos_cmd)
        self.pub_src = self.create_publisher(String, self.active_source_topic, qos_mode)

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"cmd_vel_mux ready. follow={self.follow_topic} return={self.return_topic} "
            f"output={self.output_topic} mode_topic={self.mode_topic}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_follow(self, msg: Twist) -> None:
        self._last_follow = msg
        self._last_follow_sec = self._now()

    def _on_return(self, msg: Twist) -> None:
        self._last_return = msg
        self._last_return_sec = self._now()

    def _on_mode(self, msg: String) -> None:
        new_mode = (msg.data or "").strip().upper()
        if new_mode not in (MODE_IDLE, MODE_TEACHING, MODE_RETURNING, MODE_STOPPED):
            self.get_logger().warn(
                f"cmd_vel_mux: ignoring unknown mode='{new_mode}'",
                throttle_duration_sec=2.0)
            return
        if new_mode != self._mode:
            self.get_logger().info(f"cmd_vel_mux: mode {self._mode} -> {new_mode}")
        self._mode = new_mode
        self._last_mode_sec = self._now()

    def _on_master(self, msg: MasterTarget) -> None:
        self._last_master_sec = self._now()
        self._master_locked = bool(msg.is_locked)
        self._master_state = (msg.state or "").strip().upper()

    def _master_unhealthy(self, now: float) -> bool:
        # Stream silent → treat as lost.
        if self._last_master_sec <= 0.0:
            return True
        if (now - self._last_master_sec) > self.master_stream_silence_sec:
            return True
        # Last known state unhealthy.
        if not self._master_locked:
            return True
        if self._master_state == "LOST":
            return True
        return False

    def _tick(self) -> None:
        now = self._now()
        out = Twist()
        source = "ZERO"

        # Track continuous unhealthy duration regardless of mode, so the
        # timer is already warmed up the moment we enter TEACHING.
        if self._master_unhealthy(now):
            if self._master_unhealthy_since_sec <= 0.0:
                self._master_unhealthy_since_sec = now
        else:
            self._master_unhealthy_since_sec = 0.0

        mode_stale = (self._last_mode_sec <= 0.0
                      or (now - self._last_mode_sec) > self.mode_watchdog_sec)

        if mode_stale:
            source = "ZERO_MODE_STALE"
        elif self._mode == MODE_STOPPED:
            source = "ZERO_STOPPED"
        elif self._mode == MODE_IDLE:
            source = "ZERO_IDLE"
        elif self._mode == MODE_TEACHING:
            unhealthy_age = (
                (now - self._master_unhealthy_since_sec)
                if self._master_unhealthy_since_sec > 0.0 else 0.0)
            if unhealthy_age >= self.master_lost_grace_sec:
                # Master has been lost long enough that follow_controller is
                # about to (or already has) started scan-rotating. Swallow
                # the cmd so the robot stops and breadcrumb records nothing.
                source = "ZERO_MASTER_LOST"
            elif (now - self._last_follow_sec) <= self.source_watchdog_sec:
                out = self._last_follow
                source = "FOLLOW"
            else:
                source = "ZERO_FOLLOW_STALE"
        elif self._mode == MODE_RETURNING:
            if (now - self._last_return_sec) <= self.source_watchdog_sec:
                out = self._last_return
                source = "RETURN"
            else:
                source = "ZERO_RETURN_STALE"
        else:
            source = "ZERO_UNKNOWN"

        self.pub_cmd.publish(out)
        if source != self._last_published_source:
            self.get_logger().info(f"cmd_vel_mux: active_source {self._last_published_source} -> {source}")
            self._last_published_source = source
        msg = String()
        msg.data = source
        self.pub_src.publish(msg)


def main() -> None:
    rclpy.init()
    node = CmdVelMuxNode()
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
