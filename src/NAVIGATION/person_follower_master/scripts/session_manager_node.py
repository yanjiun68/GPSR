#!/usr/bin/env python3
"""Session manager for follow + return-home.

High-level state machine that owns the lifecycle of a follow-and-return
session and glues the follow stack, breadcrumb teach-and-repeat, and the
return-safety layer together.

States
------
  IDLE      : nothing active; mux outputs zero
  TEACHING  : follow controller drives the robot behind a person,
              breadcrumb records the path
  RETURNING : breadcrumb drives the robot home along the taught path,
              return_safety_wrapper gates the output
  ARRIVED   : robot reached home; quiesces before dropping back to IDLE
  STOPPED   : E-stop latched; requires a fresh /return_home/start to leave

Operator services
-----------------
  /return_home/start     (std_srvs/Trigger) - begin TEACHING
  /return_home/go_home   (std_srvs/Trigger) - transition TEACHING → RETURNING
  /return_home/stop      (std_srvs/Trigger) - hard E-stop, latch STOPPED
  /return_home/reset     (std_srvs/Trigger) - clear STOPPED/ARRIVED back to IDLE

Outbound
--------
  /session_manager/mode           (std_msgs/String, transient local)
  /session_manager/state          (std_msgs/String, transient local) — same
                                   lifecycle as mode but retains ARRIVED which
                                   the mux collapses to IDLE
  /session_manager/diagnostics    (diagnostic_msgs/DiagnosticStatus)

Automatic transitions
---------------------
  * TEACHING + master target missing for auto_go_home_on_lost_sec → RETURNING
    (configurable; set <= 0 to disable).
  * RETURNING + return_safety status == ABORT → STOPPED.
  * RETURNING + breadcrumb localized pose within arrival_tol_m of the
    taught origin AND /cmd_vel_return_raw zero for arrival_hold_sec
    → ARRIVED → IDLE.
  * RETURNING + elapsed > max(repeat_timeout_floor_sec,
      repeat_timeout_ratio * teach_duration) → STOPPED.

Inter-node service calls are made asynchronously; this node never blocks
in a service handler.
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
)

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from person_follower_master.msg import MasterTarget


STATE_IDLE = "IDLE"
STATE_TEACHING = "TEACHING"
STATE_RETURNING = "RETURNING"
STATE_ARRIVED = "ARRIVED"
STATE_STOPPED = "STOPPED"

# Mode published to cmd_vel_mux — ARRIVED and IDLE both collapse to IDLE
# (the mux only cares whether we want FOLLOW, RETURN, or nothing).
_STATE_TO_MODE = {
    STATE_IDLE: "IDLE",
    STATE_TEACHING: "TEACHING",
    STATE_RETURNING: "RETURNING",
    STATE_ARRIVED: "IDLE",
    STATE_STOPPED: "STOPPED",
}


def _quat_to_yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class SessionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("session_manager")

        # --- Params ---
        self.declare_parameter("mode_topic", "/session_manager/mode")
        self.declare_parameter("state_topic", "/session_manager/state")
        self.declare_parameter("diagnostics_topic", "/session_manager/diagnostics")
        self.declare_parameter("master_target_topic", "/master_target")
        self.declare_parameter("breadcrumb_pose_topic", "/breadcrumb_loc_pose")
        self.declare_parameter("return_cmd_raw_topic", "/cmd_vel_return_raw")
        self.declare_parameter("return_safety_status_topic", "/return_safety/status")

        self.declare_parameter("breadcrumb_start_teach_srv",
                               "/breadcrumb_teach_repeat/start_teach")
        self.declare_parameter("breadcrumb_finish_teach_srv",
                               "/breadcrumb_teach_repeat/finish_teach")
        self.declare_parameter("breadcrumb_start_repeat_srv",
                               "/breadcrumb_teach_repeat/start_repeat")
        self.declare_parameter("breadcrumb_stop_srv",
                               "/breadcrumb_teach_repeat/stop")

        # Go_home gesture: master stands facing the robot, still, for
        # auto_go_home_on_still_sec. "Facing" = |bearing| below
        # still_max_bearing_rad. "Still" = speed below still_max_speed.
        # Distance must be in [still_min_dist_m, still_max_dist_m] so an
        # occluded/partial track at 3 m can't trip it.
        #
        # This is the ONLY auto trigger. We deliberately do NOT auto-fire on
        # master-lost: while LOST, follow_controller scan-rotates the robot
        # in place, and breadcrumb records those rotations into the taught
        # path. Replaying a path poisoned with scan-rotate spins would leave
        # the robot spinning at the loss point instead of finding home.
        # If the operator truly can't do the gesture, /return_home/go_home
        # is always available.
        self.declare_parameter("auto_go_home_on_still_sec", 5.0)
        self.declare_parameter("auto_go_home_still_max_bearing", 0.30)
        self.declare_parameter("auto_go_home_still_min_dist", 0.8)
        self.declare_parameter("auto_go_home_still_max_dist", 2.0)
        self.declare_parameter("auto_go_home_still_max_speed", 0.12)

        # Auto-start TEACHING as soon as the master_tracker reports a locked
        # master. Lets the operator run the whole session hands-free: stack
        # comes up, stand in front for 3 s, robot enrolls AND starts following
        # + recording, no terminal needed.
        self.declare_parameter("auto_start_on_master_enroll", True)
        self.declare_parameter("auto_start_hold_sec", 0.5)
        # Minimum gap between auto-start attempts if breadcrumb rejects the
        # first one (e.g. service not yet ready at boot).
        self.declare_parameter("auto_start_retry_sec", 3.0)

        # Arrival detection.
        self.declare_parameter("arrival_tol_m", 0.25)
        self.declare_parameter("arrival_tol_yaw", 0.35)
        self.declare_parameter("arrival_hold_sec", 1.5)
        self.declare_parameter("arrival_zero_cmd_sec", 0.8)
        self.declare_parameter("arrived_quiesce_sec", 2.0)

        # Timeouts.
        self.declare_parameter("repeat_timeout_ratio", 3.0)
        self.declare_parameter("repeat_timeout_floor_sec", 60.0)
        self.declare_parameter("service_wait_sec", 5.0)

        self.declare_parameter("tick_rate_hz", 5.0)

        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.master_target_topic = str(self.get_parameter("master_target_topic").value)
        self.breadcrumb_pose_topic = str(self.get_parameter("breadcrumb_pose_topic").value)
        self.return_cmd_raw_topic = str(self.get_parameter("return_cmd_raw_topic").value)
        self.return_safety_status_topic = str(self.get_parameter("return_safety_status_topic").value)

        self.svc_start_teach = str(self.get_parameter("breadcrumb_start_teach_srv").value)
        self.svc_finish_teach = str(self.get_parameter("breadcrumb_finish_teach_srv").value)
        self.svc_start_repeat = str(self.get_parameter("breadcrumb_start_repeat_srv").value)
        self.svc_stop = str(self.get_parameter("breadcrumb_stop_srv").value)

        self.auto_go_home_on_still_sec = float(
            self.get_parameter("auto_go_home_on_still_sec").value)
        self.auto_go_home_still_max_bearing = float(
            self.get_parameter("auto_go_home_still_max_bearing").value)
        self.auto_go_home_still_min_dist = float(
            self.get_parameter("auto_go_home_still_min_dist").value)
        self.auto_go_home_still_max_dist = float(
            self.get_parameter("auto_go_home_still_max_dist").value)
        self.auto_go_home_still_max_speed = float(
            self.get_parameter("auto_go_home_still_max_speed").value)

        self.auto_start_on_master_enroll = bool(
            self.get_parameter("auto_start_on_master_enroll").value)
        self.auto_start_hold_sec = float(
            self.get_parameter("auto_start_hold_sec").value)
        self.auto_start_retry_sec = float(
            self.get_parameter("auto_start_retry_sec").value)

        self.arrival_tol_m = float(self.get_parameter("arrival_tol_m").value)
        self.arrival_tol_yaw = float(self.get_parameter("arrival_tol_yaw").value)
        self.arrival_hold_sec = float(self.get_parameter("arrival_hold_sec").value)
        self.arrival_zero_cmd_sec = float(self.get_parameter("arrival_zero_cmd_sec").value)
        self.arrived_quiesce_sec = float(self.get_parameter("arrived_quiesce_sec").value)

        self.repeat_timeout_ratio = float(self.get_parameter("repeat_timeout_ratio").value)
        self.repeat_timeout_floor_sec = float(self.get_parameter("repeat_timeout_floor_sec").value)
        self.service_wait_sec = float(self.get_parameter("service_wait_sec").value)

        self.tick_rate_hz = float(self.get_parameter("tick_rate_hz").value)

        # --- State ---
        self._state: str = STATE_IDLE
        self._state_entered_sec: float = self._now()

        self._last_master_sec: float = 0.0
        self._master_state: str = ""
        self._master_locked: bool = False

        self._loc_pose_dist: float = math.inf
        self._loc_pose_yaw_err: float = math.inf
        self._in_arrival_tol_since_sec: float = 0.0

        self._last_return_cmd_sec: float = 0.0
        self._last_return_cmd_zero_since_sec: float = 0.0

        self._last_safety_status: str = ""
        self._last_safety_status_sec: float = 0.0

        self._teach_started_sec: float = 0.0
        self._repeat_started_sec: float = 0.0
        self._teach_duration_sec: float = 0.0

        self._last_transition_reason: str = "init"

        # Auto-start (IDLE → TEACHING on master enrollment).
        self._master_lock_since_sec: float = 0.0
        self._last_auto_start_sec: float = 0.0

        # Still-gesture go_home (TEACHING → RETURNING when master stands
        # centered + close + not moving).
        self._master_still_since_sec: float = 0.0

        # --- QoS ---
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
        qos_default = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub_mode = self.create_publisher(String, self.mode_topic, qos_mode)
        self.pub_state = self.create_publisher(String, self.state_topic, qos_mode)
        self.pub_diag = self.create_publisher(
            DiagnosticStatus, self.diagnostics_topic, qos_default)

        # --- Subs ---
        self.create_subscription(
            MasterTarget, self.master_target_topic, self._on_master, qos_master)
        self.create_subscription(
            PoseWithCovarianceStamped, self.breadcrumb_pose_topic,
            self._on_breadcrumb_pose, qos_default)
        self.create_subscription(
            Twist, self.return_cmd_raw_topic, self._on_return_cmd_raw, qos_default)
        self.create_subscription(
            String, self.return_safety_status_topic,
            self._on_safety_status, qos_default)

        # --- Service clients (async) ---
        self.client_start_teach = self.create_client(Trigger, self.svc_start_teach)
        self.client_finish_teach = self.create_client(Trigger, self.svc_finish_teach)
        self.client_start_repeat = self.create_client(Trigger, self.svc_start_repeat)
        self.client_stop_bc = self.create_client(Trigger, self.svc_stop)

        # --- Service servers ---
        self.create_service(Trigger, "/return_home/start", self._handle_start)
        self.create_service(Trigger, "/return_home/go_home", self._handle_go_home)
        self.create_service(Trigger, "/return_home/stop", self._handle_stop)
        self.create_service(Trigger, "/return_home/reset", self._handle_reset)

        # --- Timers ---
        self.create_timer(1.0 / max(1.0, self.tick_rate_hz), self._tick)

        # Initial broadcast so late-starters get the mode immediately.
        self._publish_state_and_mode()

        self.get_logger().info("session_manager ready. state=IDLE")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_state(self, new_state: str, reason: str) -> None:
        if new_state == self._state:
            return
        old = self._state
        self._state = new_state
        self._state_entered_sec = self._now()
        self._last_transition_reason = reason
        if new_state == STATE_TEACHING:
            self._teach_started_sec = self._state_entered_sec
            self._teach_duration_sec = 0.0
        if new_state == STATE_RETURNING:
            if self._teach_started_sec > 0.0:
                self._teach_duration_sec = max(
                    10.0, self._state_entered_sec - self._teach_started_sec)
            self._repeat_started_sec = self._state_entered_sec
            # Reset arrival timers on entry.
            self._in_arrival_tol_since_sec = 0.0
            self._last_return_cmd_zero_since_sec = 0.0
        self.get_logger().info(f"state {old} -> {new_state} ({reason})")
        self._publish_state_and_mode()

    def _publish_state_and_mode(self) -> None:
        m = String()
        m.data = _STATE_TO_MODE.get(self._state, "IDLE")
        self.pub_mode.publish(m)
        s = String()
        s.data = self._state
        self.pub_state.publish(s)

    def _publish_diag(self, level: int, message: str) -> None:
        diag = DiagnosticStatus()
        diag.level = level
        diag.name = "session_manager"
        diag.message = message
        diag.hardware_id = "robot"
        now = self._now()
        diag.values = [
            KeyValue(key="state", value=self._state),
            KeyValue(key="mode", value=_STATE_TO_MODE.get(self._state, "IDLE")),
            KeyValue(key="state_age_sec",
                     value=f"{now - self._state_entered_sec:.1f}"),
            KeyValue(key="teach_duration_sec",
                     value=f"{self._teach_duration_sec:.1f}"),
            KeyValue(key="master_state", value=self._master_state),
            KeyValue(key="master_age_sec",
                     value=(f"{now - self._last_master_sec:.1f}"
                            if self._last_master_sec > 0.0 else "inf")),
            KeyValue(key="breadcrumb_dist_to_home_m",
                     value=f"{self._loc_pose_dist:.2f}"),
            KeyValue(key="breadcrumb_yaw_err_rad",
                     value=f"{self._loc_pose_yaw_err:.2f}"),
            KeyValue(key="safety_status", value=self._last_safety_status or "-"),
            KeyValue(key="last_reason", value=self._last_transition_reason),
        ]
        self.pub_diag.publish(diag)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_master(self, msg: MasterTarget) -> None:
        now = self._now()
        self._last_master_sec = now
        self._master_state = msg.state
        self._master_locked = bool(msg.is_locked)
        healthy = self._master_locked and (msg.state or "").upper() != "LOST"
        if healthy:
            if self._master_lock_since_sec <= 0.0:
                self._master_lock_since_sec = now
        else:
            self._master_lock_since_sec = 0.0

        # Still-gesture detection: master locked, centered, close, not moving.
        speed = math.hypot(float(msg.vx), float(msg.vy))
        centered = abs(float(msg.bearing)) <= self.auto_go_home_still_max_bearing
        in_range = (self.auto_go_home_still_min_dist
                    <= float(msg.distance)
                    <= self.auto_go_home_still_max_dist)
        still = speed <= self.auto_go_home_still_max_speed
        if healthy and centered and in_range and still:
            if self._master_still_since_sec <= 0.0:
                self._master_still_since_sec = now
        else:
            self._master_still_since_sec = 0.0

    def _on_breadcrumb_pose(self, msg: PoseWithCovarianceStamped) -> None:
        # Breadcrumb publishes in its own 'breadcrumb_map' frame with
        # breadcrumbs[0] anchored at origin (0,0,0). Distance to home is just
        # the norm of (x, y).
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = _quat_to_yaw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self._loc_pose_dist = math.hypot(x, y)
        self._loc_pose_yaw_err = abs(_wrap(yaw))

        now = self._now()
        if (self._loc_pose_dist <= self.arrival_tol_m
                and self._loc_pose_yaw_err <= self.arrival_tol_yaw):
            if self._in_arrival_tol_since_sec <= 0.0:
                self._in_arrival_tol_since_sec = now
        else:
            self._in_arrival_tol_since_sec = 0.0

    def _on_return_cmd_raw(self, msg: Twist) -> None:
        now = self._now()
        self._last_return_cmd_sec = now
        is_zero = (abs(msg.linear.x) < 1e-3
                   and abs(msg.linear.y) < 1e-3
                   and abs(msg.angular.z) < 1e-3)
        if is_zero:
            if self._last_return_cmd_zero_since_sec <= 0.0:
                self._last_return_cmd_zero_since_sec = now
        else:
            self._last_return_cmd_zero_since_sec = 0.0

    def _on_safety_status(self, msg: String) -> None:
        self._last_safety_status = (msg.data or "").strip().upper()
        self._last_safety_status_sec = self._now()

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------
    def _handle_start(self, _req, resp):
        if self._state == STATE_TEACHING:
            resp.success = True
            resp.message = "Already TEACHING; start ignored."
            return resp
        if self._state == STATE_RETURNING:
            resp.success = False
            resp.message = "Cannot start teach while RETURNING. Call /return_home/stop first."
            return resp
        if self._state == STATE_STOPPED:
            resp.success = False
            resp.message = "STOPPED latched; call /return_home/reset first."
            return resp
        if not self.client_start_teach.service_is_ready():
            if not self.client_start_teach.wait_for_service(timeout_sec=self.service_wait_sec):
                resp.success = False
                resp.message = f"breadcrumb service {self.svc_start_teach} unavailable"
                return resp

        self._set_state(STATE_TEACHING, "operator: /return_home/start")
        future = self.client_start_teach.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._on_breadcrumb_reply(
            f, "start_teach", failure_transition=STATE_IDLE,
            failure_reason="breadcrumb start_teach failed"))
        resp.success = True
        resp.message = "Teach phase armed; follow_controller driving, breadcrumb recording."
        return resp

    def _handle_go_home(self, _req, resp):
        if self._state != STATE_TEACHING:
            resp.success = False
            resp.message = f"Cannot go_home from state {self._state}. Expect TEACHING."
            return resp
        if not self.client_finish_teach.service_is_ready():
            if not self.client_finish_teach.wait_for_service(timeout_sec=self.service_wait_sec):
                resp.success = False
                resp.message = f"breadcrumb service {self.svc_finish_teach} unavailable"
                return resp
        # Optimistic transition; if breadcrumb rejects we roll back.
        self._set_state(STATE_RETURNING, "operator: /return_home/go_home")

        finish_future = self.client_finish_teach.call_async(Trigger.Request())

        def _after_finish(fut):
            ok, msg = self._extract_trigger(fut)
            if not ok:
                self.get_logger().error(
                    f"breadcrumb finish_teach failed: {msg} — latching STOPPED")
                self._set_state(STATE_STOPPED,
                                f"breadcrumb finish_teach failed: {msg}")
                return
            self.get_logger().info(f"breadcrumb finish_teach OK: {msg}")
            if not self.client_start_repeat.service_is_ready():
                self.client_start_repeat.wait_for_service(timeout_sec=self.service_wait_sec)
            repeat_future = self.client_start_repeat.call_async(Trigger.Request())
            repeat_future.add_done_callback(lambda rf: self._on_breadcrumb_reply(
                rf, "start_repeat", failure_transition=STATE_STOPPED,
                failure_reason="breadcrumb start_repeat failed"))

        finish_future.add_done_callback(_after_finish)
        resp.success = True
        resp.message = "Return-home armed; breadcrumb finalising & starting repeat."
        return resp

    def _handle_stop(self, _req, resp):
        prev = self._state
        self._set_state(STATE_STOPPED, "operator: /return_home/stop")
        # Fire-and-forget breadcrumb stop so it releases /cmd_vel_return_raw.
        if self.client_stop_bc.service_is_ready() or self.client_stop_bc.wait_for_service(timeout_sec=1.0):
            self.client_stop_bc.call_async(Trigger.Request())
        resp.success = True
        resp.message = f"STOPPED latched (was {prev}). Call /return_home/reset to clear."
        return resp

    def _handle_reset(self, _req, resp):
        if self._state in (STATE_STOPPED, STATE_ARRIVED, STATE_IDLE):
            self._set_state(STATE_IDLE, "operator: /return_home/reset")
            resp.success = True
            resp.message = "session_manager reset to IDLE."
        else:
            resp.success = False
            resp.message = (
                f"Refusing to reset from {self._state}. Call /return_home/stop first.")
        return resp

    def _on_breadcrumb_reply(self, fut, label: str,
                             failure_transition: str, failure_reason: str) -> None:
        ok, msg = self._extract_trigger(fut)
        if not ok:
            self.get_logger().error(f"breadcrumb {label} failed: {msg}")
            self._set_state(failure_transition, failure_reason + f": {msg}")
        else:
            self.get_logger().info(f"breadcrumb {label} OK: {msg}")

    @staticmethod
    def _extract_trigger(fut) -> tuple:
        try:
            result = fut.result()
        except Exception as exc:  # noqa: BLE001
            return False, f"exception: {exc}"
        if result is None:
            return False, "no result"
        return bool(result.success), str(result.message or "")

    # ------------------------------------------------------------------
    # Tick — autonomous transitions + diagnostics
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        now = self._now()
        self._publish_state_and_mode()

        if self._state == STATE_IDLE:
            self._check_auto_start(now)
        elif self._state == STATE_TEACHING:
            self._check_auto_go_home(now)
        elif self._state == STATE_RETURNING:
            self._check_returning_conditions(now)
        elif self._state == STATE_ARRIVED:
            if (now - self._state_entered_sec) >= self.arrived_quiesce_sec:
                self._set_state(STATE_IDLE, "arrived quiesce complete")

        # Diagnostics level: OK unless stopped or safety ABORT.
        level = DiagnosticStatus.OK
        message = f"state={self._state}"
        if self._state == STATE_STOPPED:
            level = DiagnosticStatus.ERROR
            message = f"STOPPED ({self._last_transition_reason})"
        elif self._state == STATE_RETURNING and self._last_safety_status in ("STOP", "EMERGENCY"):
            level = DiagnosticStatus.WARN
            message = f"RETURNING blocked ({self._last_safety_status})"
        elif self._state == STATE_RETURNING and self._last_safety_status == "ABORT":
            level = DiagnosticStatus.ERROR
            message = "RETURNING abort from safety wrapper"
        self._publish_diag(level, message)

    def _check_auto_start(self, now: float) -> None:
        if not self.auto_start_on_master_enroll:
            return
        if self._master_lock_since_sec <= 0.0:
            return
        if (now - self._master_lock_since_sec) < self.auto_start_hold_sec:
            return
        if (self._last_auto_start_sec > 0.0
                and (now - self._last_auto_start_sec) < self.auto_start_retry_sec):
            return
        self._last_auto_start_sec = now
        self.get_logger().info(
            f"auto-start: master locked (state={self._master_state}) "
            f"for {(now - self._master_lock_since_sec):.1f}s — entering TEACHING")
        resp = Trigger.Response()
        self._handle_start(Trigger.Request(), resp)
        if not resp.success:
            self.get_logger().warn(
                f"auto-start rejected: {resp.message}; will retry in "
                f"{self.auto_start_retry_sec:.1f}s")

    def _check_auto_go_home(self, now: float) -> None:
        # Only trigger: master stands facing the robot, still, for
        # auto_go_home_on_still_sec. See __init__ for why we don't auto-fire
        # on master-lost (scan-rotate would poison the breadcrumb path).
        if self.auto_go_home_on_still_sec <= 0.0:
            return
        if self._master_still_since_sec <= 0.0:
            return
        if (now - self._master_still_since_sec) < self.auto_go_home_on_still_sec:
            return
        held = now - self._master_still_since_sec
        self.get_logger().warn(
            f"auto-trigger: master stood still >= {self.auto_go_home_on_still_sec:.1f}s "
            f"(held={held:.1f}s) — going home")
        self._master_still_since_sec = 0.0
        self._handle_go_home(Trigger.Request(), Trigger.Response())

    def _check_returning_conditions(self, now: float) -> None:
        # 1. Safety ABORT → latch STOPPED.
        if self._last_safety_status == "ABORT":
            self._set_state(STATE_STOPPED, "return_safety ABORT")
            # Fire-and-forget breadcrumb stop.
            if self.client_stop_bc.service_is_ready():
                self.client_stop_bc.call_async(Trigger.Request())
            return

        # 2. Global repeat timeout.
        elapsed = now - self._repeat_started_sec
        budget = max(self.repeat_timeout_floor_sec,
                     self.repeat_timeout_ratio * self._teach_duration_sec)
        if elapsed > budget:
            self.get_logger().error(
                f"RETURNING timeout ({elapsed:.0f}s > {budget:.0f}s). Latching STOPPED.")
            self._set_state(STATE_STOPPED, f"return timeout after {elapsed:.0f}s")
            if self.client_stop_bc.service_is_ready():
                self.client_stop_bc.call_async(Trigger.Request())
            return

        # 3. Arrival detection: within tolerance, cmd zero held, raw cmd fresh.
        in_tol_ok = (self._in_arrival_tol_since_sec > 0.0
                     and (now - self._in_arrival_tol_since_sec) >= self.arrival_hold_sec)
        zero_ok = (self._last_return_cmd_zero_since_sec > 0.0
                   and (now - self._last_return_cmd_zero_since_sec) >= self.arrival_zero_cmd_sec)
        raw_fresh = (self._last_return_cmd_sec > 0.0
                     and (now - self._last_return_cmd_sec) <= 2.0)
        if in_tol_ok and zero_ok and raw_fresh:
            self._set_state(STATE_ARRIVED,
                            f"arrived (d={self._loc_pose_dist:.2f}m, "
                            f"yaw_err={self._loc_pose_yaw_err:.2f}rad)")


def main() -> None:
    rclpy.init()
    node = SessionManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
