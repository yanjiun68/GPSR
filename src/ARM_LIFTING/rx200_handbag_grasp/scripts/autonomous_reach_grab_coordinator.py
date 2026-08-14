#!/usr/bin/env python3
"""
Autonomous reach-and-grab coordinator for PutrabotUPM.

This node glues together existing subsystems:
  - VISION/scripts/vision_node.py publishes 3D detections on /cam_detections
  - NAVIGATION/putrabot_base/scripts/cmd_vel.py consumes /cmd_vel
  - ARM_LIFTING/rx200_handbag_grasp/scripts/grasp_node.py exposes /grasp/execute

Behavior:
  1) Track the best visible object from /cam_detections (already in base frame)
  2) Drive the mobile base until the object reaches a configurable pickup window
  3) Stop and trigger /grasp/execute for precise arm + gripper grasping
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from vision.msg import CamDetection, CamDetections


class AutoState(Enum):
    SEARCH = auto()
    APPROACH = auto()
    REQUEST_GRASP = auto()
    WAIT_GRASP = auto()
    SUCCESS = auto()
    RETRY = auto()


@dataclass
class TrackedTarget:
    x: float
    y: float
    z: float
    confidence: float
    stamp_sec: float


class AutonomousReachGrabCoordinator(Node):
    def __init__(self) -> None:
        super().__init__('autonomous_reach_grab_coordinator')

        # Topics / services
        self.declare_parameter('detection_topic', '/cam_detections')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('grasp_service', '/grasp/execute')
        self.declare_parameter('grasp_busy_topic', '/grasp/busy')

        # Target selection
        self.declare_parameter('min_confidence', 0.55)
        self.declare_parameter('detection_timeout_sec', 0.7)
        self.declare_parameter('workspace_x_min', 0.10)
        self.declare_parameter('workspace_x_max', 1.50)
        self.declare_parameter('workspace_y_abs_max', 0.80)
        self.declare_parameter('target_filter_alpha', 0.30)

        # Mobile base alignment setpoint (object position in base frame)
        self.declare_parameter('pickup_x', 0.34)
        self.declare_parameter('pickup_y', 0.00)
        self.declare_parameter('x_tolerance', 0.03)
        self.declare_parameter('y_tolerance', 0.03)
        self.declare_parameter('stable_cycles_required', 8)

        # Base motion controller
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('kp_x', 1.1)
        self.declare_parameter('kp_y', 1.4)
        self.declare_parameter('max_vx', 0.20)
        self.declare_parameter('max_vy', 0.20)
        self.declare_parameter('min_cmd_speed', 0.02)

        # Retry / recovery
        self.declare_parameter('max_grasp_attempts', 2)
        self.declare_parameter('retry_wait_sec', 1.0)
        self.declare_parameter('auto_reset_after_success', False)

        # Load params
        self.detection_topic = self.get_parameter('detection_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.grasp_service_name = self.get_parameter('grasp_service').value
        self.grasp_busy_topic = self.get_parameter('grasp_busy_topic').value

        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.detection_timeout_sec = float(self.get_parameter('detection_timeout_sec').value)
        self.workspace_x_min = float(self.get_parameter('workspace_x_min').value)
        self.workspace_x_max = float(self.get_parameter('workspace_x_max').value)
        self.workspace_y_abs_max = float(self.get_parameter('workspace_y_abs_max').value)
        self.target_filter_alpha = float(self.get_parameter('target_filter_alpha').value)

        self.pickup_x = float(self.get_parameter('pickup_x').value)
        self.pickup_y = float(self.get_parameter('pickup_y').value)
        self.x_tolerance = float(self.get_parameter('x_tolerance').value)
        self.y_tolerance = float(self.get_parameter('y_tolerance').value)
        self.stable_cycles_required = int(self.get_parameter('stable_cycles_required').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.kp_x = float(self.get_parameter('kp_x').value)
        self.kp_y = float(self.get_parameter('kp_y').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.min_cmd_speed = float(self.get_parameter('min_cmd_speed').value)

        self.max_grasp_attempts = int(self.get_parameter('max_grasp_attempts').value)
        self.retry_wait_sec = float(self.get_parameter('retry_wait_sec').value)
        self.auto_reset_after_success = bool(self.get_parameter('auto_reset_after_success').value)

        # Runtime state
        self.state = AutoState.SEARCH
        self.arm_busy = False
        self.last_target: Optional[TrackedTarget] = None
        self._filtered_target: Optional[TrackedTarget] = None
        self._stable_cycles = 0
        self._grasp_attempts = 0
        self._grasp_future = None
        self._retry_deadline_sec = 0.0
        self._last_log_times = {}

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.det_sub = self.create_subscription(
            CamDetections, self.detection_topic, self._detections_cb, 10
        )
        self.busy_sub = self.create_subscription(
            Bool, self.grasp_busy_topic, self._busy_cb, 10
        )
        self.grasp_client = self.create_client(Trigger, self.grasp_service_name)

        self.control_timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)

        self.get_logger().info('Autonomous reach-grab coordinator started')
        self.get_logger().info(
            f'Listening {self.detection_topic}, driving {self.cmd_vel_topic}, calling {self.grasp_service_name}'
        )
        self.get_logger().info(
            f'Pickup window x={self.pickup_x:.2f}±{self.x_tolerance:.2f}, '
            f'y={self.pickup_y:.2f}±{self.y_tolerance:.2f}'
        )

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------
    def _busy_cb(self, msg: Bool) -> None:
        self.arm_busy = bool(msg.data)

    def _detections_cb(self, msg: CamDetections) -> None:
        now_sec = self._now_sec()
        best = self._select_best_detection(msg.detections)
        if best is None:
            return

        target = TrackedTarget(
            x=float(best.x),
            y=float(best.y),
            z=float(best.z),
            confidence=float(best.confidence),
            stamp_sec=now_sec,
        )

        if self._filtered_target is None:
            self._filtered_target = target
        else:
            a = self.target_filter_alpha
            prev = self._filtered_target
            self._filtered_target = TrackedTarget(
                x=(1.0 - a) * prev.x + a * target.x,
                y=(1.0 - a) * prev.y + a * target.y,
                z=(1.0 - a) * prev.z + a * target.z,
                confidence=target.confidence,
                stamp_sec=now_sec,
            )

        self.last_target = self._filtered_target

    # --------------------------------------------------
    # Core control loop
    # --------------------------------------------------
    def _control_loop(self) -> None:
        target = self._get_fresh_target()

        if self.state == AutoState.SEARCH:
            self._publish_stop()
            self._stable_cycles = 0
            if target is not None:
                self._set_state(AutoState.APPROACH, 'target acquired')
            else:
                self._log_throttled('search', 'Waiting for valid /cam_detections...', 2.0)
            return

        if self.state == AutoState.APPROACH:
            if target is None:
                self._publish_stop()
                self._set_state(AutoState.SEARCH, 'target lost')
                return

            ex = target.x - self.pickup_x
            ey = target.y - self.pickup_y

            if abs(ex) <= self.x_tolerance and abs(ey) <= self.y_tolerance:
                self._stable_cycles += 1
                self._publish_stop()
                self._log_throttled(
                    'stabilize',
                    f'Target in pickup window ({self._stable_cycles}/{self.stable_cycles_required})',
                    0.5,
                )
                if self._stable_cycles >= self.stable_cycles_required:
                    self._set_state(AutoState.REQUEST_GRASP, 'target stabilized')
                return

            self._stable_cycles = 0
            vx = self._apply_min_speed(self._clamp(self.kp_x * ex, -self.max_vx, self.max_vx))
            vy = self._apply_min_speed(self._clamp(self.kp_y * ey, -self.max_vy, self.max_vy))
            self._publish_cmd(vx=vx, vy=vy, wz=0.0)
            self._log_throttled(
                'approach',
                f'Approaching target: ex={ex:+.3f} m ey={ey:+.3f} m -> vx={vx:+.2f} vy={vy:+.2f}',
                0.5,
            )
            return

        if self.state == AutoState.REQUEST_GRASP:
            self._publish_stop()

            if target is None:
                self._set_state(AutoState.SEARCH, 'target stale before grasp')
                return

            if self.arm_busy:
                self._log_throttled('arm_busy', 'Grasp node busy, waiting...', 1.0)
                return

            if self._grasp_future is None:
                if not self.grasp_client.service_is_ready():
                    self._log_throttled(
                        'svc_wait',
                        f'Waiting for service {self.grasp_service_name}...',
                        1.0,
                    )
                    return
                req = Trigger.Request()
                self._grasp_future = self.grasp_client.call_async(req)
                self._set_state(AutoState.WAIT_GRASP, 'grasp request sent')
            return

        if self.state == AutoState.WAIT_GRASP:
            self._publish_stop()

            if self._grasp_future is None:
                self._set_state(AutoState.RETRY, 'missing grasp future')
                return

            if not self._grasp_future.done():
                self._log_throttled('wait_grasp', 'Waiting for /grasp/execute result...', 1.0)
                return

            future = self._grasp_future
            self._grasp_future = None

            try:
                resp = future.result()
            except Exception as exc:  # noqa: BLE001
                self._grasp_attempts += 1
                self.get_logger().error(f'/grasp/execute call failed: {exc}')
                self._schedule_retry()
                return

            if resp is not None and bool(resp.success):
                self._publish_stop()
                self._set_state(AutoState.SUCCESS, f'grasp success: {resp.message}')
                return

            self._grasp_attempts += 1
            msg = resp.message if resp is not None else 'no response'
            self.get_logger().warn(f'Grasp failed (attempt {self._grasp_attempts}): {msg}')
            self._schedule_retry()
            return

        if self.state == AutoState.RETRY:
            self._publish_stop()
            if self._now_sec() >= self._retry_deadline_sec:
                self._set_state(AutoState.SEARCH, 'retry timeout elapsed')
            return

        if self.state == AutoState.SUCCESS:
            self._publish_stop()
            if self.auto_reset_after_success:
                self._grasp_attempts = 0
                self._set_state(AutoState.SEARCH, 'auto reset after success')
            return

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------
    def _schedule_retry(self) -> None:
        if self._grasp_attempts > self.max_grasp_attempts:
            self.get_logger().error(
                f'Max grasp attempts exceeded ({self.max_grasp_attempts}). Returning to SEARCH.'
            )
            self._grasp_attempts = 0
            self._set_state(AutoState.SEARCH, 'max attempts exceeded')
            return
        self._retry_deadline_sec = self._now_sec() + self.retry_wait_sec
        self._set_state(AutoState.RETRY, f'retrying in {self.retry_wait_sec:.1f}s')

    def _get_fresh_target(self) -> Optional[TrackedTarget]:
        if self.last_target is None:
            return None
        age = self._now_sec() - self.last_target.stamp_sec
        if age > self.detection_timeout_sec:
            return None
        return self.last_target

    def _select_best_detection(self, detections) -> Optional[CamDetection]:
        best_det = None
        best_score = None

        for det in detections:
            conf = float(det.confidence)
            x = float(det.x)
            y = float(det.y)
            if conf < self.min_confidence:
                continue
            if x < self.workspace_x_min or x > self.workspace_x_max:
                continue
            if abs(y) > self.workspace_y_abs_max:
                continue

            # Prefer detections already closest to the arm pickup setpoint.
            dx = x - self.pickup_x
            dy = y - self.pickup_y
            dist_score = (dx * dx + dy * dy) ** 0.5
            conf_bonus = 0.2 * conf
            score = dist_score - conf_bonus

            if best_score is None or score < best_score:
                best_score = score
                best_det = det

        return best_det

    def _publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0, 0.0)

    def _set_state(self, new_state: AutoState, reason: str) -> None:
        if self.state == new_state:
            return
        self.get_logger().info(f'State {self.state.name} -> {new_state.name} ({reason})')
        self.state = new_state
        if new_state in (AutoState.SEARCH, AutoState.RETRY, AutoState.SUCCESS):
            self._stable_cycles = 0

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def _apply_min_speed(self, cmd: float) -> float:
        if abs(cmd) < 1e-6:
            return 0.0
        if abs(cmd) < self.min_cmd_speed:
            return self.min_cmd_speed if cmd > 0.0 else -self.min_cmd_speed
        return cmd

    def _log_throttled(self, key: str, msg: str, period_sec: float) -> None:
        now = self._now_sec()
        last = self._last_log_times.get(key, -1e9)
        if (now - last) >= period_sec:
            self._last_log_times[key] = now
            self.get_logger().info(msg)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self):  # type: ignore[override]
        try:
            self._publish_stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousReachGrabCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
