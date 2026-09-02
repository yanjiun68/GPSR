"""ROS 2 action server for feedback-aware RX200 gripper closing."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from interbotix_xs_msgs.msg import JointSingleCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rx200_gripper_guard_interfaces.action import CloseUntilContact
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from rx200_gripper_guard import constants as phase
from rx200_gripper_guard.contact_detector import ContactDetector, ContactObservation
from rx200_gripper_guard.guard_state_machine import GuardStateMachine
from rx200_gripper_guard.health_monitor import HealthMonitor


@dataclass
class GripperFeedback:
    """Thread-safe snapshot payload derived only from actual named joints."""

    timestamp: float = 0.0
    motor_position: float = 0.0
    motor_velocity: float = 0.0
    motor_effort: float = 0.0
    finger_aperture: float = 0.0
    right_finger_position: float = 0.0
    feedback_valid: bool = False
    sequence: int = 0


class GripperGuardNode(Node):
    """Own guarded-close PWM commands while leaving hardware ownership with xs_sdk."""

    def __init__(self, **kwargs) -> None:
        """Configure parameters, isolated callback groups, ROS interfaces, and diagnostics."""
        super().__init__("rx200_gripper_guard", **kwargs)
        self._declare_parameters()
        self._load_parameters()

        # Section: Keep feedback, action execution, and register health callbacks independent.
        self.feedback_group = ReentrantCallbackGroup()
        self.action_group = ReentrantCallbackGroup()
        self.health_group = MutuallyExclusiveCallbackGroup()
        self._feedback_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._feedback = GripperFeedback()
        self._load_history: deque[float] = deque(maxlen=self.history_length)
        self._goal_reserved = False
        self._commanded_pwm = 0.0
        self._close_commands_enabled = False
        self._stop_published = True
        self._pwm_mode_verified_once = False
        self._load_baseline = 0.0
        self._phase = phase.IDLE
        self._last_observation = self._empty_observation()
        self._feedback_announced = False

        # Section: Communicate exclusively through the existing Interbotix ROS interfaces.
        self.command_publisher = self.create_publisher(
            JointSingleCommand, self.gripper_command_topic, 10
        )
        self.contact_publisher = self.create_publisher(Bool, self.contact_topic, 10)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, self.diagnostics_topic, 10
        )
        self.joint_subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_state_callback,
            10,
            callback_group=self.feedback_group,
        )
        self.health_monitor = HealthMonitor(
            self,
            self.register_service,
            self.motor_joint,
            self.health_fast_update_rate_hz,
            self.health_medium_update_rate_hz,
            self.health_slow_update_rate_hz,
            self.health_group,
            self.health_enabled,
        )
        self.action_server = ActionServer(
            self,
            CloseUntilContact,
            self.action_name,
            execute_callback=self._execute_close,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.action_group,
        )
        self.diagnostic_timer = self.create_timer(
            1.0,
            self._publish_diagnostics,
            callback_group=self.health_group,
        )
        self.get_logger().info("Gripper guard waiting for joint feedback")

    def _declare_parameters(self) -> None:
        """Declare every topic, threshold, calibration value, and command bound."""
        # Section: Interface and joint names.
        self.declare_parameter("robot_name", "rx200")
        self.declare_parameter("topics.joint_states", phase.DEFAULT_JOINT_STATES_TOPIC)
        self.declare_parameter("topics.gripper_command", phase.DEFAULT_GRIPPER_COMMAND_TOPIC)
        self.declare_parameter("topics.diagnostics", phase.DEFAULT_DIAGNOSTICS_TOPIC)
        self.declare_parameter("topics.contact", phase.DEFAULT_CONTACT_TOPIC)
        self.declare_parameter("services.get_motor_registers", phase.DEFAULT_REGISTER_SERVICE)
        self.declare_parameter("actions.close_until_contact", phase.DEFAULT_ACTION_NAME)
        self.declare_parameter("joints.motor_joint", "gripper")
        self.declare_parameter("joints.finger_joint", "left_finger")
        self.declare_parameter("joints.right_finger_joint", "right_finger")

        # Section: Existing close strength is the maximum allowed guarded-close magnitude.
        self.declare_parameter("control.default_close_pwm", -75.0)
        self.declare_parameter("control.default_hold_pwm", 0.0)
        self.declare_parameter("control.command_period_sec", 0.05)
        self.declare_parameter("control.min_pwm", -200.0)
        self.declare_parameter("control.max_pwm", 200.0)
        self.declare_parameter("control.max_close_pwm_magnitude", 75.0)
        self.declare_parameter("control.command_mode", "pwm")
        self.declare_parameter("control.expected_operating_mode", phase.PWM_OPERATING_MODE)

        # Section: Conservative initial thresholds are positive and remain calibratable.
        self.declare_parameter("contact.required_samples", 3)
        self.declare_parameter("contact.velocity_threshold", 0.02)
        self.declare_parameter("contact.position_span_threshold", 0.003)
        self.declare_parameter("contact.load_delta_threshold", 10.0)
        self.declare_parameter("contact.require_load_confirmation", False)
        self.declare_parameter("contact.aperture_margin", 0.001)
        self.declare_parameter("contact.history_length", 3)
        self.declare_parameter("calibration.empty_closed_aperture", 0.015)

        # Section: Safety timeouts and low-rate health polling.
        self.declare_parameter("safety.feedback_timeout_sec", 0.25)
        self.declare_parameter("safety.action_timeout_sec", 5.0)
        self.declare_parameter("safety.stop_pwm", 0.0)
        self.declare_parameter("health.enabled", True)
        self.declare_parameter("health.fast_update_rate_hz", 10.0)
        self.declare_parameter("health.medium_update_rate_hz", 2.0)
        self.declare_parameter("health.slow_update_rate_hz", 0.2)
        self.declare_parameter("health.critical_status_timeout_sec", 0.5)

    def _load_parameters(self) -> None:
        """Read declared values once for deterministic action behavior."""
        # Section: ROS interface parameters.
        self.joint_states_topic = str(self.get_parameter("topics.joint_states").value)
        self.gripper_command_topic = str(self.get_parameter("topics.gripper_command").value)
        self.diagnostics_topic = str(self.get_parameter("topics.diagnostics").value)
        self.contact_topic = str(self.get_parameter("topics.contact").value)
        self.register_service = str(self.get_parameter("services.get_motor_registers").value)
        self.action_name = str(self.get_parameter("actions.close_until_contact").value)
        self.motor_joint = str(self.get_parameter("joints.motor_joint").value)
        self.finger_joint = str(self.get_parameter("joints.finger_joint").value)
        self.right_finger_joint = str(self.get_parameter("joints.right_finger_joint").value)

        # Section: Command and contact parameters.
        self.default_close_pwm = float(self.get_parameter("control.default_close_pwm").value)
        self.default_hold_pwm = float(self.get_parameter("control.default_hold_pwm").value)
        self.command_period_sec = float(self.get_parameter("control.command_period_sec").value)
        self.min_pwm = float(self.get_parameter("control.min_pwm").value)
        self.max_pwm = float(self.get_parameter("control.max_pwm").value)
        self.max_close_pwm_magnitude = float(
            self.get_parameter("control.max_close_pwm_magnitude").value
        )
        self.command_mode = str(self.get_parameter("control.command_mode").value).lower()
        self.expected_operating_mode = int(
            self.get_parameter("control.expected_operating_mode").value
        )
        self.required_samples = int(self.get_parameter("contact.required_samples").value)
        self.velocity_threshold = float(
            self.get_parameter("contact.velocity_threshold").value
        )
        self.position_span_threshold = float(
            self.get_parameter("contact.position_span_threshold").value
        )
        self.load_delta_threshold = float(
            self.get_parameter("contact.load_delta_threshold").value
        )
        self.require_load_confirmation = bool(
            self.get_parameter("contact.require_load_confirmation").value
        )
        self.aperture_margin = float(self.get_parameter("contact.aperture_margin").value)
        self.history_length = int(self.get_parameter("contact.history_length").value)
        self.empty_closed_aperture = float(
            self.get_parameter("calibration.empty_closed_aperture").value
        )
        self.feedback_timeout_sec = float(
            self.get_parameter("safety.feedback_timeout_sec").value
        )
        self.action_timeout_sec = float(self.get_parameter("safety.action_timeout_sec").value)
        self.stop_pwm = float(self.get_parameter("safety.stop_pwm").value)
        self.health_enabled = bool(self.get_parameter("health.enabled").value)
        self.health_fast_update_rate_hz = float(
            self.get_parameter("health.fast_update_rate_hz").value
        )
        self.health_medium_update_rate_hz = float(
            self.get_parameter("health.medium_update_rate_hz").value
        )
        self.health_slow_update_rate_hz = float(
            self.get_parameter("health.slow_update_rate_hz").value
        )
        self.critical_status_timeout_sec = float(
            self.get_parameter("health.critical_status_timeout_sec").value
        )
        if (
            self.command_mode != "pwm"
            or self.expected_operating_mode != phase.PWM_OPERATING_MODE
        ):
            raise ValueError("gripper guard supports only verified DYNAMIXEL PWM mode (16)")

    def _joint_state_callback(self, message: JointState) -> None:
        """Extract motor feedback by name without using synthesized finger effort or velocity."""
        names = {name: index for index, name in enumerate(message.name)}
        if self.motor_joint not in names or self.finger_joint not in names:
            return
        motor_index = names[self.motor_joint]
        finger_index = names[self.finger_joint]
        if motor_index >= len(message.position) or finger_index >= len(message.position):
            return

        # Section: Missing/non-finite motion feedback is unsafe and must age out the watchdog.
        if motor_index >= len(message.velocity):
            return
        motor_velocity = float(message.velocity[motor_index])
        motor_effort = (
            float(message.effort[motor_index]) if motor_index < len(message.effort) else 0.0
        )
        right_position = 0.0
        right_index = names.get(self.right_finger_joint)
        if right_index is not None and right_index < len(message.position):
            right_position = float(message.position[right_index])

        sample_values = (
            float(message.position[motor_index]),
            motor_velocity,
            float(message.position[finger_index]),
            motor_effort,
        )
        if not all(math.isfinite(value) for value in sample_values):
            return

        now = self._now_seconds()
        with self._feedback_lock:
            self._feedback = GripperFeedback(
                timestamp=now,
                motor_position=float(message.position[motor_index]),
                motor_velocity=motor_velocity,
                motor_effort=motor_effort,
                finger_aperture=float(message.position[finger_index]),
                right_finger_position=right_position,
                feedback_valid=True,
                sequence=self._feedback.sequence + 1,
            )
            self._load_history.append(motor_effort)
        if not self._feedback_announced:
            self._feedback_announced = True
            self.get_logger().info("Gripper feedback available")

    def _goal_callback(self, goal_request: CloseUntilContact.Goal) -> GoalResponse:
        """Validate safety prerequisites and reserve the single gripper command owner."""
        with self._goal_lock:
            if self._goal_reserved:
                return GoalResponse.REJECT
            snapshot = self._feedback_snapshot()
            health = self.health_monitor.snapshot()
            health_valid = (
                not self.health_enabled
                or (
                    health.critical_status_fresh(
                        self._now_seconds(), self.critical_status_timeout_sec
                    )
                    and not health.hardware_fault
                    and health.operating_mode_matches(self.expected_operating_mode)
                )
            )
            valid = (
                snapshot.feedback_valid
                and self._now_seconds() - snapshot.timestamp <= self.feedback_timeout_sec
                and math.isfinite(goal_request.close_pwm)
                and math.isfinite(goal_request.hold_pwm)
                and math.isfinite(goal_request.timeout_sec)
                and goal_request.close_pwm < 0.0
                and goal_request.timeout_sec > 0.0
                and health_valid
            )
            if not valid:
                self.get_logger().warn(
                    "Rejected CloseUntilContact goal: feedback/direction/timeout invalid, "
                    f"or health contract not ready (error={health.hardware_error_status}, "
                    f"mode={health.operating_mode}, expected_mode={self.expected_operating_mode})"
                )
                return GoalResponse.REJECT
            self._pwm_mode_verified_once = True
            self._goal_reserved = True
        self.get_logger().info("CloseUntilContact goal accepted")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        """Accept cancellation so the execute loop can issue a stop command."""
        return CancelResponse.ACCEPT

    def _execute_close(self, goal_handle) -> CloseUntilContact.Result:
        """Run the feedback-fresh, cancellable guarded-close control loop."""
        detector = ContactDetector(
            self.required_samples,
            self.velocity_threshold,
            self.position_span_threshold,
            self.load_delta_threshold,
            self.require_load_confirmation,
            self.empty_closed_aperture,
            self.aperture_margin,
            self.history_length,
        )
        with self._feedback_lock:
            baseline_samples = list(self._load_history)
        detector.reset(baseline_samples)
        self._load_baseline = detector.baseline_load
        guard = GuardStateMachine()
        success = False
        contact_detected = False
        reason = phase.REASON_COMMAND_FAILURE
        leave_hold_active = False
        last_sequence = -1
        timeout_sec = min(float(goal_handle.request.timeout_sec), self.action_timeout_sec)
        close_pwm = self._bounded_close_pwm(float(goal_handle.request.close_pwm))
        hold_pwm = self._bounded_hold_pwm(float(goal_handle.request.hold_pwm), close_pwm)
        start_time = self._now_seconds()
        next_command_time = start_time + self.command_period_sec

        try:
            guard.transition(phase.CLOSING)
            self._phase = guard.state
            self.contact_publisher.publish(Bool(data=False))
            self.get_logger().info("Closing gripper")
            self._enable_closing_output()
            self.start_close(close_pwm)

            while rclpy.ok():
                now = self._now_seconds()
                snapshot = self._feedback_snapshot()
                health = self.health_monitor.snapshot()

                # Section: Cancellation, stale feedback, faults, and timeout always stop closing.
                if goal_handle.is_cancel_requested:
                    guard.transition(phase.CANCELLED)
                    self._phase = guard.state
                    reason = phase.REASON_CANCELLED
                    self.get_logger().info("Goal cancelled")
                    self.stop_gripper()
                    goal_handle.canceled()
                    break
                if now - snapshot.timestamp > self.feedback_timeout_sec:
                    guard.transition(phase.FAULT)
                    self._phase = guard.state
                    reason = phase.REASON_FEEDBACK_TIMEOUT
                    self.get_logger().error("Joint feedback timeout")
                    self.stop_gripper()
                    goal_handle.abort()
                    break
                if self.health_enabled and health.hardware_fault:
                    guard.transition(phase.FAULT)
                    self._phase = guard.state
                    reason = phase.REASON_HARDWARE_FAULT
                    self.get_logger().error(
                        "Hardware fault: "
                        f"status={health.hardware_error_status}, command={self._commanded_pwm}, "
                        f"position={snapshot.motor_position:.5f}, "
                        f"velocity={snapshot.motor_velocity:.5f}, "
                        f"load={health.present_load}, pwm={health.present_pwm}"
                    )
                    self.stop_gripper()
                    goal_handle.abort()
                    break
                if self.health_enabled and not health.critical_status_fresh(
                    now, self.critical_status_timeout_sec
                ):
                    guard.transition(phase.FAULT)
                    self._phase = guard.state
                    reason = phase.REASON_HEALTH_TIMEOUT
                    self.get_logger().error("Critical Hardware_Error_Status feedback is stale")
                    self.stop_gripper()
                    goal_handle.abort()
                    break
                if self.health_enabled and not health.operating_mode_matches(
                    self.expected_operating_mode
                ):
                    guard.transition(phase.FAULT)
                    self._phase = guard.state
                    reason = phase.REASON_MODE_MISMATCH
                    self.get_logger().error(
                        f"Operating_Mode changed to {health.operating_mode}; expected PWM "
                        f"mode {self.expected_operating_mode}"
                    )
                    self._suppress_output_for_mode_mismatch()
                    goal_handle.abort()
                    break
                if now - start_time >= timeout_sec:
                    guard.transition(phase.TIMEOUT)
                    self._phase = guard.state
                    reason = phase.REASON_CONTACT_TIMEOUT
                    self.get_logger().warn("Contact timeout")
                    self.stop_gripper()
                    goal_handle.abort()
                    break

                # Section: Evaluate each JointState sample once so persistence is sample-based.
                if snapshot.sequence != last_sequence:
                    last_sequence = snapshot.sequence
                    observation = detector.update(
                        snapshot.motor_position,
                        snapshot.motor_velocity,
                        snapshot.finger_aperture,
                        snapshot.motor_effort,
                        True,
                        snapshot.timestamp,
                    )
                    self._last_observation = observation
                    if observation.contact_candidate and guard.state == phase.CLOSING:
                        guard.transition(phase.CONTACT_CANDIDATE)
                        self._phase = guard.state
                        self.get_logger().info("Contact candidate detected")
                    elif (
                        not observation.contact_candidate
                        and guard.state == phase.CONTACT_CANDIDATE
                    ):
                        guard.transition(phase.CLOSING)
                        self._phase = guard.state

                    if observation.empty_close_detected:
                        guard.transition(phase.EMPTY_CLOSED)
                        self._phase = guard.state
                        reason = phase.REASON_EMPTY_CLOSE
                        self.get_logger().warn(
                            "Empty mechanical close: "
                            f"position={snapshot.motor_position:.5f}, "
                            f"velocity={snapshot.motor_velocity:.5f}, "
                            f"finger={snapshot.finger_aperture:.5f} m, "
                            f"span={observation.position_span:.5f}, "
                            f"load_delta={observation.load_delta:.3f}"
                        )
                        self.stop_gripper()
                        goal_handle.abort()
                        break
                    if observation.contact_confirmed:
                        guard.transition(phase.CONTACT_CONFIRMED)
                        self._phase = guard.state
                        contact_detected = True
                        self.get_logger().info(
                            "Contact confirmed: "
                            f"position={snapshot.motor_position:.5f}, "
                            f"velocity={snapshot.motor_velocity:.5f}, "
                            f"finger={snapshot.finger_aperture:.5f} m, "
                            f"span={observation.position_span:.5f}, "
                            f"load_delta={observation.load_delta:.3f}, "
                            f"samples={observation.stable_samples}, "
                            f"elapsed={now - start_time:.3f} s"
                        )
                        if hold_pwm != self.stop_pwm:
                            guard.transition(phase.HOLDING)
                            self._phase = guard.state
                            self.get_logger().info("Applying hold command")
                            self.start_hold(hold_pwm)
                            leave_hold_active = True
                        else:
                            guard.transition(phase.STOPPING)
                            self._phase = guard.state
                            self.stop_gripper()
                        success = True
                        reason = phase.REASON_CONTACT_CONFIRMED
                        goal_handle.succeed()
                        self.get_logger().info("Goal completed")
                        break

                if now >= next_command_time:
                    self.start_close(close_pwm)
                    next_command_time = now + self.command_period_sec
                self._publish_action_feedback(goal_handle, snapshot, detector.stable_samples)
                time.sleep(min(0.01, self.command_period_sec))
        except Exception as exc:
            self.get_logger().error(f"Guarded close exception: {exc}")
            reason = phase.REASON_COMMAND_FAILURE
            try:
                self.stop_gripper()
            finally:
                goal_handle.abort()
        finally:
            # Section: Never retain closing PWM after a failure, cancellation, or exception.
            if not success or not leave_hold_active:
                self.stop_gripper()
            with self._goal_lock:
                self._goal_reserved = False
            guard.reset()
            self._phase = phase.IDLE
            self.contact_publisher.publish(Bool(data=contact_detected))

        return self._make_result(success, contact_detected, reason)

    def send_gripper_pwm(self, value: float) -> float:
        """Validate, clamp, and publish one Interbotix gripper PWM command."""
        if not math.isfinite(value):
            raise ValueError("gripper PWM must be finite")
        bounded = min(max(float(value), self.min_pwm), self.max_pwm)
        if bounded != value:
            self.get_logger().warn(f"Clamped gripper PWM from {value} to {bounded}")
        message = JointSingleCommand()
        message.name = self.motor_joint
        message.cmd = float(bounded)
        self.command_publisher.publish(message)
        self._commanded_pwm = bounded
        return bounded

    def start_close(self, value: float) -> float:
        """Publish a closing command no stronger than the deployed grasp magnitude."""
        if not self._close_commands_enabled:
            self.get_logger().warn("Suppressed closing PWM after terminal stop")
            return self._commanded_pwm
        return self.send_gripper_pwm(self._bounded_close_pwm(value))

    def start_hold(self, value: float) -> float:
        """Publish a non-escalating optional hold command."""
        self._close_commands_enabled = False
        self._stop_published = False
        return self.send_gripper_pwm(value)

    def stop_gripper(self) -> float:
        """Latch closing off and publish PWM zero exactly once per active output."""
        self._close_commands_enabled = False
        if self._stop_published:
            return self.stop_pwm
        if not self._pwm_mode_verified_once:
            self.get_logger().error("Suppressed stop because PWM mode was never verified")
            return self._commanded_pwm
        self.get_logger().info("Stopping gripper")
        stopped_value = self.send_gripper_pwm(self.stop_pwm)
        self._stop_published = True
        return stopped_value

    def _enable_closing_output(self) -> None:
        """Arm the close-output latch only after goal and PWM-mode validation."""
        if self.health_enabled and not self._pwm_mode_verified_once:
            raise RuntimeError("cannot enable gripper output before PWM-mode verification")
        self._close_commands_enabled = True
        self._stop_published = False

    def _suppress_output_for_mode_mismatch(self) -> None:
        """Stop publishing entirely when PWM-zero semantics are no longer valid."""
        self._close_commands_enabled = False
        self._stop_published = True
        self._pwm_mode_verified_once = False
        self.get_logger().error(
            "All gripper output suppressed; PWM-zero was not sent in a non-PWM mode"
        )

    def _bounded_close_pwm(self, value: float) -> float:
        """Preserve negative-close direction and cap magnitude at the current safe value."""
        if not math.isfinite(value) or value >= 0.0:
            raise ValueError("close PWM must be finite and negative")
        return max(float(value), -abs(self.max_close_pwm_magnitude))

    def _bounded_hold_pwm(self, value: float, close_pwm: float) -> float:
        """Prevent hold PWM from reversing or exceeding the active close command."""
        if not math.isfinite(value):
            raise ValueError("hold PWM must be finite")
        if value > 0.0:
            self.get_logger().warn("Positive hold PWM rejected; using stop PWM")
            return self.stop_pwm
        return max(float(value), float(close_pwm))

    def _publish_action_feedback(self, goal_handle, snapshot, stable_samples: int) -> None:
        """Publish bounded-rate action telemetry from the latest valid feedback sample."""
        feedback = CloseUntilContact.Feedback()
        feedback.phase = self._phase
        feedback.motor_position = snapshot.motor_position
        feedback.finger_aperture = snapshot.finger_aperture
        feedback.velocity = snapshot.motor_velocity
        feedback.load = snapshot.motor_effort
        feedback.stable_samples = int(stable_samples)
        goal_handle.publish_feedback(feedback)

    def _make_result(self, success: bool, contact: bool, reason: str):
        """Populate the action result from the latest gripper motor and finger sample."""
        snapshot = self._feedback_snapshot()
        result = CloseUntilContact.Result()
        result.success = bool(success)
        result.contact_detected = bool(contact)
        result.reason = reason
        result.final_motor_position = snapshot.motor_position
        result.final_finger_aperture = snapshot.finger_aperture
        result.final_velocity = snapshot.motor_velocity
        result.final_load = snapshot.motor_effort
        return result

    def _feedback_snapshot(self) -> GripperFeedback:
        """Return an immutable-by-convention copy for concurrent action evaluation."""
        with self._feedback_lock:
            return GripperFeedback(**vars(self._feedback))

    def _publish_diagnostics(self) -> None:
        """Publish combined feedback, contact, command, and low-rate health diagnostics."""
        snapshot = self._feedback_snapshot()
        health = self.health_monitor.snapshot()
        now = self._now_seconds()
        feedback_age = now - snapshot.timestamp if snapshot.feedback_valid else float("inf")
        status = DiagnosticStatus()
        status.name = "RX200 Gripper Guard"
        status.hardware_id = self.motor_joint
        if not snapshot.feedback_valid or feedback_age > self.feedback_timeout_sec:
            status.level = DiagnosticStatus.STALE
            status.message = "joint feedback unavailable or stale"
        elif health.hardware_fault:
            status.level = DiagnosticStatus.ERROR
            status.message = "gripper hardware error"
        elif self.health_enabled and not health.operating_mode_matches(
            self.expected_operating_mode
        ):
            status.level = DiagnosticStatus.ERROR
            status.message = "gripper operating mode is not verified PWM"
        elif self.health_enabled and not health.critical_status_fresh(
            now, self.critical_status_timeout_sec
        ):
            status.level = DiagnosticStatus.STALE
            status.message = "critical hardware status is stale"
        elif self.health_enabled and not health.register_service_available:
            status.level = DiagnosticStatus.WARN
            status.message = "register diagnostics unavailable"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "guard ready"

        observation = self._last_observation
        values = {
            "state": self._phase,
            "feedback_age": feedback_age,
            "contact_detected": observation.contact_confirmed,
            "contact_candidate": observation.contact_candidate,
            "stable_samples": observation.stable_samples,
            "motor_position": snapshot.motor_position,
            "motor_velocity": snapshot.motor_velocity,
            "finger_aperture": snapshot.finger_aperture,
            "load": snapshot.motor_effort,
            "load_baseline": self._load_baseline,
            "load_delta": observation.load_delta,
            "commanded_pwm": self._commanded_pwm,
            "present_load": health.present_load,
            "present_pwm": health.present_pwm,
            "temperature": health.temperature,
            "input_voltage": health.input_voltage,
            "hardware_error_status": health.hardware_error_status,
            "hardware_error_age": (
                now - health.hardware_error_timestamp
                if health.hardware_error_timestamp > 0.0
                else float("inf")
            ),
            "operating_mode": health.operating_mode,
            "expected_operating_mode": self.expected_operating_mode,
            "register_service_available": health.register_service_available,
        }
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_publisher.publish(array)

    def _now_seconds(self) -> float:
        """Use the node's ROS clock consistently for feedback age and action timeout."""
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def _empty_observation() -> ContactObservation:
        """Create a neutral diagnostic observation before the first guarded close."""
        return ContactObservation(False, False, False, False, False, 0, 0, 0.0, 0.0)


def main(args=None) -> None:
    """Run the guard with enough executor threads to keep feedback non-blocking."""
    rclpy.init(args=args)
    node = GripperGuardNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Section: Stop only an output that was previously armed in verified PWM mode.
        node.stop_gripper()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
