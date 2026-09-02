"""Prioritized, non-blocking Interbotix register health monitoring."""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Optional

from interbotix_xs_msgs.srv import RegisterValues
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node


# Section: Interpret every non-zero DYNAMIXEL hardware status as a fault bitmask.
def is_hardware_fault(value: Optional[int]) -> bool:
    """Return true for any non-zero Hardware_Error_Status value or combination."""
    return value is not None and int(value) != 0


@dataclass
class HealthSnapshot:
    """Store the latest independently available gripper health registers."""

    register_service_available: bool = False
    hardware_fault: bool = False
    hardware_error_status: Optional[int] = None
    hardware_error_timestamp: float = 0.0
    operating_mode: Optional[int] = None
    operating_mode_timestamp: float = 0.0
    present_load: Optional[int] = None
    present_pwm: Optional[int] = None
    temperature: Optional[int] = None
    input_voltage: Optional[int] = None
    values: dict[str, Optional[int]] = field(default_factory=dict)

    def critical_status_fresh(self, now: float, timeout_sec: float) -> bool:
        """Require a recent successful fault-register read before guarded motion."""
        return (
            self.hardware_error_status is not None
            and now - self.hardware_error_timestamp <= timeout_sec
        )

    def operating_mode_matches(self, expected_mode: int) -> bool:
        """Verify the runtime DYNAMIXEL mode matches the command contract."""
        return self.operating_mode is not None and self.operating_mode == int(expected_mode)


class HealthMonitor:
    """Poll critical, advisory, and slow registers at separate bounded rates."""

    def __init__(
        self,
        node: Node,
        service_name: str,
        motor_joint: str,
        fast_update_rate_hz: float,
        medium_update_rate_hz: float,
        slow_update_rate_hz: float,
        callback_group: CallbackGroup,
        enabled: bool = True,
    ) -> None:
        """Create one register client and three priority-specific polling timers."""
        self.node = node
        self.motor_joint = motor_joint
        self.enabled = bool(enabled)
        self.client = node.create_client(
            RegisterValues,
            service_name,
            callback_group=callback_group,
        )
        self._lock = threading.Lock()
        self._snapshot = HealthSnapshot()
        self._pending_registers: set[str] = set()
        self._medium_registers = ("Present_Load", "Present_PWM", "Operating_Mode")
        self._slow_registers = ("Present_Temperature", "Present_Input_Voltage")
        self._medium_index = 2  # Section: Read Operating_Mode on the first medium tick.
        self._slow_index = 0

        # Section: Critical status is independent from advisory and thermal telemetry.
        self.fast_timer = node.create_timer(
            1.0 / max(0.1, float(fast_update_rate_hz)),
            self._poll_hardware_error,
            callback_group=callback_group,
        )
        self.medium_timer = node.create_timer(
            1.0 / max(0.1, float(medium_update_rate_hz)),
            self._poll_medium,
            callback_group=callback_group,
        )
        self.slow_timer = node.create_timer(
            1.0 / max(0.01, float(slow_update_rate_hz)),
            self._poll_slow,
            callback_group=callback_group,
        )

    def snapshot(self) -> HealthSnapshot:
        """Return an isolated copy safe for action and diagnostic callbacks."""
        with self._lock:
            return HealthSnapshot(
                register_service_available=self._snapshot.register_service_available,
                hardware_fault=self._snapshot.hardware_fault,
                hardware_error_status=self._snapshot.hardware_error_status,
                hardware_error_timestamp=self._snapshot.hardware_error_timestamp,
                operating_mode=self._snapshot.operating_mode,
                operating_mode_timestamp=self._snapshot.operating_mode_timestamp,
                present_load=self._snapshot.present_load,
                present_pwm=self._snapshot.present_pwm,
                temperature=self._snapshot.temperature,
                input_voltage=self._snapshot.input_voltage,
                values=dict(self._snapshot.values),
            )

    def _poll_hardware_error(self) -> None:
        """Request the critical error bitmask on every fast timer tick."""
        self._request_register("Hardware_Error_Status")

    def _poll_medium(self) -> None:
        """Round-robin estimated load, applied PWM, and operating mode."""
        register = self._medium_registers[self._medium_index]
        self._medium_index = (self._medium_index + 1) % len(self._medium_registers)
        self._request_register(register)

    def _poll_slow(self) -> None:
        """Round-robin temperature and voltage without loading the bus at 10 Hz."""
        register = self._slow_registers[self._slow_index]
        self._slow_index = (self._slow_index + 1) % len(self._slow_registers)
        self._request_register(register)

    def _request_register(self, register: str) -> None:
        """Start one asynchronous request unless the same register is already pending."""
        if not self.enabled:
            return
        if not self.client.service_is_ready():
            with self._lock:
                self._snapshot.register_service_available = False
            return
        with self._lock:
            if register in self._pending_registers:
                return
            self._pending_registers.add(register)

        request = RegisterValues.Request()
        request.cmd_type = "single"
        request.name = self.motor_joint
        request.reg = register
        request.value = 0
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda completed, requested_register=register: self._handle_result(
                requested_register, completed
            )
        )

    def _handle_result(self, register: str, future) -> None:
        """Record one register result while isolating service and register failures."""
        value: Optional[int] = None
        response_valid = False
        try:
            response = future.result()
            if response is not None and response.values:
                value = int(response.values[0])
                response_valid = True
        except Exception as exc:  # Section: Diagnostics must survive driver/service errors.
            self.node.get_logger().debug(f"Register read failed for {register}: {exc}")

        now = self.node.get_clock().now().nanoseconds / 1_000_000_000.0
        with self._lock:
            self._pending_registers.discard(register)
            self._snapshot.register_service_available = response_valid
            self._snapshot.values[register] = value
            if register == "Hardware_Error_Status":
                self._snapshot.hardware_error_status = value
                self._snapshot.hardware_fault = is_hardware_fault(value)
                if response_valid:
                    self._snapshot.hardware_error_timestamp = now
            elif register == "Operating_Mode":
                self._snapshot.operating_mode = value
                if response_valid:
                    self._snapshot.operating_mode_timestamp = now
            elif register == "Present_Load":
                self._snapshot.present_load = value
            elif register == "Present_PWM":
                self._snapshot.present_pwm = value
            elif register == "Present_Temperature":
                self._snapshot.temperature = value
            elif register == "Present_Input_Voltage":
                self._snapshot.input_voltage = value
