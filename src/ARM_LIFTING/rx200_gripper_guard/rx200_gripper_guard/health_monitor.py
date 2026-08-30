"""Low-rate, non-blocking Interbotix register health monitor."""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Optional

from interbotix_xs_msgs.srv import RegisterValues
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node


@dataclass
class HealthSnapshot:
    """Store the latest independently available gripper health registers."""

    register_service_available: bool = False
    hardware_fault: bool = False
    hardware_error_status: Optional[int] = None
    present_load: Optional[int] = None
    present_pwm: Optional[int] = None
    temperature: Optional[int] = None
    input_voltage: Optional[int] = None
    values: dict[str, Optional[int]] = field(default_factory=dict)


class HealthMonitor:
    """Poll supported diagnostic registers one at a time without blocking callbacks."""

    def __init__(
        self,
        node: Node,
        service_name: str,
        motor_joint: str,
        update_rate_hz: float,
        callback_group: CallbackGroup,
        enabled: bool = True,
    ) -> None:
        """Create the register client and bounded low-frequency polling timer."""
        self.node = node
        self.motor_joint = motor_joint
        self.enabled = bool(enabled)
        self.client = node.create_client(
            RegisterValues,
            service_name,
            callback_group=callback_group,
        )
        self.registers = (
            "Hardware_Error_Status",
            "Present_Load",
            "Present_PWM",
            "Present_Temperature",
            "Present_Input_Voltage",
        )
        self._lock = threading.Lock()
        self._snapshot = HealthSnapshot()
        self._register_index = 0
        self._request_pending = False
        period = 1.0 / max(0.1, float(update_rate_hz))
        self.timer = node.create_timer(period, self._poll_next, callback_group=callback_group)

    def snapshot(self) -> HealthSnapshot:
        """Return an isolated copy safe for action and diagnostic callbacks."""
        with self._lock:
            return HealthSnapshot(
                register_service_available=self._snapshot.register_service_available,
                hardware_fault=self._snapshot.hardware_fault,
                hardware_error_status=self._snapshot.hardware_error_status,
                present_load=self._snapshot.present_load,
                present_pwm=self._snapshot.present_pwm,
                temperature=self._snapshot.temperature,
                input_voltage=self._snapshot.input_voltage,
                values=dict(self._snapshot.values),
            )

    def _poll_next(self) -> None:
        """Start at most one asynchronous register request per timer tick."""
        if not self.enabled:
            return
        if self._request_pending:
            return
        if not self.client.service_is_ready():
            with self._lock:
                self._snapshot.register_service_available = False
            return

        register = self.registers[self._register_index]
        self._register_index = (self._register_index + 1) % len(self.registers)
        request = RegisterValues.Request()
        request.cmd_type = "single"
        request.name = self.motor_joint
        request.reg = register
        request.value = 0
        self._request_pending = True
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda completed, requested_register=register: self._handle_result(
                requested_register, completed
            )
        )

    def _handle_result(self, register: str, future) -> None:
        """Record one register result while isolating unsupported-register failures."""
        value: Optional[int] = None
        try:
            response = future.result()
            if response is not None and response.values:
                value = int(response.values[0])
        except Exception as exc:  # Section: Diagnostics must survive driver/service errors.
            self.node.get_logger().debug(f"Register read failed for {register}: {exc}")

        with self._lock:
            self._request_pending = False
            self._snapshot.register_service_available = True
            self._snapshot.values[register] = value
            if register == "Hardware_Error_Status":
                self._snapshot.hardware_error_status = value
                self._snapshot.hardware_fault = value is not None and value != 0
            elif register == "Present_Load":
                self._snapshot.present_load = value
            elif register == "Present_PWM":
                self._snapshot.present_pwm = value
            elif register == "Present_Temperature":
                self._snapshot.temperature = value
            elif register == "Present_Input_Voltage":
                self._snapshot.input_voltage = value
