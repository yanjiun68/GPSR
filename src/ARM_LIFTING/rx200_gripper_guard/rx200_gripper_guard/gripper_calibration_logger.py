"""CSV logger for conservative offline RX200 gripper threshold calibration."""

from __future__ import annotations

import csv
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from interbotix_xs_msgs.msg import JointSingleCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperCalibrationLogger(Node):
    """Record feedback and guard evidence without commanding or tuning the gripper."""

    def __init__(self) -> None:
        """Create read-only subscriptions and initialize the requested CSV output."""
        super().__init__("rx200_gripper_calibration_logger")
        self.declare_parameter("output_csv", "/tmp/rx200_gripper_calibration.csv")
        self.declare_parameter("joint_states_topic", "/rx200/sdk_joint_states")
        self.declare_parameter("command_topic", "/rx200/commands/joint_single")
        self.declare_parameter("diagnostics_topic", "/rx200/gripper_guard/diagnostics")
        self.declare_parameter("motor_joint", "gripper")
        self.declare_parameter("finger_joint", "left_finger")
        self.motor_joint = str(self.get_parameter("motor_joint").value)
        self.finger_joint = str(self.get_parameter("finger_joint").value)
        output_path = Path(str(self.get_parameter("output_csv").value))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = output_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._file,
            fieldnames=[
                "timestamp",
                "motor_position",
                "motor_velocity",
                "motor_effort_load",
                "finger_aperture",
                "commanded_pwm",
                "contact_candidate",
                "stable_samples",
            ],
        )
        self._writer.writeheader()
        self._commanded_pwm = 0.0
        self._diagnostics: dict[str, str] = {}
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_callback,
            10,
        )
        self.create_subscription(
            JointSingleCommand,
            str(self.get_parameter("command_topic").value),
            self._command_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            self._diagnostic_callback,
            10,
        )
        self.get_logger().info(f"Writing calibration samples to {output_path}")

    def _command_callback(self, message: JointSingleCommand) -> None:
        """Track the latest observed command without publishing any command."""
        if message.name == self.motor_joint:
            self._commanded_pwm = float(message.cmd)

    def _diagnostic_callback(self, message: DiagnosticArray) -> None:
        """Cache contact evidence published by the authoritative guard node."""
        if message.status:
            self._diagnostics = {value.key: value.value for value in message.status[0].values}

    def _joint_callback(self, message: JointState) -> None:
        """Write one row from named motor and finger feedback fields."""
        if self.motor_joint not in message.name or self.finger_joint not in message.name:
            return
        motor_index = message.name.index(self.motor_joint)
        finger_index = message.name.index(self.finger_joint)
        if motor_index >= len(message.position) or finger_index >= len(message.position):
            return
        row = {
            "timestamp": self.get_clock().now().nanoseconds / 1_000_000_000.0,
            "motor_position": message.position[motor_index],
            "motor_velocity": (
                message.velocity[motor_index] if motor_index < len(message.velocity) else 0.0
            ),
            "motor_effort_load": (
                message.effort[motor_index] if motor_index < len(message.effort) else 0.0
            ),
            "finger_aperture": message.position[finger_index],
            "commanded_pwm": self._commanded_pwm,
            "contact_candidate": self._diagnostics.get("contact_candidate", "False"),
            "stable_samples": self._diagnostics.get("stable_samples", "0"),
        }
        self._writer.writerow(row)
        self._file.flush()

    def destroy_node(self):
        """Flush and close the CSV before releasing ROS resources."""
        self._file.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the read-only calibration logger."""
    rclpy.init(args=args)
    node = GripperCalibrationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
