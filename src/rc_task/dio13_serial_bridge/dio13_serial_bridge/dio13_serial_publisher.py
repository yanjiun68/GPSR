#!/usr/bin/env python3
"""
ROS 2 serial bridge for Arduino DIO13 reporting.

Expected Arduino serial lines:
    DIO13=HIGH
    DIO13=LOW

This node publishes:
    /dio13_state      -> std_msgs/Bool
    /dio13_state_text -> std_msgs/String
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

import serial
from serial import SerialException


class DIO13SerialPublisher(Node):
    """Read Arduino serial text and publish DIO13 state as ROS 2 topics."""

    def __init__(self) -> None:
        super().__init__('dio13_serial_publisher')

        # Parameters so beginners can change settings without editing the code.
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('read_period_sec', 0.02)
        self.declare_parameter('topic_name', '/dio13_state')
        self.declare_parameter('text_topic_name', '/dio13_state_text')
        self.declare_parameter('publish_only_on_change', False)
        self.declare_parameter('reconnect_period_sec', 2.0)

        self.port = str(self.get_parameter('port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.read_period_sec = float(self.get_parameter('read_period_sec').value)
        self.topic_name = str(self.get_parameter('topic_name').value)
        self.text_topic_name = str(self.get_parameter('text_topic_name').value)
        self.publish_only_on_change = bool(self.get_parameter('publish_only_on_change').value)
        self.reconnect_period_sec = float(self.get_parameter('reconnect_period_sec').value)

        self.bool_publisher = self.create_publisher(Bool, self.topic_name, 10)
        self.text_publisher = self.create_publisher(String, self.text_topic_name, 10)

        self.serial_connection: Optional[serial.Serial] = None
        self.last_state: Optional[bool] = None
        self.last_connect_attempt_ns = 0

        # Timer repeatedly polls the serial port.
        self.timer = self.create_timer(self.read_period_sec, self._timer_callback)
        self._connect_serial()

    def _connect_serial(self) -> None:
        """Open the serial port. If it fails, the node will retry later."""
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.0,
            )
            self.get_logger().info(
                f'Connected to Arduino on {self.port} at {self.baud_rate} baud.'
            )
        except SerialException as exc:
            self.serial_connection = None
            self.get_logger().warning(
                f'Could not open serial port {self.port}: {exc}. '
                f'Will retry every {self.reconnect_period_sec:.1f} seconds.'
            )

    def _timer_callback(self) -> None:
        """Read available serial lines and convert them into ROS messages."""
        if self.serial_connection is None or not self.serial_connection.is_open:
            self._retry_connection_if_needed()
            return

        try:
            while self.serial_connection.in_waiting > 0:
                raw_line = self.serial_connection.readline().decode(
                    'utf-8', errors='ignore'
                ).strip()
                if raw_line:
                    self._handle_line(raw_line)
        except (SerialException, OSError) as exc:
            self.get_logger().error(f'Serial read failed: {exc}')
            self._close_serial()

    def _retry_connection_if_needed(self) -> None:
        """Retry serial connection after the configured wait time."""
        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = (now_ns - self.last_connect_attempt_ns) / 1e9
        if elapsed_sec < self.reconnect_period_sec:
            return
        self.last_connect_attempt_ns = now_ns
        self._connect_serial()

    def _handle_line(self, raw_line: str) -> None:
        """Parse a single serial line from Arduino."""
        # Ignore unrelated debug text from the Arduino.
        if not raw_line.startswith('DIO13='):
            return

        payload = raw_line.split('=', 1)[1].strip().upper()

        if payload == 'HIGH':
            state = True
        elif payload == 'LOW':
            state = False
        else:
            self.get_logger().warning(f'Unexpected DIO13 payload: {raw_line}')
            return

        if (
            self.publish_only_on_change
            and self.last_state is not None
            and state == self.last_state
        ):
            return

        bool_msg = Bool()
        bool_msg.data = state
        self.bool_publisher.publish(bool_msg)

        text_msg = String()
        text_msg.data = payload
        self.text_publisher.publish(text_msg)

        self.last_state = state

    def _close_serial(self) -> None:
        """Safely close the serial port after an error."""
        try:
            if self.serial_connection is not None and self.serial_connection.is_open:
                self.serial_connection.close()
        finally:
            self.serial_connection = None

    def destroy_node(self) -> bool:
        """Close serial before shutting down the node."""
        self._close_serial()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """ROS 2 entry point."""
    rclpy.init(args=args)
    node = DIO13SerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
