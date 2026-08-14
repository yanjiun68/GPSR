#!/usr/bin/env python3
"""
Beginner-friendly example consumer node for the Arduino DIO13 serial bridge.

This node subscribes to /dio13_state and does three useful things:
1. Logs every state transition clearly.
2. Republishes a human-readable event string.
3. Triggers a simple software latch when DIO13 becomes HIGH.

You can later replace the latch section with:
- a service call
- an action goal
- a state machine event
- a safety stop request
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class DIO13MonitorNode(Node):
    """Monitor /dio13_state and generate easy-to-use software events."""

    def __init__(self) -> None:
        super().__init__('dio13_monitor_node')

        # Parameters let you reuse the node without editing source code.
        self.declare_parameter('input_topic', '/dio13_state')
        self.declare_parameter('event_topic', '/dio13_event')
        self.declare_parameter('latch_high_event', True)
        self.declare_parameter('periodic_status_sec', 2.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.event_topic = self.get_parameter('event_topic').value
        self.latch_high_event = self.get_parameter('latch_high_event').value
        self.periodic_status_sec = float(self.get_parameter('periodic_status_sec').value)

        # Publisher for readable software events.
        self.event_pub = self.create_publisher(String, self.event_topic, 10)

        # Subscriber to the Bool topic produced by the serial bridge node.
        self.subscription = self.create_subscription(
            Bool,
            self.input_topic,
            self._dio13_callback,
            10,
        )

        # Timer used only for periodic debug/status printouts.
        self.status_timer = self.create_timer(self.periodic_status_sec, self._status_timer_callback)

        # Internal state variables.
        self.last_state: bool | None = None
        self.high_event_latched = False
        self.message_count = 0

        self.get_logger().info(
            f'DIO13 monitor started. Subscribing to {self.input_topic} and publishing events to {self.event_topic}.'
        )

    def _dio13_callback(self, msg: Bool) -> None:
        """Handle each incoming DIO13 state message."""
        current_state = bool(msg.data)
        self.message_count += 1

        # First message case.
        if self.last_state is None:
            self.last_state = current_state
            self._publish_event('INITIAL_HIGH' if current_state else 'INITIAL_LOW')
            self.get_logger().info(f'Initial DIO13 state received: {current_state}')
            if current_state:
                self._handle_high_event()
            return

        # Transition detection.
        if current_state != self.last_state:
            if current_state:
                self.get_logger().info('DIO13 transition detected: LOW -> HIGH')
                self._publish_event('RISING_EDGE')
                self._handle_high_event()
            else:
                self.get_logger().info('DIO13 transition detected: HIGH -> LOW')
                self._publish_event('FALLING_EDGE')

                # When signal goes low, clear the latch so the next HIGH can trigger again.
                if self.latch_high_event and self.high_event_latched:
                    self.high_event_latched = False
                    self.get_logger().info('HIGH-event latch cleared because input returned LOW.')

        self.last_state = current_state

    def _handle_high_event(self) -> None:
        """Example place to trigger robot behavior when DIO13 becomes HIGH."""
        if self.latch_high_event:
            if self.high_event_latched:
                self.get_logger().info('HIGH detected again, but event is latched so no repeat trigger is sent.')
                return
            self.high_event_latched = True

        # Replace this section later with your actual robot action.
        self.get_logger().warn('DIO13 HIGH event: this is where you can trigger homing, stop, or a state-machine event.')
        self._publish_event('HIGH_TRIGGERED')

    def _publish_event(self, text: str) -> None:
        """Publish a readable software event string."""
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)

    def _status_timer_callback(self) -> None:
        """Periodic health/debug status message."""
        self.get_logger().info(
            f'Status: received_messages={self.message_count}, '
            f'last_state={self.last_state}, '
            f'high_event_latched={self.high_event_latched}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DIO13MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
