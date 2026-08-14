#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KinematicsKeypad(Node):
    def __init__(self):
        super().__init__('kinematics_keypad')

        self.declare_parameter('trigger_topic', '/kinematics_test/trigger')
        self.trigger_topic = self.get_parameter('trigger_topic').value

        self.publisher = self.create_publisher(String, self.trigger_topic, 10)

        self.keymap = {
            'w': ('forward', 'Forward'),
            's': ('backward', 'Backward'),
            'a': ('left', 'Strafe Left'),
            'd': ('right', 'Strafe Right'),
            'q': ('ccw', 'Rotate CCW'),
            'e': ('cw', 'Rotate CW'),
            ' ': ('stop', 'Stop'),
            'h': ('help', 'Help'),
            'x': ('quit', 'Quit'),
        }

        if not sys.stdin.isatty():
            raise RuntimeError('This program must run in a terminal (TTY).')

        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.get_logger().info(f'Publishing trigger commands to {self.trigger_topic}')
        self.print_help()
        self.create_timer(0.05, self.timer_cb)

    def print_help(self):
        self.get_logger().info(
            'Keys: w=forward s=backward a=left d=right q=ccw e=cw space=stop h=help x=quit'
        )

    def publish_command(self, command, label):
        self.publisher.publish(String(data=command))
        self.get_logger().info(f'Sent command: {label}')

    def timer_cb(self):
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return

        key = sys.stdin.read(1)
        if key == '\x03':
            self.publish_command('quit', 'Quit')
            rclpy.shutdown()
            return

        entry = self.keymap.get(key.lower())
        if entry is None:
            return

        command, label = entry
        if command == 'help':
            self.print_help()
            return

        self.publish_command(command, label)
        if command == 'quit':
            rclpy.shutdown()

    def destroy_node(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KinematicsKeypad()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
