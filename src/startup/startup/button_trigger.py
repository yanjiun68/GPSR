#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

TOPIC_NAME = "/dio13_state"


class ButtonTrigger(Node):
    def __init__(self):
        super().__init__('button_trigger_node')
        self.create_subscription(
            Bool,
            TOPIC_NAME,
            callback=self.topic_callback,
            qos_profile=10,
        )
        self.get_logger().info(f'Waiting for {TOPIC_NAME} content = True')

    def topic_callback(self, content: Bool):
        if content.data:
            self.get_logger().info(f'{TOPIC_NAME} is True; starting state machine.')
            raise SystemExit


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ButtonTrigger()
    # try:
    rclpy.spin(node)
    # except (KeyboardInterrupt, ExternalShutdownException):
    #     pass
    # finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
