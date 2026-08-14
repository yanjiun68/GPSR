import rclpy
from rclpy.node import Node

from test_tools.message_utils import load_message_class
from test_tools.message_utils import make_qos_profile
from test_tools.message_utils import message_from_yaml


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        # Override these parameters to choose the topic, message type, and payload.
        self.declare_parameter('topic', '/test_tools/output')
        self.declare_parameter('message_type', 'std_msgs/msg/String')
        self.declare_parameter('message_yaml', "data: 'test message'")
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('publish_once', False)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_durability', 'volatile')

        topic = self.get_parameter('topic').value
        message_type = self.get_parameter('message_type').value
        message_yaml = self.get_parameter('message_yaml').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        publish_once = bool(self.get_parameter('publish_once').value)
        qos_depth = int(self.get_parameter('qos_depth').value)
        qos_reliability = self.get_parameter('qos_reliability').value
        qos_durability = self.get_parameter('qos_durability').value

        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be greater than zero.')

        self.done = False
        self._message = message_from_yaml(load_message_class(message_type), message_yaml)
        qos_profile = make_qos_profile(qos_depth, qos_reliability, qos_durability)
        self._publisher = self.create_publisher(type(self._message), topic, qos_profile)
        self._publish_once = publish_once

        self.get_logger().info(
            f"Publishing {message_type} on '{topic}' at {rate_hz:g} Hz "
            f"with payload: {self._message}"
        )
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self):
        self._publisher.publish(self._message)
        self.get_logger().info(f'Published: {self._message}')

        if self._publish_once:
            self._timer.cancel()
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TestPublisher()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
