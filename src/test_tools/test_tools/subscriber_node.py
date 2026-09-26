import rclpy
from rclpy.node import Node

from test_tools.message_utils import load_message_class
from test_tools.message_utils import make_qos_profile


class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')

        # Override these lists to choose which topics and message types to inspect.
        self.declare_parameter('topics', ['/test_tools/output'])
        self.declare_parameter('message_types', ['std_msgs/msg/String'])
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_durability', 'volatile')

        topics = list(self.get_parameter('topics').value)
        message_types = list(self.get_parameter('message_types').value)
        qos_depth = int(self.get_parameter('qos_depth').value)
        qos_reliability = self.get_parameter('qos_reliability').value
        qos_durability = self.get_parameter('qos_durability').value

        if not topics:
            raise ValueError('topics must contain at least one topic name.')
        if len(topics) != len(message_types):
            raise ValueError('topics and message_types must have the same number of entries.')

        qos_profile = make_qos_profile(qos_depth, qos_reliability, qos_durability)
        self._subscriptions = []

        for topic, message_type in zip(topics, message_types):
            message_class = load_message_class(message_type)
            subscription = self.create_subscription(
                message_class,
                topic,
                self._make_callback(topic),
                qos_profile,
            )
            self._subscriptions.append(subscription)
            self.get_logger().info(f"Subscribed to '{topic}' as {message_type}")

    def _make_callback(self, topic):
        def callback(message):
            self.get_logger().info(f"Received on '{topic}': {message}")

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TestSubscriber()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
