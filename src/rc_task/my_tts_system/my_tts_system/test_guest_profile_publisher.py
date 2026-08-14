#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


TEST_PROFILE = {
    'target': 'John',
    'guest_name': 'Alan',
    'age': '21',
    'favourite_drink': 'water',
    'shirt_color': 'blue shirt',
    'item1': 'glasses',
    'item2': 'masks',
    'item3': 'wallet',
    'item4': 'watch',
}


class TestGuestProfilePublisher(Node):
    def __init__(self):
        super().__init__('test_guest_profile_publisher')

        for field_name, default_value in TEST_PROFILE.items():
            self.declare_parameter(f'{field_name}_topic', f'/guest_profile/{field_name}')
            self.declare_parameter(field_name, default_value)

        self.declare_parameter('trigger_topic', '/guest_profile/ready')
        self.declare_parameter('publish_stored_topic', '/guest_profile/publish_stored')
        self.declare_parameter('publish_once', True)
        self.declare_parameter('ready_delay_sec', 0.1)

        self.topic_by_field = {
            field_name: self.get_parameter(f'{field_name}_topic').get_parameter_value().string_value
            for field_name in TEST_PROFILE
        }
        trigger_topic = self.get_parameter('trigger_topic').get_parameter_value().string_value
        publish_stored_topic = (
            self.get_parameter('publish_stored_topic').get_parameter_value().string_value
        )
        self.publish_once = self.get_parameter('publish_once').get_parameter_value().bool_value
        self.ready_delay_sec = (
            self.get_parameter('ready_delay_sec').get_parameter_value().double_value
        )
        self.profile = {
            field_name: str(self.get_parameter(field_name).value)
            for field_name in TEST_PROFILE
        }

        self.publishers = {
            field_name: self.create_publisher(String, topic, 10)
            for field_name, topic in self.topic_by_field.items()
        }
        self.trigger_publisher = self.create_publisher(Empty, trigger_topic, 10)
        self._ready_timer = None
        self.field_subscriptions = []
        for field_name, topic in self.topic_by_field.items():
            subscription = self.create_subscription(
                String,
                topic,
                self.make_store_callback(field_name),
                10,
            )
            self.field_subscriptions.append(subscription)
        self.publish_stored_subscription = self.create_subscription(
            Empty,
            publish_stored_topic,
            self.publish_stored_profile,
            10,
        )
        self.timer = self.create_timer(3.0, self.publish_profile)
        self.sent_once = False

        for field_name, topic in self.topic_by_field.items():
            self.get_logger().info(f'Storing and publishing {field_name} on: {topic}')
        self.get_logger().info(f'Publishing profile trigger to: {trigger_topic}')
        self.get_logger().info(f'Republishing stored profile when triggered on: {publish_stored_topic}')

    def make_store_callback(self, field_name: str):
        def callback(msg: String) -> None:
            value = msg.data.strip()
            if not value:
                return

            self.profile[field_name] = value
            self.get_logger().debug(f'Stored {field_name}: {value}')

        return callback

    def publish_profile(self):
        if self.publish_once and self.sent_once:
            return

        self.publish_current_profile()
        self.sent_once = True

    def publish_stored_profile(self, _msg: Empty):
        self.publish_current_profile()

    def publish_current_profile(self):
        for field_name, value in self.profile.items():
            msg = String()
            msg.data = value
            self.publishers[field_name].publish(msg)

        self.schedule_ready_trigger()

        self.get_logger().info(f'Published stored guest profile fields: {self.profile}')

    def schedule_ready_trigger(self):
        if self._ready_timer is not None:
            self._ready_timer.cancel()
            self.destroy_timer(self._ready_timer)

        self._ready_timer = self.create_timer(self.ready_delay_sec, self.publish_ready_trigger)

    def publish_ready_trigger(self):
        timer = self._ready_timer
        self._ready_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

        self.trigger_publisher.publish(Empty())
        self.get_logger().info('Published stored guest profile ready trigger')


def main(args=None):
    rclpy.init(args=args)
    node = TestGuestProfilePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# This node shall be replaced, vision will substitute their outcome here.
