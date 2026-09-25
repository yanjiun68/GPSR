#!/usr/bin/env python3

import rclpy
from robot_ai_interfaces.action import SpeakText
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String


PROFILE_DEFAULTS = {
    'target': 'guest',
    'guest_name': 'Unknown',
    'age': 'unknown',
    'favourite_drink': 'unknown drink',
    'shirt_color': 'unknown shirt',
    'item1': 'nothing',
    'item2': 'nothing',
    'item3': 'nothing',
    'item4': 'nothing',
}


class GuestTextRenderer(Node):
    def __init__(self):
        super().__init__('guest_text_renderer')

        for field_name in PROFILE_DEFAULTS:
            self.declare_parameter(f'{field_name}_topic', f'/guest_profile/{field_name}')

        self.declare_parameter('trigger_topic', '/guest_profile/ready')
        self.declare_parameter('auto_render_on_update', False)
        self.declare_parameter('render_delay_sec', 0.2)
        self.declare_parameter('action_name', '/tts/speak')
        self.declare_parameter(
            'template',
            'Hi {target}, this is {guest_name}, {guest_name} likes {favourite_drink}, '
            '{guest_name} is {age} years old, {guest_name} wears a {shirt_color}, '
            'has {item1}, {item2}, {item3}, {item4}.'
        )

        self.topic_by_field = {
            field_name: self.get_parameter(f'{field_name}_topic').get_parameter_value().string_value
            for field_name in PROFILE_DEFAULTS
        }
        trigger_topic = self.get_parameter('trigger_topic').get_parameter_value().string_value
        self.auto_render_on_update = (
            self.get_parameter('auto_render_on_update').get_parameter_value().bool_value
        )
        self.render_delay_sec = (
            self.get_parameter('render_delay_sec').get_parameter_value().double_value
        )
        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.template = self.get_parameter('template').get_parameter_value().string_value

        self.profile_values = dict(PROFILE_DEFAULTS)
        self.received_fields = set()
        self._field_subscriptions = []
        self._render_timer = None
        self._last_rendered_profile = None

        for field_name, topic in self.topic_by_field.items():
            subscription = self.create_subscription(
                String,
                topic,
                self.make_field_callback(field_name),
                10,
            )
            self._field_subscriptions.append(subscription)

        self.trigger_subscription = self.create_subscription(
            Empty,
            trigger_topic,
            self.trigger_callback,
            10,
        )
        self.tts_client = ActionClient(self, SpeakText, self.action_name)
        self._pending_goal_futures = set()
        self._pending_result_futures = set()

        for field_name, topic in self.topic_by_field.items():
            self.get_logger().info(f'Subscribed to {field_name}: {topic}')
        self.get_logger().info(f'Subscribed to profile trigger: {trigger_topic}')
        self.get_logger().info(f'Sending rendered text to action: {self.action_name}')

    def make_field_callback(self, field_name: str):
        def callback(msg: String) -> None:
            value = msg.data.strip()
            self.profile_values[field_name] = value if value else PROFILE_DEFAULTS[field_name]
            self.received_fields.add(field_name)
            self.get_logger().debug(
                f'Updated guest profile field {field_name}: {self.profile_values[field_name]}'
            )

            if self.auto_render_on_update:
                self.schedule_render()

        return callback

    def trigger_callback(self, _msg: Empty) -> None:
        self.render_current_profile()

    def schedule_render(self) -> None:
        if self._render_timer is not None:
            self._render_timer.cancel()
            self.destroy_timer(self._render_timer)

        self._render_timer = self.create_timer(self.render_delay_sec, self.render_timer_callback)

    def render_timer_callback(self) -> None:
        timer = self._render_timer
        self._render_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

        self.render_current_profile()

    def render_current_profile(self) -> None:
        try:
            current_profile = tuple(
                (field_name, self.profile_values[field_name])
                for field_name in PROFILE_DEFAULTS
            )
            if current_profile == self._last_rendered_profile:
                self.get_logger().debug('Skipped duplicate guest profile render')
                return

            missing_fields = [
                field_name for field_name in PROFILE_DEFAULTS
                if field_name not in self.received_fields
            ]
            if missing_fields:
                self.get_logger().warn(
                    'Rendering guest profile with default values for: '
                    f'{", ".join(missing_fields)}'
                )

            rendered_text = self.template.format(**self.profile_values)

            if self.send_speak_goal(rendered_text):
                self._last_rendered_profile = current_profile
                self.get_logger().info(f'Sent rendered text as TTS goal: {rendered_text}')

        except Exception as exc:
            self.get_logger().error(f'Failed to render text: {exc}')

    def send_speak_goal(self, rendered_text: str) -> bool:
        if not self.tts_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f'TTS action server not available: {self.action_name}')
            return False

        goal_msg = SpeakText.Goal()
        goal_msg.text = rendered_text

        future = self.tts_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._pending_goal_futures.add(future)
        future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future) -> None:
        self._pending_goal_futures.discard(future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('TTS goal was rejected by Piper action server')
            return

        result_future = goal_handle.get_result_async()
        self._pending_result_futures.add(result_future)
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future) -> None:
        self._pending_result_futures.discard(future)
        result = future.result().result

        if result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().error(result.message)

    def feedback_callback(self, feedback_msg) -> None:
        self.get_logger().debug(f'TTS feedback: {feedback_msg.feedback.state}')


def main(args=None):
    rclpy.init(args=args)
    node = GuestTextRenderer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
