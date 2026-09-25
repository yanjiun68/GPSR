#!/usr/bin/env python3

import json
from pathlib import Path

import rclpy
from robot_ai_interfaces.action import SpeakText
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


PROFILE_DEFAULTS = {
    'target': 'everyone',
    'guest_name': 'guest',
    'age': 'unknown',
    'favourite_drink': 'unknown drink',
    'shirt_color': 'unknown shirt',
    'item1': 'nothing',
    'item2': 'nothing',
    'item3': 'nothing',
    'item4': 'nothing',
}


class JsonGuestTextRenderer(Node):
    def __init__(self):
        super().__init__('json_guest_text_renderer')

        self.declare_parameter('json_path', '~/.ros/rc_task/guest_memory.json')
        self.declare_parameter('json_trigger_topic', '/guest_profile/speak_from_json')
        self.declare_parameter('default_profile_id', 'latest')
        self.declare_parameter('target_name', 'everyone')
        self.declare_parameter('action_name', '/tts/speak')
        self.declare_parameter('tts_server_timeout_sec', 1.0)
        self.declare_parameter(
            'template',
            'Hi {target}, this is {guest_name}, {guest_name} likes {favourite_drink}, '
            '{guest_name} is {age} years old, {guest_name} wears a {shirt_color}, '
            'has {item1}, {item2}, {item3}, {item4}.'
        )

        self.json_path = Path(str(self.get_parameter('json_path').value)).expanduser()
        self.default_profile_id = str(self.get_parameter('default_profile_id').value)
        self.target_name = str(self.get_parameter('target_name').value)
        self.action_name = str(self.get_parameter('action_name').value)
        self.tts_server_timeout_sec = float(
            self.get_parameter('tts_server_timeout_sec').value
        )
        self.template = str(self.get_parameter('template').value)
        self.trigger_topic = str(self.get_parameter('json_trigger_topic').value)

        self.trigger_subscription = self.create_subscription(
            String,
            self.trigger_topic,
            self.trigger_callback,
            10,
        )
        self.tts_client = ActionClient(self, SpeakText, self.action_name)
        self._pending_goal_futures = set()
        self._pending_result_futures = set()

        self.get_logger().info(f'Reading guest memory JSON from: {self.json_path}')
        self.get_logger().info(f'Subscribed to JSON TTS trigger: {self.trigger_topic}')
        self.get_logger().info(f'Sending rendered text to action: {self.action_name}')

    def trigger_callback(self, msg: String) -> None:
        requested_profile_id = msg.data.strip() or self.default_profile_id
        try:
            payload = self.load_json_memory()
            profile = self.resolve_profile(payload, requested_profile_id)
            values = self.profile_to_template_values(profile)
            rendered_text = self.template.format(**values)
            self.send_speak_goal(rendered_text)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to render JSON guest profile "{requested_profile_id}": {exc}'
            )

    def load_json_memory(self) -> dict:
        with self.json_path.open('r', encoding='utf-8') as file_handle:
            payload = json.load(file_handle)
        if int(payload.get('schema_version', 0)) != 1:
            raise ValueError(f'unsupported schema_version={payload.get("schema_version")}')
        return payload

    def resolve_profile(self, payload: dict, requested_profile_id: str) -> dict:
        committed_guests = payload.get('committed_guests') or {}
        profile_id = requested_profile_id.strip() or self.default_profile_id

        if profile_id == 'latest':
            profile_id = str(payload.get('latest_committed_profile_id') or '').strip()

        if profile_id and profile_id in committed_guests:
            return committed_guests[profile_id]

        current_guest = payload.get('current_guest')
        if current_guest:
            self.get_logger().warn(
                f'Profile "{requested_profile_id}" not committed; using current_guest.'
            )
            return current_guest

        if profile_id:
            raise ValueError(f'profile "{profile_id}" was not found in JSON memory')
        raise ValueError('no latest committed guest or current guest exists in JSON memory')

    def profile_to_template_values(self, profile: dict) -> dict:
        values = dict(PROFILE_DEFAULTS)
        appearance = profile.get('appearance') or {}
        items = self.appearance_items(appearance)
        while len(items) < 4:
            items.append('nothing')

        values.update(
            {
                'target': self.target_name,
                'guest_name': str(profile.get('name') or 'guest').strip() or 'guest',
                'age': self.age_text(profile.get('age')),
                'favourite_drink': str(
                    profile.get('favorite_drink') or 'unknown drink'
                ).strip(),
                'shirt_color': self.shirt_color_text(appearance),
                'item1': items[0],
                'item2': items[1],
                'item3': items[2],
                'item4': items[3],
            }
        )
        return values

    def age_text(self, age_value) -> str:
        try:
            age = int(age_value)
        except (TypeError, ValueError):
            return 'unknown'
        return str(age) if age > 0 else 'unknown'

    def shirt_color_text(self, appearance: dict) -> str:
        shirt_color = str(appearance.get('shirt_color') or '').strip()
        if not shirt_color or shirt_color.lower() == 'unknown':
            return 'unknown shirt'
        if 'shirt' in shirt_color.lower():
            return shirt_color
        return f'{shirt_color} shirt'

    def appearance_items(self, appearance: dict) -> list[str]:
        items = []
        if appearance.get('glasses') == 'yes':
            items.append('glasses')
        if appearance.get('hat') == 'yes':
            items.append('a hat')
        if appearance.get('face_mask') == 'yes':
            items.append('a face mask')
        return items

    def send_speak_goal(self, rendered_text: str) -> bool:
        if not self.tts_client.wait_for_server(timeout_sec=self.tts_server_timeout_sec):
            self.get_logger().error(f'TTS action server not available: {self.action_name}')
            return False

        goal_msg = SpeakText.Goal()
        goal_msg.text = rendered_text

        future = self.tts_client.send_goal_async(goal_msg)
        self._pending_goal_futures.add(future)
        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'Sent JSON-rendered text as TTS goal: {rendered_text}')
        return True

    def goal_response_callback(self, future) -> None:
        self._pending_goal_futures.discard(future)
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'TTS goal send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().warn('TTS goal was rejected by Piper action server')
            return

        result_future = goal_handle.get_result_async()
        self._pending_result_futures.add(result_future)
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future) -> None:
        self._pending_result_futures.discard(future)
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f'TTS result failed: {exc}')
            return

        if result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().error(result.message)


def main(args=None):
    rclpy.init(args=args)
    node = JsonGuestTextRenderer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
