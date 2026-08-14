#!/usr/bin/env python3
"""Section: Count-people blackboard reporter node."""

from __future__ import annotations

import json
import time
from collections import deque
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from voice_interfaces.srv import SayText


class CountPeopleBlackboardTTSNode(Node):
    """Special function: Mirror confirmed human quantities into blackboard memory and speak them."""

    def __init__(self) -> None:
        """Special function: Create ROS interfaces, local YASMIN blackboard, and report timers."""
        super().__init__("count_people_blackboard_tts_node")

        # Section: Declare ROS topic, service, and reporting behavior parameters.
        self.declare_parameter("human_quantities_topic", "/perception/human_quantities")
        self.declare_parameter("tts_service_name", "/voice/tts/say")
        self.declare_parameter("speak_confirmed_only", True)
        self.declare_parameter("speak_empty_scene", False)
        self.declare_parameter("speak_on_change_only", True)
        self.declare_parameter("min_report_interval_sec", 5.0)

        # Section: Keep count-people skill data in normal YASMIN blackboard keys.
        self.blackboard = Blackboard()
        self.blackboard[bb.COUNT_RESULT] = 0
        self.blackboard[bb.REPORT_TEXT] = ""
        self.blackboard[bb.COUNT_PEOPLE_RESULT_JSON] = "{}"
        self.blackboard[bb.DETECTED_PEOPLE] = []

        # Section: Track speech queue state so high-rate perception does not block callbacks.
        self.last_fingerprint = ""
        self.last_report_time_sec = 0.0
        self.speech_queue: deque[str] = deque()
        self.active_tts_future: Any | None = None
        self.active_tts_text = ""

        # Section: Subscribe to stable confirmed human quantity summaries.
        self.create_subscription(
            String,
            str(self.get_parameter("human_quantities_topic").value),
            self.human_quantities_callback,
            10,
        )

        # Section: Send report text to the existing GPSR voice service.
        self.tts_client = self.create_client(
            SayText,
            str(self.get_parameter("tts_service_name").value),
        )

        # Section: Drain queued speech without doing blocking service work in the subscriber.
        self.create_timer(0.1, self.tts_timer_callback)

        # Section: Log wiring so launch output shows where the report node is connected.
        self.get_logger().info(
            "Count people blackboard TTS node ready: "
            f"topic={self.get_parameter('human_quantities_topic').value}, "
            f"tts={self.get_parameter('tts_service_name').value}"
        )

    def human_quantities_callback(self, msg: String) -> None:
        """Special function: Parse human quantity JSON and update count-people blackboard keys."""
        payload = self.parse_payload(msg.data)
        if payload is None:
            return

        # Section: Ignore raw/unconfirmed summaries when stable reporting is requested.
        confirmed_only = bool(self.get_parameter("speak_confirmed_only").value)
        if confirmed_only and not bool(payload.get("confirmed", False)):
            return

        # Section: Normalize and store the current human count in blackboard memory.
        count = self.extract_people_count(payload)
        detected_people = self.build_detected_people(count)
        self.blackboard[bb.COUNT_RESULT] = count
        self.blackboard[bb.COUNT_PEOPLE_RESULT_JSON] = json.dumps(payload, sort_keys=True)
        self.blackboard[bb.DETECTED_PEOPLE] = detected_people

        # Section: Build report text from the same blackboard keys that the skill uses.
        report_text = self.build_report_text(count)
        self.blackboard[bb.REPORT_TEXT] = report_text

        # Section: Queue speech only when the report is meaningful and allowed by throttling.
        if self.should_queue_report(count):
            self.speech_queue.clear()
            self.speech_queue.append(report_text)
            self.last_fingerprint = self.make_fingerprint(count)
            self.last_report_time_sec = time.monotonic()
            self.get_logger().info(f"Queued people count report: {report_text}")

    def parse_payload(self, raw_text: str) -> dict[str, Any] | None:
        """Special function: Parse JSON from /perception/human_quantities safely."""
        text = str(raw_text).strip()
        if not text:
            self.get_logger().warn("Ignored empty human quantity message.")
            return None
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignored invalid human quantity JSON: {text}")
            return None
        if not isinstance(payload, dict):
            self.get_logger().warn("Ignored human quantity payload because it is not a JSON object.")
            return None
        return payload

    def extract_people_count(self, payload: dict[str, Any]) -> int:
        """Special function: Extract the confirmed person count from a human quantity payload."""
        quantities = payload.get("human_quantities", {})
        if isinstance(quantities, dict) and "person" in quantities:
            try:
                return max(0, int(quantities.get("person", 0)))
            except (TypeError, ValueError):
                return 0
        try:
            return max(0, int(payload.get("total_detected_count", 0)))
        except (TypeError, ValueError):
            return 0

    def build_detected_people(self, count: int) -> list[dict[str, Any]]:
        """Special function: Build blackboard-friendly people placeholders from a quantity-only topic."""
        return [{"entity_id": f"person_{index + 1}", "name": "person"} for index in range(max(0, count))]

    def should_queue_report(self, count: int) -> bool:
        """Special function: Decide whether this blackboard update should trigger TTS."""
        now_sec = time.monotonic()
        fingerprint = self.make_fingerprint(count)
        min_interval = float(self.get_parameter("min_report_interval_sec").value)
        speak_empty = bool(self.get_parameter("speak_empty_scene").value)
        on_change_only = bool(self.get_parameter("speak_on_change_only").value)

        # Section: Skip empty reports unless the operator explicitly wants empty-scene speech.
        if count <= 0 and not speak_empty:
            return False

        # Section: Skip unchanged reports when change-only reporting is active.
        if on_change_only and fingerprint == self.last_fingerprint:
            return False

        # Section: Throttle speech so fast perception updates do not overwhelm TTS.
        if (now_sec - self.last_report_time_sec) < min_interval:
            return False

        return True

    def build_report_text(self, count: int) -> str:
        """Special function: Build human-friendly TTS text from the people count."""
        if count <= 0:
            return "I do not see any confirmed people right now."
        if count == 1:
            return "I can see one person."
        return f"I can see {self.number_to_words(count)} people."

    def number_to_words(self, count: int) -> str:
        """Special function: Use spoken words for small counts and digits for larger counts."""
        words = {
            0: "zero",
            1: "one",
            2: "two",
            3: "three",
            4: "four",
            5: "five",
            6: "six",
            7: "seven",
            8: "eight",
            9: "nine",
            10: "ten",
        }
        return words.get(int(count), str(int(count)))

    def make_fingerprint(self, count: int) -> str:
        """Special function: Build a compact change key for throttled speech reports."""
        return str(max(0, int(count)))

    def tts_timer_callback(self) -> None:
        """Special function: Send queued blackboard report lines to the TTS service."""
        if self.active_tts_future is not None:
            self.handle_active_tts_future()
            return
        if not self.speech_queue:
            return
        if not self.tts_client.service_is_ready():
            self.get_logger().warn("TTS service is not ready; keeping people report queued.")
            return

        # Section: Pop the next report line and send it asynchronously.
        self.active_tts_text = self.speech_queue.popleft()
        request = SayText.Request()
        request.text = self.active_tts_text
        self.active_tts_future = self.tts_client.call_async(request)

    def handle_active_tts_future(self) -> None:
        """Special function: Complete the active TTS request and continue the report queue."""
        if self.active_tts_future is None or not self.active_tts_future.done():
            return

        # Section: Read the TTS result without throwing from the timer callback.
        try:
            response = self.active_tts_future.result()
            success = bool(response is not None and response.success)
            message = str(getattr(response, "message", ""))
        except Exception as exc:  # noqa: BLE001
            success = False
            message = str(exc)

        # Section: Report the speech outcome for manual troubleshooting.
        if success:
            self.get_logger().info(f"Spoken people report line: {self.active_tts_text}")
        else:
            self.get_logger().warn(f"TTS failed for people report line: {message}")
        self.active_tts_future = None
        self.active_tts_text = ""


def main(args: list[str] | None = None) -> None:
    """Special function: Run the count-people blackboard TTS reporter node."""
    rclpy.init(args=args)
    node = CountPeopleBlackboardTTSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
