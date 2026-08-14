#!/usr/bin/env python3
"""Section: Count-object blackboard reporter node."""

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


class CountObjectsBlackboardTTSNode(Node):
    """Special function: Mirror confirmed object quantities into blackboard memory and speak them."""

    def __init__(self) -> None:
        """Special function: Create ROS interfaces, local YASMIN blackboard, and report timers."""
        super().__init__("count_objects_blackboard_tts_node")

        # Section: Declare ROS topic, service, and reporting behavior parameters.
        self.declare_parameter("object_quantities_topic", "/perception/object_quantities")
        self.declare_parameter("tts_service_name", "/voice/tts/say")
        self.declare_parameter("speak_confirmed_only", True)
        self.declare_parameter("speak_empty_scene", False)
        self.declare_parameter("speak_on_change_only", True)
        self.declare_parameter("min_report_interval_sec", 5.0)
        self.declare_parameter("max_items_per_report", 8)
        self.declare_parameter("report_style", "combined")

        # Section: Keep count-object skill data in normal YASMIN blackboard keys.
        self.blackboard = Blackboard()
        self.blackboard[bb.OBJECT_QUANTITIES] = {}
        self.blackboard[bb.COUNT_RESULT] = 0
        self.blackboard[bb.REPORT_TEXT] = ""
        self.blackboard[bb.COUNT_OBJECTS_RESULT_JSON] = "{}"

        # Section: Track speech queue state so high-rate perception does not block callbacks.
        self.last_fingerprint = ""
        self.last_report_time_sec = 0.0
        self.speech_queue: deque[str] = deque()
        self.active_tts_future: Any | None = None
        self.active_tts_text = ""

        # Section: Subscribe to stable confirmed object quantity summaries.
        self.create_subscription(
            String,
            str(self.get_parameter("object_quantities_topic").value),
            self.object_quantities_callback,
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
            "Count object blackboard TTS node ready: "
            f"topic={self.get_parameter('object_quantities_topic').value}, "
            f"tts={self.get_parameter('tts_service_name').value}"
        )

    def object_quantities_callback(self, msg: String) -> None:
        """Special function: Parse object quantity JSON and update count-object blackboard keys."""
        payload = self.parse_payload(msg.data)
        if payload is None:
            return

        # Section: Ignore raw/unconfirmed summaries when stable reporting is requested.
        confirmed_only = bool(self.get_parameter("speak_confirmed_only").value)
        if confirmed_only and not bool(payload.get("confirmed", False)):
            return

        # Section: Normalize and store quantities in blackboard memory.
        quantities = self.normalize_quantities(payload.get("object_quantities", {}))
        total_count = sum(quantities.values())
        self.blackboard[bb.OBJECT_QUANTITIES] = quantities
        self.blackboard[bb.COUNT_RESULT] = total_count
        self.blackboard[bb.COUNT_OBJECTS_RESULT_JSON] = json.dumps(payload, sort_keys=True)

        # Section: Build report text from the same blackboard keys that the skill uses.
        report_lines = self.build_report_lines(quantities)
        report_text = " ".join(report_lines)
        self.blackboard[bb.REPORT_TEXT] = report_text

        # Section: Queue speech only when the report is meaningful and allowed by throttling.
        if self.should_queue_report(quantities):
            self.speech_queue.clear()
            self.speech_queue.extend(report_lines)
            self.last_fingerprint = self.make_fingerprint(quantities)
            self.last_report_time_sec = time.monotonic()
            self.get_logger().info(f"Queued object count report: {report_text}")

    def parse_payload(self, raw_text: str) -> dict[str, Any] | None:
        """Special function: Parse JSON from /perception/object_quantities safely."""
        text = str(raw_text).strip()
        if not text:
            self.get_logger().warn("Ignored empty object quantity message.")
            return None
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignored invalid object quantity JSON: {text}")
            return None
        if not isinstance(payload, dict):
            self.get_logger().warn("Ignored object quantity payload because it is not a JSON object.")
            return None
        return payload

    def normalize_quantities(self, raw_quantities: Any) -> dict[str, int]:
        """Special function: Convert quantity payload into sorted positive integer counts."""
        normalized: dict[str, int] = {}
        if not isinstance(raw_quantities, dict):
            return normalized

        # Section: Keep only named object types with positive integer counts.
        for raw_name, raw_count in raw_quantities.items():
            name = str(raw_name).strip().lower()
            if not name:
                continue
            try:
                count = int(raw_count)
            except (TypeError, ValueError):
                continue
            if count <= 0:
                continue
            normalized[name] = count
        return dict(sorted(normalized.items()))

    def should_queue_report(self, quantities: dict[str, int]) -> bool:
        """Special function: Decide whether this blackboard update should trigger TTS."""
        now_sec = time.monotonic()
        fingerprint = self.make_fingerprint(quantities)
        min_interval = float(self.get_parameter("min_report_interval_sec").value)
        speak_empty = bool(self.get_parameter("speak_empty_scene").value)
        on_change_only = bool(self.get_parameter("speak_on_change_only").value)

        # Section: Skip empty reports unless the operator explicitly wants empty-scene speech.
        if not quantities and not speak_empty:
            return False

        # Section: Skip unchanged reports when change-only reporting is active.
        if on_change_only and fingerprint == self.last_fingerprint:
            return False

        # Section: Throttle speech so fast perception updates do not overwhelm TTS.
        if (now_sec - self.last_report_time_sec) < min_interval:
            return False

        return True

    def build_report_lines(self, quantities: dict[str, int]) -> list[str]:
        """Special function: Build human-friendly TTS lines from blackboard object quantities."""
        if not quantities:
            return ["I do not see any confirmed objects right now."]

        # Section: Limit speech length while preserving the highest-count object types first.
        max_items = max(1, int(self.get_parameter("max_items_per_report").value))
        sorted_items = sorted(quantities.items(), key=lambda item: (-item[1], item[0]))
        visible_items = sorted_items[:max_items]
        hidden_count = max(0, len(sorted_items) - len(visible_items))
        report_style = str(self.get_parameter("report_style").value).strip().lower()

        # Section: Per-type mode speaks one sentence for each object type.
        if report_style == "per_type":
            lines = [f"I can see {self.format_count_phrase(name, count)}." for name, count in visible_items]
            if hidden_count:
                lines.append(f"There are also {hidden_count} more object types in view.")
            return lines

        # Section: Combined mode speaks one natural summary sentence.
        phrases = [self.format_count_phrase(name, count) for name, count in visible_items]
        summary = f"I can see {self.join_human_list(phrases)}."
        if hidden_count:
            summary += f" I am skipping {hidden_count} smaller object types to keep the report short."
        return [summary]

    def format_count_phrase(self, object_name: str, count: int) -> str:
        """Special function: Format one object type and count in a natural spoken phrase."""
        number_word = self.number_to_words(count)
        display_name = object_name.replace("_", " ")
        if count == 1:
            return f"{number_word} {display_name}"
        return f"{number_word} {self.pluralize(display_name)}"

    def join_human_list(self, phrases: list[str]) -> str:
        """Special function: Join phrases using natural comma and final-and wording."""
        if not phrases:
            return "nothing"
        if len(phrases) == 1:
            return phrases[0]
        if len(phrases) == 2:
            return f"{phrases[0]} and {phrases[1]}"
        return f"{', '.join(phrases[:-1])}, and {phrases[-1]}"

    def pluralize(self, name: str) -> str:
        """Special function: Apply simple English pluralization for object labels."""
        if name.endswith(("s", "x", "ch", "sh")):
            return f"{name}es"
        if name.endswith("y") and len(name) > 1 and name[-2] not in "aeiou":
            return f"{name[:-1]}ies"
        return f"{name}s"

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

    def make_fingerprint(self, quantities: dict[str, int]) -> str:
        """Special function: Build a compact change key for throttled speech reports."""
        return json.dumps(dict(sorted(quantities.items())), sort_keys=True)

    def tts_timer_callback(self) -> None:
        """Special function: Send queued blackboard report lines to the TTS service."""
        if self.active_tts_future is not None:
            self.handle_active_tts_future()
            return
        if not self.speech_queue:
            return
        if not self.tts_client.service_is_ready():
            self.get_logger().warn("TTS service is not ready; keeping object report queued.")
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
            self.get_logger().info(f"Spoken object report line: {self.active_tts_text}")
        else:
            self.get_logger().warn(f"TTS failed for object report line: {message}")
        self.active_tts_future = None
        self.active_tts_text = ""


def main(args: list[str] | None = None) -> None:
    """Special function: Run the count-object blackboard TTS reporter node."""
    rclpy.init(args=args)
    node = CountObjectsBlackboardTTSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
