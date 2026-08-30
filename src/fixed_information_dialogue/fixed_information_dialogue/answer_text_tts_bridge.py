#!/usr/bin/env python3
"""Section: Forward fixed-information answer text to the existing TTS service."""

from __future__ import annotations

from collections import deque
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from voice_interfaces.srv import SayText


def normalize_answer_text(raw_text: str) -> str:
    """Special function: Return only trimmed, speakable answer text."""
    # Section: Normalize topic payloads without parsing or forwarding answer metadata.
    return str(raw_text).strip()


class AnswerTextTTSBridge(Node):
    """Special function: Queue answer-text messages and send them to the TTS service."""

    def __init__(self) -> None:
        """Special function: Create the answer-text subscriber, TTS client, and queue timer."""
        # Section: Initialize the ROS node with a graph name specific to this one-way bridge.
        super().__init__("fixed_information_answer_text_tts_bridge")

        # Section: Declare configurable interface names and bounded queue behavior.
        self.declare_parameter("answer_text_topic", "/fixed_information/answer_text")
        self.declare_parameter("tts_service_name", "/voice/tts/say")
        self.declare_parameter("dispatch_period_sec", 0.1)
        self.declare_parameter("max_queue_size", 10)

        # Section: Track pending answer text and at most one active TTS service request.
        self._speech_queue: deque[str] = deque()
        self._active_future: Any | None = None
        self._active_text = ""
        self._waiting_for_tts_logged = False

        # Section: Subscribe exclusively to the provider's plain answer-text topic.
        self._answer_subscription = self.create_subscription(
            String,
            str(self.get_parameter("answer_text_topic").value),
            self._answer_text_callback,
            10,
        )

        # Section: Connect output to the service interface already exposed by tts_node.py.
        self._tts_client = self.create_client(
            SayText,
            str(self.get_parameter("tts_service_name").value),
        )

        # Section: Dispatch asynchronously so a missing or busy TTS node never blocks subscription callbacks.
        dispatch_period_sec = max(
            0.01,
            float(self.get_parameter("dispatch_period_sec").value),
        )
        self._dispatch_timer = self.create_timer(
            dispatch_period_sec,
            self._dispatch_timer_callback,
        )

        # Section: Report the exact one-topic-to-one-service wiring at startup.
        self.get_logger().info(
            "Fixed-information answer-text TTS bridge ready: "
            f"topic={self.get_parameter('answer_text_topic').value}, "
            f"tts={self.get_parameter('tts_service_name').value}"
        )

    def _answer_text_callback(self, message: String) -> None:
        """Special function: Accept only non-empty answer text and place it in the speech queue."""
        # Section: Extract only String.data; the JSON answer envelope is never subscribed to or parsed.
        answer_text = normalize_answer_text(message.data)
        if not answer_text:
            self.get_logger().warning("Ignored an empty fixed-information answer text message.")
            return

        # Section: Bound queued speech so a disconnected TTS service cannot grow memory indefinitely.
        max_queue_size = max(1, int(self.get_parameter("max_queue_size").value))
        if len(self._speech_queue) >= max_queue_size:
            dropped_text = self._speech_queue.popleft()
            self.get_logger().warning(
                f"Dropped oldest queued answer because the TTS queue is full: {dropped_text}"
            )

        # Section: Preserve answer order while allowing the subscriber to return immediately.
        self._speech_queue.append(answer_text)
        self.get_logger().info(f"Queued fixed-information answer text: {answer_text}")

    def _dispatch_timer_callback(self) -> None:
        """Special function: Complete active speech or dispatch the next queued answer."""
        # Section: Finish the current asynchronous request before sending another one.
        if self._active_future is not None:
            self._finish_active_request()
            return

        # Section: Remain idle until the provider publishes answer text.
        if not self._speech_queue:
            return

        # Section: Retain queued text while tts_node.py is unavailable.
        if not self._tts_client.service_is_ready():
            if not self._waiting_for_tts_logged:
                self.get_logger().warning(
                    "TTS service is unavailable; fixed-information answer text remains queued."
                )
                self._waiting_for_tts_logged = True
            return

        # Section: Reset the one-shot availability warning after the service becomes ready.
        self._waiting_for_tts_logged = False

        # Section: Copy only the answer string into the SayText request.
        self._active_text = self._speech_queue.popleft()
        request = SayText.Request()
        request.text = self._active_text
        self._active_future = self._tts_client.call_async(request)

    def _finish_active_request(self) -> None:
        """Special function: Record the completed TTS response and release the active request."""
        # Section: Wait non-blockingly for the service future to complete.
        if self._active_future is None or not self._active_future.done():
            return

        # Section: Convert service success or exceptions into operator-visible ROS logs.
        try:
            response = self._active_future.result()
            success = bool(response is not None and response.success)
            result_message = str(getattr(response, "message", ""))
        except Exception as exc:  # noqa: BLE001
            success = False
            result_message = str(exc)

        # Section: Report whether tts_node.py accepted and played the answer text.
        if success:
            self.get_logger().info(f"Spoken fixed-information answer: {self._active_text}")
        else:
            self.get_logger().warning(
                f"TTS failed for fixed-information answer '{self._active_text}': {result_message}"
            )

        # Section: Clear active state so the next timer cycle can dispatch another answer.
        self._active_future = None
        self._active_text = ""


def main(args: list[str] | None = None) -> None:
    """Special function: Initialize and run the fixed-information answer-text TTS bridge."""
    # Section: Own the ROS context for the standalone bridge executable.
    rclpy.init(args=args)
    node = AnswerTextTTSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
