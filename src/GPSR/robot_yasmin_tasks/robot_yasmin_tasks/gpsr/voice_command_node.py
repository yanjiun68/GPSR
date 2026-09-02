"""Section: GPSR spoken command intake node."""

import json
import threading
import time
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, String
from voice_interfaces.srv import SayText


class GpsrVoiceCommandNode(Node):
    """Special function: Prompt, listen, parse, and submit a GPSR queue."""

    def __init__(self) -> None:
        """Special function: Create local TTS, STT, parser, and executor clients."""
        super().__init__("gpsr_voice_command")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("repeat", False)
        self.declare_parameter("command_topic", "/gpsr/command")
        self.declare_parameter("scheduler_state_topic", "/gpsr/state")
        self.declare_parameter("command_result_topic", "/gpsr/command_result")
        self.declare_parameter("record_sec", 6.0)
        # Section: Bound the initial GPSR spoken-command input window to twelve seconds.
        self.declare_parameter("listen_timeout_sec", 12.0)
        self.declare_parameter("prompt_text", "Please give me a GPSR command after the beep.")
        self.tts = self.create_client(SayText, "/voice/tts/say")
        self.stt_pub = self.create_publisher(Float32, "/voice/stt/start", 10)
        self.stt_sub = self.create_subscription(String, "/voice/stt/transcript", self._stt_cb, 10)
        # Section: Route transcripts only through the guarded GPSR task scheduler.
        self.command_pub = self.create_publisher(
            String, str(self.get_parameter("command_topic").value), 10
        )
        self.scheduler_state_sub = self.create_subscription(
            String,
            str(self.get_parameter("scheduler_state_topic").value),
            self._scheduler_state_cb,
            10,
        )
        self.command_result_sub = self.create_subscription(
            String,
            str(self.get_parameter("command_result_topic").value),
            self._command_result_cb,
            10,
        )
        self.scheduler_state = "INITIALIZING"
        self.latest = ""
        self.event = threading.Event()
        self.busy = False
        self.awaiting_result = False
        if bool(self.get_parameter("auto_start").value):
            self.create_timer(2.0, self._start_once)

    def _start_once(self) -> None:
        """Special function: Start one voice-command cycle."""
        if self.busy or self.awaiting_result or self.scheduler_state != "WAITING_FOR_COMMAND":
            return
        # Section: Reserve the single command worker before starting its background thread.
        self.busy = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        """Special function: Execute prompt/listen/plan/submit pipeline."""
        try:
            self._say(str(self.get_parameter("prompt_text").value))
            # Section: Reuse one worker and persistent subscriptions across timeout windows.
            while self.scheduler_state == "WAITING_FOR_COMMAND":
                timeout_sec = float(self.get_parameter("listen_timeout_sec").value)
                self.get_logger().info(
                    f"Waiting for GPSR command (timeout: {timeout_sec:.1f} s)."
                )
                transcript = self._listen()
                if transcript:
                    # Section: Publish exactly one transcript; scheduler owns parsing and execution.
                    self.get_logger().info(
                        "GPSR transcript received; awaiting scheduler result."
                    )
                    command_msg = String()
                    command_msg.data = transcript
                    self.awaiting_result = True
                    self.command_pub.publish(command_msg)
                    return
                # Section: A listening timeout only re-arms intake and never requests motion.
                self.get_logger().info(
                    f"No command received within {timeout_sec:.1f} s; "
                    "re-arming command input."
                )
        finally:
            # Section: Keep intake blocked until the scheduler supplies the command verdict.
            if not self.awaiting_result:
                self.busy = False

    def _scheduler_state_cb(self, msg: String) -> None:
        """Special function: Enable listening only while the scheduler is ready."""
        try:
            payload = json.loads(str(msg.data))
        except json.JSONDecodeError:
            return
        state = str(payload.get("state", "")).strip()
        if state:
            # Section: Log entry into ready state once per scheduler transition.
            if state == "WAITING_FOR_COMMAND" and self.scheduler_state != state:
                self.get_logger().info(
                    "GPSR ready to receive command at current robot position."
                )
            self.scheduler_state = state

    def _stt_cb(self, msg: String) -> None:
        """Special function: Receive one STT transcript."""
        self.latest = msg.data.strip()
        self.event.set()

    def _command_result_cb(self, msg: String) -> None:
        """Special function: Speak only after a submitted voice command has a scheduler verdict."""
        if not self.awaiting_result:
            return
        try:
            result = json.loads(str(msg.data))
        except json.JSONDecodeError:
            return

        # Section: Distinguish proven queue success, rejection/failure, and non-executed results.
        execution = result.get("execution")
        response_text = ""
        if isinstance(execution, dict):
            response_text = (
                "I received the command."
                if bool(execution.get("success", False))
                else "The command is rejected."
            )
        elif not bool(result.get("accepted", False)):
            response_text = "The command is rejected."

        # Section: Keep quiet for dry runs or inconclusive results, then reopen command intake.
        try:
            if response_text:
                self._say(response_text)
        finally:
            self.awaiting_result = False
            self.busy = False

    def _listen(self) -> str:
        """Special function: Trigger local STT and wait for transcript."""
        self.latest = ""
        self.event.clear()
        trigger = Float32()
        trigger.data = float(self.get_parameter("record_sec").value)
        self.stt_pub.publish(trigger)
        deadline = time.monotonic() + float(self.get_parameter("listen_timeout_sec").value)
        while True:
            # Section: Cap the final event wait at the exact remaining command window.
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return ""
            if self.event.wait(timeout=min(0.05, remaining)):
                # Section: Ignore empty STT callbacks while retaining the original deadline.
                transcript = self.latest.strip()
                if transcript:
                    return transcript
                self.event.clear()

    def _say(self, text: str) -> bool:
        """Special function: Speak prompt or status through local TTS."""
        if not self.tts.wait_for_service(timeout_sec=3.0):
            return False
        req = SayText.Request()
        req.text = text
        future = self.tts.call_async(req)
        return self._wait(future, 8.0)

    def _wait(self, future: Any, timeout: float) -> bool:
        """Special function: Wait for ROS future while executor threads spin."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return False


def main(args: list[str] | None = None) -> None:
    """Special function: Run GPSR voice command intake node."""
    rclpy.init(args=args)
    node = GpsrVoiceCommandNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
