#!/usr/bin/env python3

# Section: Standard-library imports provide a process-lifetime exclusive audio lease.
import fcntl
import os
import subprocess
from pathlib import Path
from typing import TextIO

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from voice_interfaces.action import SpeakText
from voice_interfaces.srv import SayText
from ament_index_python.packages import get_package_share_directory


# Section: Every TTS implementation cooperates on this one host-local playback lock.
AUDIO_PLAYBACK_LOCK_PATH = Path("/tmp/putrabot_audio_playback.lock")


def acquire_audio_playback_lock() -> tuple[TextIO | None, str]:
    """Special function: Claim exclusive host audio playback ownership for this TTS server."""
    # Section: Open a stable lock file whose advisory lock is released automatically on process exit.
    lock_handle = AUDIO_PLAYBACK_LOCK_PATH.open("a+", encoding="utf-8")
    try:
        # Section: Never wait for another TTS server because a second server is an ownership error.
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        # Section: Return the recorded owner PID so the rejected launch is actionable for operators.
        lock_handle.seek(0)
        owner_pid = lock_handle.read().strip() or "unknown"
        lock_handle.close()
        return None, owner_pid
    # Section: Replace stale diagnostics only after this process has obtained the exclusive lease.
    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write(str(os.getpid()))
    lock_handle.flush()
    return lock_handle, str(os.getpid())


def release_audio_playback_lock(lock_handle: TextIO | None) -> None:
    """Special function: Release the audio lease during normal TTS shutdown."""
    # Section: A missing handle means startup was rejected before this process owned playback.
    if lock_handle is None:
        return
    try:
        # Section: Unlock explicitly so a replacement server can start immediately after shutdown.
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
    finally:
        # Section: Close the descriptor regardless of unlock outcome to release kernel ownership.
        lock_handle.close()


class TTSNode(Node):
    """Thin TTS wrapper: exposes a SayText service."""

    def __init__(self):
        super().__init__("voice_tts")

        # Section: Keep service health counters available to rqt Runtime Monitor.
        self._speaking = False
        self._request_count = 0
        self._failure_count = 0
        self._last_result = "Ready to speak"

        # Parameters (configurable by launch)
        self.declare_parameter("service_name", "/voice/tts/say")
        self.declare_parameter("action_name", "/voice/tts/say_action")
        self.declare_parameter("piper_bin", "piper")
        default_model_path = (
            get_package_share_directory("voice")
            + "/models/en_US-lessac-medium.onnx"
        )
        self.declare_parameter("piper_model_path", default_model_path)

        self.service_name = self.get_parameter("service_name").get_parameter_value().string_value
        self.action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self.piper_bin = self.get_parameter("piper_bin").get_parameter_value().string_value
        self.model_path = (
            self.get_parameter("piper_model_path").get_parameter_value().string_value
            or default_model_path
        )

        # Section: Refuse a duplicate provider before registering duplicate ROS service/action endpoints.
        self._audio_playback_lock, owner_pid = acquire_audio_playback_lock()
        if self._audio_playback_lock is None:
            self.get_logger().error(
                "[voice_tts] Audio playback is already owned by PID "
                f"{owner_pid}; do not start a second TTS server. "
                "Set start_voice_tts:=false on this secondary launch."
            )
            raise RuntimeError("duplicate audio playback owner")

        # Service server
        self._srv = self.create_service(SayText, self.service_name, self._handle_say)

        # Section: Expose long-running speech playback with action progress and results.
        self._say_action = ActionServer(
            self,
            SpeakText,
            self.action_name,
            execute_callback=self._execute_say_action,
            goal_callback=self._accept_say_goal,
            cancel_callback=self._accept_say_cancel,
        )

        # Section: Publish diagnostics and expose this service-only node in topic graphs.
        self._diagnostics = Updater(self)
        self._diagnostics.setHardwareID("default_audio_output")
        self._diagnostics.add("Text-to-speech", self._produce_diagnostics)
        self._diagnostics.force_update()

        self.get_logger().info(
            f"[voice_tts] Ready. Service: {self.service_name}; action: {self.action_name} "
            f"(model: {self.model_path or 'SKIP'})"
        )

    def _handle_say(self, request: SayText.Request, response: SayText.Response):
        """Synthesize service-request text and return playback status."""
        # Section: Run the shared speech pipeline and map its result to the service response.
        response.success, response.message = self._speak_text(request.text)
        return response

    def _accept_say_goal(self, goal_request: SpeakText.Goal) -> GoalResponse:
        """Reject empty TTS action goals before execution."""
        if not goal_request.text.strip():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _accept_say_cancel(self, _goal_handle) -> CancelResponse:
        """Accept cancellation requests at safe TTS stage boundaries."""
        return CancelResponse.ACCEPT

    async def _execute_say_action(self, goal_handle) -> SpeakText.Result:
        """Synthesize action-goal text while publishing playback progress."""
        # Section: Publish synthesis feedback before starting the blocking Piper pipeline.
        feedback = SpeakText.Feedback()
        feedback.stage = "SYNTHESIZING"
        feedback.message = "Generating and playing speech"
        goal_handle.publish_feedback(feedback)

        # Section: Honor cancellation received before playback begins.
        result = SpeakText.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "Speech canceled before playback"
            return result

        # Section: Run the same implementation used by the SayText service.
        result.success, result.message = self._speak_text(goal_handle.request.text)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "Speech canceled after the active playback stage"
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _speak_text(self, requested_text: str) -> tuple[bool, str]:
        """Synthesize and play text for both service and action callers."""
        # Section: Validate shared inputs and update health counters.
        self._request_count += 1
        text = requested_text.strip()
        if not text:
            self._failure_count += 1
            self._last_result = "Rejected empty text"
            self._diagnostics.force_update()
            self.get_logger().warning("[voice_tts] Received empty text, skipping.")
            return False, "empty text"
        if not self.model_path:
            self._failure_count += 1
            self._last_result = "Piper model path is empty"
            self._diagnostics.force_update()
            self.get_logger().warning("[voice_tts] piper_model_path is empty, skipping TTS.")
            return False, "piper_model_path is empty"

        # Section: Build a streaming Piper-to-ALSA command pipeline.
        cmd = [
            self.piper_bin,
            "--model",
            self.model_path,
            "--output_file",
            "-",  # stdout WAV
        ]

        try:
            # Section: Publish the active state before starting the blocking playback pipeline.
            self._speaking = True
            self._last_result = "Speaking"
            self._diagnostics.force_update()
            self.get_logger().info(f"[voice_tts] Speaking: \"{text}\"")
            p1 = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            assert p1.stdin is not None
            assert p1.stdout is not None
            assert p1.stderr is not None
            p1.stdin.write((text + "\n").encode("utf-8"))
            p1.stdin.close()

            subprocess.run(["aplay", "-q"], stdin=p1.stdout, check=False)
            stderr = p1.stderr.read()
            p1.wait(timeout=30)

            if p1.returncode not in (0, None):
                msg = f"Piper returned {p1.returncode}: {stderr.decode('utf-8', 'ignore')[:200]}"
                self._speaking = False
                self._failure_count += 1
                self._last_result = msg
                self.get_logger().warn(f"[voice_tts] {msg}")
                self._diagnostics.force_update()
                return False, msg

            self._speaking = False
            self._last_result = "Speech completed"
            self._diagnostics.force_update()
            return True, "spoken"
        except FileNotFoundError as e:
            self._speaking = False
            self._failure_count += 1
            self._last_result = f"Missing binary: {e}"
            self._diagnostics.force_update()
            self.get_logger().error(f"[voice_tts] Missing binary: {e}")
            return False, f"missing binary: {e}"
        except Exception as e:
            self._speaking = False
            self._failure_count += 1
            self._last_result = f"TTS failed: {e}"
            self._diagnostics.force_update()
            self.get_logger().warn(f"[voice_tts] TTS failed: {e}")
            return False, f"TTS failed: {e}"

    def _produce_diagnostics(self, status: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        """Report TTS readiness and recent outcomes to ROS diagnostics."""
        # Section: Choose a health level from the active operation and accumulated failures.
        if self._speaking:
            status.summary(DiagnosticStatus.OK, "Speaking")
        elif self._last_result.startswith(("Missing binary", "TTS failed", "Piper returned")):
            status.summary(DiagnosticStatus.ERROR, self._last_result)
        elif self._last_result in ("Rejected empty text", "Piper model path is empty"):
            status.summary(DiagnosticStatus.WARN, self._last_result)
        else:
            status.summary(DiagnosticStatus.OK, self._last_result)

        # Section: Attach operator-facing configuration and counters.
        status.add("service", self.service_name)
        status.add("action", self.action_name)
        status.add("model", self.model_path)
        status.add("requests", str(self._request_count))
        status.add("failures", str(self._failure_count))
        return status

    def destroy_node(self) -> bool:
        """Special function: Release exclusive audio ownership before ROS resources are destroyed."""
        # Section: The file lock protects every supported TTS launch path on this host.
        release_audio_playback_lock(getattr(self, "_audio_playback_lock", None))
        self._audio_playback_lock = None
        return super().destroy_node()


def main(args=None):
    """Initialize and spin the standalone TTS ROS node."""
    # Section: Own the ROS context and action-server node for this process.
    rclpy.init(args=args)
    node = None
    try:
        # Section: Construct the server only after it acquires host-wide audio ownership.
        node = TTSNode()
        rclpy.spin(node)
    except RuntimeError as exc:
        # Section: A rejected secondary instance exits cleanly without advertising duplicate endpoints.
        rclpy.logging.get_logger("voice_tts").error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        # Section: Destroy action and node resources only when this process actually became the owner.
        if node is not None:
            node._say_action.destroy()
            node.destroy_node()
        # Section: Shut down the ROS context even when duplicate ownership rejected construction.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
