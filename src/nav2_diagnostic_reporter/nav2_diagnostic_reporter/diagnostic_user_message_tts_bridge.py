"""Bridge safe diagnostic user messages to the existing voice TTS service."""

# Section: Standard-library imports for a bounded non-blocking speech queue.
from collections import deque
from typing import Any

# Section: ROS message, node, and voice service imports.
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from voice_interfaces.srv import SayText


# Section: Standalone one-way diagnostic speech bridge.
class DiagnosticUserMessageTTSBridge(Node):
    """Speak only safe reporter messages through the existing SayText service."""

    # Section: Node construction and configurable one-topic-to-one-service wiring.
    def __init__(self) -> None:
        """Create a subscriber, asynchronous TTS client, and bounded queue timer."""

        super().__init__('diagnostic_user_message_tts_bridge')
        self.declare_parameter('user_message_topic', '/nav2_diagnostics/user_message')
        self.declare_parameter('tts_service_name', '/voice/tts/say')
        self.declare_parameter('dispatch_period_sec', 0.1)
        self.declare_parameter('max_queue_size', 5)
        self._speech_queue: deque[str] = deque()
        self._active_future: Any | None = None
        self._active_text = ''
        self._waiting_for_tts_logged = False
        self._subscription = self.create_subscription(
            String,
            str(self.get_parameter('user_message_topic').value),
            self._on_user_message,
            10,
        )
        self._tts_client = self.create_client(
            SayText,
            str(self.get_parameter('tts_service_name').value),
        )
        period = max(0.01, float(self.get_parameter('dispatch_period_sec').value))
        self._timer = self.create_timer(period, self._on_dispatch_timer)
        self.get_logger().info(
            'Diagnostic speech bridge ready: '
            f'topic={self.get_parameter("user_message_topic").value}, '
            f'tts={self.get_parameter("tts_service_name").value}')

    # Section: Safe String message intake with bounded memory behavior.
    def _on_user_message(self, message: String) -> None:
        """Queue a non-empty approved user message without blocking the subscriber callback."""

        text = str(message.data).strip()
        if not text:
            return
        max_queue_size = max(1, int(self.get_parameter('max_queue_size').value))
        if len(self._speech_queue) >= max_queue_size:
            self._speech_queue.popleft()
            self.get_logger().warning('Dropped oldest queued diagnostic message because the queue is full.')
        self._speech_queue.append(text)

    # Section: Non-blocking TTS dispatch loop.
    def _on_dispatch_timer(self) -> None:
        """Complete the active request or dispatch one queued message when TTS is ready."""

        if self._active_future is not None:
            self._finish_active_request()
            return
        if not self._speech_queue:
            return
        if not self._tts_client.service_is_ready():
            if not self._waiting_for_tts_logged:
                self.get_logger().warning('TTS service is unavailable; diagnostic speech remains queued.')
                self._waiting_for_tts_logged = True
            return
        self._waiting_for_tts_logged = False
        self._active_text = self._speech_queue.popleft()
        request = SayText.Request()
        request.text = self._active_text
        self._active_future = self._tts_client.call_async(request)

    # Section: Active TTS request completion handling.
    def _finish_active_request(self) -> None:
        """Release a completed request and record any TTS failure for the operator."""

        if self._active_future is None or not self._active_future.done():
            return
        try:
            response = self._active_future.result()
            if response is None or not response.success:
                self.get_logger().warning(
                    f'TTS rejected diagnostic message: {getattr(response, "message", "unknown error")}')
        except Exception as error:  # A service transport failure must not stop diagnostics.
            self.get_logger().warning(f'TTS failed for diagnostic message: {error}')
        self._active_future = None
        self._active_text = ''


# Section: Standalone executable entry point.
def main(args=None) -> None:
    """Initialize and spin the bridge without owning the voice server itself."""

    rclpy.init(args=args)
    node = DiagnosticUserMessageTTSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
