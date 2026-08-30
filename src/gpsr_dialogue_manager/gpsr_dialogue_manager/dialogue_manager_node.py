from __future__ import annotations

from enum import Enum, auto
import json
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String

from voice_interfaces.action import SpeakText, TranscribeSpeech

from .dialogue_router import DialogueAct, classify_dialogue_act


class DialogueState(Enum):
    IDLE = auto()
    SPEAKING_PROMPT = auto()
    LISTENING = auto()
    ROUTING = auto()
    WAITING_FIXED_INFO = auto()
    SPEAKING_ANSWER = auto()


class DialogueManagerNode(Node):
    """
    P0 dialogue orchestration.

    Production flow:
      TTS prompt -> wait -> STT action -> route
      -> fixed_information topic request/response
      -> TTS answer

    This node intentionally does not subscribe to /voice/stt/transcript.
    It consumes the TranscribeSpeech action result to avoid processing
    one utterance twice.
    """

    def __init__(self) -> None:
        super().__init__("gpsr_dialogue_manager")

        self.declare_parameter("listen_duration_sec", 5.0)
        self.declare_parameter("fixed_info_timeout_sec", 2.0)
        self.declare_parameter("tts_server_timeout_sec", 2.0)
        self.declare_parameter("stt_server_timeout_sec", 2.0)
        self.declare_parameter("prompt", "What would you like to ask me?")
        self.declare_parameter("language", "en")

        self.tts = ActionClient(self, SpeakText, "/voice/tts/say_action")
        self.stt = ActionClient(self, TranscribeSpeech, "/voice/stt/transcribe_action")

        self.fixed_query_pub = self.create_publisher(
            String, "/fixed_information/query", 10
        )
        self.fixed_answer_sub = self.create_subscription(
            String, "/fixed_information/answer", self._on_fixed_answer, 10
        )

        self.task_command_pub = self.create_publisher(
            String, "/dialogue/task_command", 10
        )
        self.unhandled_question_pub = self.create_publisher(
            String, "/dialogue/unhandled_question", 10
        )
        self.dialogue_status_pub = self.create_publisher(
            String, "/dialogue/status", 10
        )

        self.start_sub = self.create_subscription(
            Empty, "/dialogue/start", self._on_start, 10
        )

        self.state = DialogueState.IDLE
        self.pending_request_id = None
        self.pending_question = None
        self.fixed_deadline = None

        self.watchdog = self.create_timer(0.1, self._check_timeout)
        self._publish_status("IDLE")
        self.get_logger().info("P0 GPSR dialogue manager ready.")

    # -------------------- Status helpers --------------------

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.dialogue_status_pub.publish(msg)

    def _set_state(self, state: DialogueState) -> None:
        self.state = state
        self._publish_status(state.name)

    def _clear_pending(self) -> None:
        self.pending_request_id = None
        self.pending_question = None
        self.fixed_deadline = None

    def _return_idle(self) -> None:
        self._clear_pending()
        self._set_state(DialogueState.IDLE)

    # -------------------- Turn entry --------------------

    def _on_start(self, _msg: Empty) -> None:
        if self.state != DialogueState.IDLE:
            self.get_logger().warn(
                f"Dialogue turn rejected because state={self.state.name}"
            )
            return

        self._set_state(DialogueState.SPEAKING_PROMPT)
        prompt = str(self.get_parameter("prompt").value)
        self._speak(prompt, self._start_listening)

    # -------------------- TTS --------------------

    def _speak(self, text: str, on_complete=None) -> None:
        text = str(text).strip()
        if not text:
            self.get_logger().error("Refusing to send empty TTS text.")
            self._return_idle()
            return

        timeout = float(self.get_parameter("tts_server_timeout_sec").value)
        if not self.tts.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("TTS action server unavailable.")
            self._return_idle()
            return

        goal = SpeakText.Goal()
        goal.text = text

        future = self.tts.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_tts_goal_response(f, on_complete)
        )

    def _on_tts_goal_response(self, future, on_complete) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"TTS send-goal failed: {exc}")
            self._return_idle()
            return

        if not goal_handle.accepted:
            self.get_logger().error("TTS goal rejected.")
            self._return_idle()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_tts_finished(f, on_complete)
        )

    def _on_tts_finished(self, future, on_complete) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            if hasattr(result, "success") and not result.success:
                self.get_logger().error(
                    f"TTS failed: {getattr(result, 'message', '')}"
                )
                self._return_idle()
                return
        except Exception as exc:
            self.get_logger().error(f"TTS result failed: {exc}")
            self._return_idle()
            return

        # Critical P0 sequencing rule: STT is only started from this callback,
        # after the TTS action has completed.
        if on_complete is not None:
            on_complete()
        else:
            self._return_idle()

    # -------------------- STT --------------------

    def _start_listening(self) -> None:
        self._set_state(DialogueState.LISTENING)

        timeout = float(self.get_parameter("stt_server_timeout_sec").value)
        if not self.stt.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("STT action server unavailable.")
            self._return_idle()
            return

        goal = TranscribeSpeech.Goal()
        goal.record_sec = float(self.get_parameter("listen_duration_sec").value)
        goal.mic_device = ""
        goal.whisper_model = ""
        goal.whisper_language = str(self.get_parameter("language").value)
        goal.wav_path = ""

        future = self.stt.send_goal_async(goal)
        future.add_done_callback(self._on_stt_goal_response)

    def _on_stt_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"STT send-goal failed: {exc}")
            self._return_idle()
            return

        if not goal_handle.accepted:
            self.get_logger().error("STT goal rejected.")
            self._return_idle()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_stt_finished)

    def _on_stt_finished(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self.get_logger().error(f"STT result failed: {exc}")
            self._return_idle()
            return

        if not result.success:
            self._set_state(DialogueState.SPEAKING_ANSWER)
            self._speak(
                "I could not understand you.",
                self._return_idle,
            )
            return

        transcript = str(result.transcript).strip()
        if not transcript:
            self._set_state(DialogueState.SPEAKING_ANSWER)
            self._speak(
                "I did not hear anything.",
                self._return_idle,
            )
            return

        self.get_logger().info(f"Heard: {transcript}")
        self._route_utterance(transcript)

    # -------------------- Routing --------------------

    def _route_utterance(self, text: str) -> None:
        self._set_state(DialogueState.ROUTING)
        act = classify_dialogue_act(text)

        if act == DialogueAct.QUESTION:
            self._query_fixed_information(text)
            return

        if act == DialogueAct.TASK_COMMAND:
            msg = String()
            msg.data = text
            self.task_command_pub.publish(msg)
            self.get_logger().info("Routed utterance to /dialogue/task_command")
            self._return_idle()
            return

        if act == DialogueAct.CANCEL:
            self._set_state(DialogueState.SPEAKING_ANSWER)
            self._speak("Okay.", self._return_idle)
            return

        # Confirmation/correction/unknown routing is intentionally left to the
        # higher-level dialogue layer in later phases.
        msg = String()
        msg.data = text
        self.unhandled_question_pub.publish(msg)
        self.get_logger().info(
            f"Routed {act.name} utterance to /dialogue/unhandled_question"
        )
        self._return_idle()

    # -------------------- Fixed-information provider --------------------

    def _query_fixed_information(self, question: str) -> None:
        self._set_state(DialogueState.WAITING_FIXED_INFO)

        request_id = str(uuid.uuid4())
        self.pending_request_id = request_id
        self.pending_question = question

        timeout = float(self.get_parameter("fixed_info_timeout_sec").value)
        self.fixed_deadline = time.monotonic() + timeout

        payload = {
            "request_id": request_id,
            "text": question,
            "context": {
                "source": "gpsr_dialogue_manager",
            },
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.fixed_query_pub.publish(msg)

    def _on_fixed_answer(self, msg: String) -> None:
        if self.state != DialogueState.WAITING_FIXED_INFO:
            return

        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Ignoring invalid fixed-information JSON.")
            return

        if result.get("request_id") != self.pending_request_id:
            # A different requester may also use the provider.
            return

        status = str(result.get("status", "")).upper()

        if status == "FOUND":
            answer = str(result.get("answer_text") or "").strip()
            self._clear_pending()
            if not answer:
                self.get_logger().error(
                    "Fixed-information response was FOUND but answer_text was empty."
                )
                self._return_idle()
                return

            self._set_state(DialogueState.SPEAKING_ANSWER)
            self._speak(answer, self._return_idle)
            return

        if status == "AMBIGUOUS":
            self._clear_pending()
            self._set_state(DialogueState.SPEAKING_ANSWER)
            self._speak(
                "Could you please rephrase your question?",
                self._return_idle,
            )
            return

        if status in {"NO_MATCH", "NOT_CONFIGURED"}:
            question = self.pending_question or ""
            self._clear_pending()

            out = String()
            out.data = question
            self.unhandled_question_pub.publish(out)
            self.get_logger().info(
                f"Fixed provider returned {status}; routed question onward."
            )
            self._return_idle()
            return

        self.get_logger().warn(f"Unknown fixed-information status: {status}")

    def _check_timeout(self) -> None:
        if self.state != DialogueState.WAITING_FIXED_INFO:
            return
        if self.fixed_deadline is None:
            return
        if time.monotonic() < self.fixed_deadline:
            return

        question = self.pending_question or ""
        self.get_logger().error("Fixed-information query timed out.")
        self._clear_pending()

        if question:
            msg = String()
            msg.data = question
            self.unhandled_question_pub.publish(msg)

        self._return_idle()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DialogueManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
