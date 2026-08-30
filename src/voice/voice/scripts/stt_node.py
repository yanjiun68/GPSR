#!/usr/bin/env python3

import os
import subprocess
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from faster_whisper import WhisperModel
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import Float32, String
from voice_interfaces.action import TranscribeSpeech
from voice_interfaces.srv import TranscribeAudio


class STTNode(Node):
    """
    Provide topic, service, and action speech transcription APIs.

    A Float32 trigger records once using its positive duration or the configured
    default, then publishes the transcript for compatibility with older clients.
    """

    def __init__(self):
        super().__init__("voice_stt")

        # Section: Keep health counters available to rqt Runtime Monitor.
        self._recording = False
        self._request_count = 0
        self._failure_count = 0
        self._last_result = "Waiting for a recording request"

        # Parameters
        self.declare_parameter("start_topic", "/voice/stt/start")
        self.declare_parameter("transcript_topic", "/voice/stt/transcript")
        self.declare_parameter("service_name", "/voice/stt/transcribe")
        self.declare_parameter("action_name", "/voice/stt/transcribe_action")
        self.declare_parameter("arecord_bin", "arecord")
        self.declare_parameter("mic_device", "default")
        self.declare_parameter("record_sec", 4.0)
        self.declare_parameter("wav_path", "/tmp/voice_stt.wav")
        self.declare_parameter("whisper_model", "base")
        self.declare_parameter("whisper_language", "en")

        self.start_topic = self.get_parameter("start_topic").get_parameter_value().string_value
        self.transcript_topic = (
            self.get_parameter("transcript_topic").get_parameter_value().string_value
        )
        self.service_name = self.get_parameter("service_name").get_parameter_value().string_value
        self.action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self.arecord_bin = self.get_parameter("arecord_bin").get_parameter_value().string_value
        self.mic_device = self.get_parameter("mic_device").get_parameter_value().string_value
        self.record_sec = float(
            self.get_parameter("record_sec").get_parameter_value().double_value
        )
        self.wav_path = self.get_parameter("wav_path").get_parameter_value().string_value
        self.whisper_model_name = (
            self.get_parameter("whisper_model").get_parameter_value().string_value
        )
        self.whisper_language = (
            self.get_parameter("whisper_language").get_parameter_value().string_value
        )

        self._whisper_model = None
        self._whisper_model_name = None

        # Comm
        self.create_subscription(Float32, self.start_topic, self._start_cb, 10)
        self.pub_transcript = self.create_publisher(String, self.transcript_topic, 10)

        # Section: Expose request/response and long-running action APIs for transcription clients.
        self._transcribe_service = self.create_service(
            TranscribeAudio,
            self.service_name,
            self._handle_transcribe_service,
        )
        self._transcribe_action = ActionServer(
            self,
            TranscribeSpeech,
            self.action_name,
            execute_callback=self._execute_transcribe_action,
            goal_callback=self._accept_transcribe_goal,
            cancel_callback=self._accept_transcribe_cancel,
        )

        # Section: Publish standard diagnostics and expose the node in topic graphs.
        self._diagnostics = Updater(self)
        self._diagnostics.setHardwareID("default_microphone")
        self._diagnostics.add("Speech-to-text", self._produce_diagnostics)
        self._diagnostics.force_update()

        self.get_logger().info(
            f"[voice_stt] Ready. Topic: {self.start_topic}; service: {self.service_name}; "
            f"action: {self.action_name}; transcript: {self.transcript_topic}"
        )

    # --- callbacks ---
    def _start_cb(self, msg: Float32):
        """Preserve the legacy topic trigger by running one transcription."""
        # Section: Convert the topic duration into the shared transcription request.
        duration = float(msg.data)
        success, _transcript, message, _wav_path = self._run_transcription(
            record_sec=duration if duration > 0 else self.record_sec,
            mic_device=self.mic_device,
            whisper_model=self.whisper_model_name,
            whisper_language=self.whisper_language,
            wav_path=self.wav_path,
        )
        if not success:
            self.get_logger().warning(f"[voice_stt] Topic request failed: {message}")

    def _handle_transcribe_service(
        self,
        request: TranscribeAudio.Request,
        response: TranscribeAudio.Response,
    ) -> TranscribeAudio.Response:
        """Record and return one transcript through a ROS service call."""
        # Section: Resolve optional request fields against node parameter defaults.
        success, transcript, message, wav_path = self._run_transcription(
            record_sec=float(request.record_sec) if request.record_sec > 0 else self.record_sec,
            mic_device=request.mic_device or self.mic_device,
            whisper_model=request.whisper_model or self.whisper_model_name,
            whisper_language=request.whisper_language,
            wav_path=request.wav_path or self.wav_path,
        )

        # Section: Copy the shared operation result into the generated service response.
        response.success = success
        response.transcript = transcript
        response.message = message
        response.wav_path = wav_path
        return response

    def _accept_transcribe_goal(self, _goal_request: TranscribeSpeech.Goal) -> GoalResponse:
        """Accept serialized STT goals for the local microphone."""
        return GoalResponse.ACCEPT

    def _accept_transcribe_cancel(self, _goal_handle) -> CancelResponse:
        """Accept cancellation requests at safe STT stage boundaries."""
        return CancelResponse.ACCEPT

    async def _execute_transcribe_action(self, goal_handle) -> TranscribeSpeech.Result:
        """Record speech, publish action progress, and return its transcript."""
        # Section: Publish accepted-goal feedback before the blocking microphone operation.
        feedback = TranscribeSpeech.Feedback()
        feedback.stage = "RECORDING"
        feedback.message = "Recording microphone audio"
        goal_handle.publish_feedback(feedback)

        # Section: Honor cancellation received before audio capture begins.
        result = TranscribeSpeech.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "Transcription canceled before recording"
            return result

        # Section: Resolve optional action goal values and run the shared transcription operation.
        goal = goal_handle.request
        success, transcript, message, wav_path = self._run_transcription(
            record_sec=float(goal.record_sec) if goal.record_sec > 0 else self.record_sec,
            mic_device=goal.mic_device or self.mic_device,
            whisper_model=goal.whisper_model or self.whisper_model_name,
            whisper_language=goal.whisper_language,
            wav_path=goal.wav_path or self.wav_path,
        )

        # Section: Complete the action with cancellation, success, or failure semantics.
        result.success = success
        result.transcript = transcript
        result.message = message
        result.wav_path = wav_path
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "Transcription canceled after the active audio stage"
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _run_transcription(
        self,
        record_sec: float,
        mic_device: str,
        whisper_model: str,
        whisper_language: str,
        wav_path: str,
    ) -> tuple[bool, str, str, str]:
        """Execute one shared record/transcribe operation for every ROS API."""
        # Section: Publish recording state through diagnostics before blocking on ALSA.
        self._request_count += 1
        self._recording = True
        self._last_result = "Recording audio"
        self._diagnostics.force_update()
        self.get_logger().info(f"[voice_stt] Recording for {record_sec:.1f} seconds...")

        # Section: Capture audio using request-specific values without mutating node defaults.
        recorded_wav = self._record(record_sec, mic_device, wav_path)
        if not recorded_wav:
            self._recording = False
            self._failure_count += 1
            self._last_result = "Audio recording failed"
            self._diagnostics.force_update()
            return False, "", self._last_result, wav_path

        # Section: Transcribe the recorded file and publish the compatibility transcript topic.
        transcript = self._transcribe(recorded_wav, whisper_model, whisper_language)
        self._recording = False
        if transcript:
            self._last_result = "Transcript published"
            message = "Transcription completed"
        else:
            self._failure_count += 1
            self._last_result = "Transcription returned no text"
            message = self._last_result
        self.pub_transcript.publish(String(data=transcript))
        self._diagnostics.force_update()
        self.get_logger().info(f"[voice_stt] Transcript: \"{transcript}\"")
        return bool(transcript), transcript, message, recorded_wav

    def _produce_diagnostics(self, status: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        """Report STT readiness and recent outcomes to ROS diagnostics."""
        # Section: Choose a health level from the current operation and most recent result.
        if self._recording:
            status.summary(DiagnosticStatus.OK, "Recording audio")
        elif self._last_result in ("Audio recording failed", "Transcription returned no text"):
            status.summary(DiagnosticStatus.WARN, self._last_result)
        else:
            status.summary(DiagnosticStatus.OK, self._last_result)

        # Section: Attach operator-facing configuration and counters.
        status.add("start_topic", self.start_topic)
        status.add("transcript_topic", self.transcript_topic)
        status.add("service", self.service_name)
        status.add("action", self.action_name)
        status.add("microphone_device", self.mic_device)
        status.add("requests", str(self._request_count))
        status.add("failures", str(self._failure_count))
        return status

    # --- audio capture ---
    def _record(self, record_sec: float, mic_device: str, wav_path: str) -> Optional[str]:
        """Capture one mono WAV file from the requested ALSA device."""
        # Section: Ensure the selected output directory exists before invoking arecord.
        wav_directory = os.path.dirname(wav_path) or "."
        os.makedirs(wav_directory, exist_ok=True)
        cmd = [
            self.arecord_bin,
            "-D",
            mic_device,
            "-f",
            "S16_LE",
            "-c",
            "1",
            "-r",
            "16000",
            "-d",
            str(int(max(1.0, record_sec))),
            wav_path,
        ]

        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=record_sec + 5.0)
            if r.returncode != 0:
                self.get_logger().warn(f"[voice_stt] arecord rc={r.returncode}: {r.stderr[:200]}")
                return None
            return wav_path
        except FileNotFoundError as e:
            self.get_logger().error(f"[voice_stt] Missing arecord: {e}")
            return None
        except Exception as e:
            self.get_logger().warn(f"[voice_stt] Recording failed: {e}")
            return None

    # --- whisper ---
    def _load_whisper(self, whisper_model: str):
        """Load and cache the requested faster-whisper model lazily."""
        # Section: Replace the cached model only when the requested model name changes.
        if self._whisper_model is None or self._whisper_model_name != whisper_model:
            self.get_logger().info(f"[voice_stt] Loading faster-whisper model: {whisper_model}")
            self._whisper_model = WhisperModel(
                whisper_model,
                device="cpu",
                compute_type="int8",
            )
            self._whisper_model_name = whisper_model
        return self._whisper_model

    def _transcribe(self, wav_path: str, whisper_model: str, whisper_language: str) -> str:
        """Convert one recorded WAV file into normalized transcript text."""
        # Section: Reject missing recordings before loading the inference model.
        if not os.path.exists(wav_path):
            return ""
        try:
            model = self._load_whisper(whisper_model)
            segments, _ = model.transcribe(
                wav_path,
                language=whisper_language if whisper_language else None,
                vad_filter=False,
                beam_size=5,
            )
            return " ".join(segment.text.strip() for segment in segments).strip()
        except Exception as e:
            self.get_logger().warn(f"[voice_stt] Transcription failed: {e}")
            return ""


def main(args=None):
    """Initialize and spin the standalone STT ROS node."""
    # Section: Own the ROS context and action-server node for this process.
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._transcribe_action.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
