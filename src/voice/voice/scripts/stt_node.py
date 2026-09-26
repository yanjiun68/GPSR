#!/usr/bin/env python3

# Section: POSIX file locking prevents multiple STT processes from competing for one microphone.
import fcntl
import os
import subprocess
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Optional, TextIO

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from faster_whisper import WhisperModel
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import Float32, String
from voice_interfaces.action import TranscribeSpeech
from voice_interfaces.srv import TranscribeAudio

try:
    # Section: Import WebRTC VAD only for the optional continuous utterance listener.
    import webrtcvad
except ImportError:
    # Section: Keep the existing on-demand STT APIs usable when the optional dependency is absent.
    webrtcvad = None


# Section: All STT launch paths share this host-local microphone ownership lease.
MICROPHONE_CAPTURE_LOCK_PATH = Path("/tmp/putrabot_microphone_capture.lock")


def acquire_microphone_capture_lock() -> tuple[TextIO | None, str]:
    """Special function: Claim exclusive host microphone ownership for this STT server."""
    # Section: Open a stable lock file whose advisory kernel lock clears when its process exits.
    lock_handle = MICROPHONE_CAPTURE_LOCK_PATH.open("a+", encoding="utf-8")
    try:
        # Section: Reject competing STT servers immediately instead of allowing concurrent ALSA capture.
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        # Section: Report the existing owner PID so an operator can find the conflicting launch.
        lock_handle.seek(0)
        owner_pid = lock_handle.read().strip() or "unknown"
        lock_handle.close()
        return None, owner_pid
    # Section: Update diagnostic ownership only after this process has acquired the exclusive lease.
    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write(str(os.getpid()))
    lock_handle.flush()
    return lock_handle, str(os.getpid())


def release_microphone_capture_lock(lock_handle: TextIO | None) -> None:
    """Special function: Release the microphone lease during normal STT shutdown."""
    # Section: A missing handle means startup was rejected before this process owned microphone capture.
    if lock_handle is None:
        return
    try:
        # Section: Unlock before close so a replacement STT process can start immediately.
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
    finally:
        # Section: Closing guarantees kernel cleanup even if explicit unlock cannot report success.
        lock_handle.close()


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
        # Section: Keep continuous VAD transcription opt-in so existing launch behavior is unchanged.
        self.declare_parameter("enable_vad_utterance", False)
        self.declare_parameter("vad_partial_topic", "/voice/stt/partial_transcript")
        self.declare_parameter("vad_final_topic", "/voice/stt/utterance_transcript")
        self.declare_parameter("vad_whisper_model", "small.en")
        self.declare_parameter("vad_compute_type", "float16")
        self.declare_parameter("vad_device", "cuda")
        self.declare_parameter("vad_frame_ms", 100)
        self.declare_parameter("vad_partial_update_ms", 900)
        self.declare_parameter("vad_rolling_context_sec", 5.0)
        self.declare_parameter("vad_end_silence_ms", 600)
        self.declare_parameter("vad_mode", 2)

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
        self.enable_vad_utterance = bool(self.get_parameter("enable_vad_utterance").value)
        self.vad_partial_topic = self.get_parameter("vad_partial_topic").value
        self.vad_final_topic = self.get_parameter("vad_final_topic").value
        self.vad_whisper_model_name = self.get_parameter("vad_whisper_model").value
        self.vad_compute_type = self.get_parameter("vad_compute_type").value
        self.vad_device = self.get_parameter("vad_device").value
        self.vad_frame_ms = int(self.get_parameter("vad_frame_ms").value)
        self.vad_partial_update_ms = int(self.get_parameter("vad_partial_update_ms").value)
        self.vad_rolling_context_sec = float(self.get_parameter("vad_rolling_context_sec").value)
        self.vad_end_silence_ms = int(self.get_parameter("vad_end_silence_ms").value)
        self.vad_mode = int(self.get_parameter("vad_mode").value)

        # Section: Refuse a duplicate STT server before registering duplicate topic, service, and action endpoints.
        self._microphone_capture_lock, owner_pid = acquire_microphone_capture_lock()
        if self._microphone_capture_lock is None:
            self.get_logger().error(
                "[voice_stt] Microphone capture is already owned by PID "
                f"{owner_pid}; do not start a second STT server. "
                "Set start_voice_stt:=false on this secondary launch."
            )
            raise RuntimeError("duplicate microphone capture owner")

        # Section: Initialize model and VAD state only after this process owns microphone capture.
        self._whisper_model = None
        self._whisper_model_name = None
        # Section: Use an isolated model cache because the VAD feature has its own GPU/precision settings.
        self._vad_whisper_model = None
        self._vad_whisper_model_name = None
        self._vad_whisper_model_config = None
        self._vad_stop_event = threading.Event()
        self._vad_thread = None
        self._vad_process = None
        self._vad_utterance_active = False
        self._vad_utterance_id = 0
        self._vad_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="voice_vad_stt")

        # Comm
        self.create_subscription(Float32, self.start_topic, self._start_cb, 10)
        self.pub_transcript = self.create_publisher(String, self.transcript_topic, 10)
        # Section: Keep VAD partial and final results separate from the legacy one-shot transcript topic.
        self.pub_vad_partial = self.create_publisher(String, self.vad_partial_topic, 10)
        self.pub_vad_final = self.create_publisher(String, self.vad_final_topic, 10)

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

        # Section: Start microphone streaming only when explicitly requested by the launch configuration.
        if self.enable_vad_utterance:
            self._start_vad_utterance_listener()

        self.get_logger().info(
            f"[voice_stt] Ready. Topic: {self.start_topic}; service: {self.service_name}; "
            f"action: {self.action_name}; transcript: {self.transcript_topic}; "
            f"VAD utterance listener: {'enabled' if self.enable_vad_utterance else 'disabled'}"
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
            # Section: Use the configured English default when service clients omit their optional language field.
            whisper_language=request.whisper_language or self.whisper_language,
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
        # Section: Prevent two arecord processes from contending for the same microphone device.
        if self.enable_vad_utterance:
            return (
                False,
                "",
                "One-shot STT is unavailable while enable_vad_utterance is true.",
                wav_path,
            )

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

    # --- continuous VAD utterance transcription ---
    def _start_vad_utterance_listener(self) -> None:
        """Validate and start the optional continuous VAD microphone listener."""
        # Section: WebRTC VAD accepts 10/20/30 ms input, so each requested 100 ms decision is split below.
        if webrtcvad is None:
            self.get_logger().error(
                "[voice_stt] VAD utterance listener requires the webrtcvad Python package."
            )
            self.enable_vad_utterance = False
            return
        if self.vad_frame_ms != 100:
            self.get_logger().error(
                "[voice_stt] vad_frame_ms must be 100 to preserve the configured utterance timing."
            )
            self.enable_vad_utterance = False
            return
        if not 0 <= self.vad_mode <= 3:
            self.get_logger().error("[voice_stt] vad_mode must be in the range 0 through 3.")
            self.enable_vad_utterance = False
            return

        # Section: Run blocking ALSA reads outside the ROS executor so subscriptions and actions remain responsive.
        self._vad_thread = threading.Thread(
            target=self._vad_capture_loop,
            name="voice_vad_capture",
            daemon=True,
        )
        self._vad_thread.start()

    def _vad_capture_loop(self) -> None:
        """Read 16 kHz PCM frames, group speech into utterances, and dispatch transcripts."""
        # Section: Keep VAD state local to its single capture thread to avoid cross-thread audio races.
        vad = webrtcvad.Vad(self.vad_mode)
        frame_bytes = 16000 * self.vad_frame_ms // 1000 * 2
        context_frames = max(1, round(self.vad_rolling_context_sec * 1000 / self.vad_frame_ms))
        state = {
            "active_pcm": bytearray(),
            "partial_window": deque(maxlen=context_frames),
            "pre_roll": deque(maxlen=3),
            "silence_ms": 0,
            "last_partial_at": 0.0,
            "utterance_id": 0,
        }
        command = [
            self.arecord_bin,
            "-q",
            "-D",
            self.mic_device,
            "-t",
            "raw",
            "-f",
            "S16_LE",
            "-c",
            "1",
            "-r",
            "16000",
        ]

        try:
            # Section: Stream raw PCM so utterance boundaries are decided before Whisper runs.
            self._vad_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )
            pending = b""
            while not self._vad_stop_event.is_set():
                chunk = self._vad_process.stdout.read(frame_bytes)
                if not chunk:
                    break
                pending += chunk
                while len(pending) >= frame_bytes:
                    frame, pending = pending[:frame_bytes], pending[frame_bytes:]
                    self._consume_vad_frame(frame, vad, state)
        except FileNotFoundError:
            self.get_logger().error(f"[voice_stt] Missing arecord executable: {self.arecord_bin}")
        except Exception as e:
            self.get_logger().error(f"[voice_stt] VAD audio capture failed: {e}")
        finally:
            # Section: Mark the listener idle before process teardown so queued partials cannot publish stale text.
            self._vad_utterance_active = False
            if self._vad_process is not None and self._vad_process.poll() is None:
                self._vad_process.terminate()
            self._vad_process = None

    def _consume_vad_frame(self, frame: bytes, vad, state: dict) -> None:
        """Use one 100 ms VAD decision to update the active utterance state."""
        # Section: Split the 100 ms policy frame into ten WebRTC-compatible 10 ms subframes.
        subframe_bytes = 16000 * 10 // 1000 * 2
        speech = any(
            vad.is_speech(frame[offset:offset + subframe_bytes], 16000)
            for offset in range(0, len(frame), subframe_bytes)
        )
        now = time.monotonic()

        # Section: Retain a short leading margin to avoid cutting the first phoneme of a new utterance.
        if not self._vad_utterance_active:
            if not speech:
                state["pre_roll"].append(frame)
                return
            state["utterance_id"] += 1
            self._vad_utterance_id = state["utterance_id"]
            self._vad_utterance_active = True
            state["active_pcm"] = bytearray(b"".join(state["pre_roll"]))
            state["active_pcm"].extend(frame)
            state["partial_window"].clear()
            state["partial_window"].extend((*state["pre_roll"], frame))
            state["pre_roll"].clear()
            state["silence_ms"] = 0
            state["last_partial_at"] = now
            self.get_logger().info(
                f"[voice_stt] VAD detected utterance {state['utterance_id']}."
            )
            return

        # Section: Accumulate every active frame so the final result contains the complete utterance.
        state["active_pcm"].extend(frame)
        state["partial_window"].append(frame)
        state["silence_ms"] = 0 if speech else state["silence_ms"] + self.vad_frame_ms

        # Section: Transcribe only the rolling 5-second context at roughly 900 ms intervals for low-latency previews.
        elapsed_ms = (now - state["last_partial_at"]) * 1000.0
        if speech and elapsed_ms >= self.vad_partial_update_ms:
            state["last_partial_at"] = now
            self._vad_executor.submit(
                self._publish_vad_partial,
                state["utterance_id"],
                b"".join(state["partial_window"]),
            )

        # Section: Finish only after 600 ms of silence and submit the complete utterance for final transcription.
        if state["silence_ms"] >= self.vad_end_silence_ms:
            utterance_id = state["utterance_id"]
            complete_pcm = bytes(state["active_pcm"])
            self._vad_utterance_active = False
            state["pre_roll"].clear()
            state["silence_ms"] = 0
            self._vad_executor.submit(self._publish_vad_final, utterance_id, complete_pcm)

    def _load_vad_whisper(self):
        """Load the exact optional VAD listener model and floating-point compute configuration."""
        # Section: Cache independently so normal base-model requests cannot replace small.en float16 inference.
        config = (self.vad_whisper_model_name, self.vad_device, self.vad_compute_type)
        if self._vad_whisper_model is None or self._vad_whisper_model_config != config:
            self.get_logger().info(
                "[voice_stt] Loading VAD faster-whisper model: "
                f"{self.vad_whisper_model_name} ({self.vad_device}, {self.vad_compute_type})"
            )
            self._vad_whisper_model = WhisperModel(
                self.vad_whisper_model_name,
                device=self.vad_device,
                compute_type=self.vad_compute_type,
            )
            self._vad_whisper_model_name = self.vad_whisper_model_name
            self._vad_whisper_model_config = config
        return self._vad_whisper_model

    def _transcribe_vad_pcm(self, pcm_data: bytes) -> str:
        """Convert raw utterance PCM into text using the VAD listener's small.en float16 model."""
        # Section: Convert exactly the 16-bit mono stream supplied by arecord into faster-whisper float audio.
        audio = np.frombuffer(pcm_data, dtype=np.int16).astype(np.float32) / 32768.0
        if not audio.size:
            return ""
        try:
            model = self._load_vad_whisper()
            segments, _ = model.transcribe(
                audio,
                language="en",
                vad_filter=False,
                beam_size=5,
                condition_on_previous_text=False,
            )
            return " ".join(segment.text.strip() for segment in segments).strip()
        except Exception as e:
            self.get_logger().error(f"[voice_stt] VAD transcription failed: {e}")
            return ""

    def _publish_vad_partial(self, utterance_id: int, pcm_data: bytes) -> None:
        """Publish a rolling-context preview only while its utterance is still active."""
        # Section: Drop delayed executor work after a newer utterance or final boundary has superseded it.
        if not self._vad_utterance_active or utterance_id != self._vad_utterance_id:
            return
        transcript = self._transcribe_vad_pcm(pcm_data)
        if transcript and self._vad_utterance_active and utterance_id == self._vad_utterance_id:
            self.pub_vad_partial.publish(String(data=transcript))

    def _publish_vad_final(self, utterance_id: int, pcm_data: bytes) -> None:
        """Publish one completed utterance after its configured end-of-speech silence."""
        # Section: Final results are never truncated to the preview context window.
        transcript = self._transcribe_vad_pcm(pcm_data)
        if transcript:
            self.pub_vad_final.publish(String(data=transcript))
            self.get_logger().info(
                f"[voice_stt] VAD utterance {utterance_id} final: \"{transcript}\""
            )

    def destroy_node(self):
        """Special function: Stop STT resources and release microphone ownership before ROS teardown."""
        # Section: Terminate the blocking arecord reader so shutdown does not leave the ALSA device occupied.
        self._vad_stop_event.set()
        if self._vad_process is not None and self._vad_process.poll() is None:
            self._vad_process.terminate()
        if self._vad_thread is not None:
            self._vad_thread.join(timeout=2.0)
        self._vad_executor.shutdown(wait=False, cancel_futures=True)
        # Section: Release the host-wide microphone lease only after all capture resources stop.
        release_microphone_capture_lock(getattr(self, "_microphone_capture_lock", None))
        self._microphone_capture_lock = None
        return super().destroy_node()


def main(args=None):
    """Initialize and spin the standalone STT ROS node."""
    # Section: Own the ROS context and action-server node for this process.
    rclpy.init(args=args)
    node = None
    try:
        # Section: Construct the server only after it obtains exclusive microphone ownership.
        node = STTNode()
        rclpy.spin(node)
    except RuntimeError as exc:
        # Section: A rejected secondary instance exits without subscribing to or recording from the microphone.
        rclpy.logging.get_logger("voice_stt").error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        # Section: Destroy action and node resources only when this process became the STT owner.
        if node is not None:
            node._transcribe_action.destroy()
            node.destroy_node()
        # Section: Shut down ROS after normal exit or rejected duplicate startup.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
