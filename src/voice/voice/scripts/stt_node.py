#!/usr/bin/env python3

import os
import subprocess
from typing import Optional

import rclpy
from faster_whisper import WhisperModel
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32


class STTNode(Node):
    """
    Thin STT wrapper: when it receives a Float32 on /voice/stt/start, it records once
    (using the provided duration if >0, otherwise the current record_sec) and
    publishes the transcript to /voice/stt/transcript. No decision-making.
    """

    def __init__(self):
        super().__init__("voice_stt")

        # Parameters
        self.declare_parameter("start_topic", "/voice/stt/start")
        self.declare_parameter("transcript_topic", "/voice/stt/transcript")
        self.declare_parameter("arecord_bin", "arecord")
        self.declare_parameter("mic_device", "default")
        self.declare_parameter("record_sec", 4.0)
        self.declare_parameter("wav_path", "/tmp/voice_stt.wav")
        self.declare_parameter("whisper_model", "base")
        self.declare_parameter("whisper_language", "en")

        self.start_topic = self.get_parameter("start_topic").get_parameter_value().string_value
        self.transcript_topic = self.get_parameter("transcript_topic").get_parameter_value().string_value
        self.arecord_bin = self.get_parameter("arecord_bin").get_parameter_value().string_value
        self.mic_device = self.get_parameter("mic_device").get_parameter_value().string_value
        self.record_sec = float(self.get_parameter("record_sec").get_parameter_value().double_value)
        self.wav_path = self.get_parameter("wav_path").get_parameter_value().string_value
        self.whisper_model_name = self.get_parameter("whisper_model").get_parameter_value().string_value
        self.whisper_language = self.get_parameter("whisper_language").get_parameter_value().string_value

        self._whisper_model = None
        self._whisper_model_name = None

        # Comm
        self.create_subscription(Float32, self.start_topic, self._start_cb, 10)
        self.pub_transcript = self.create_publisher(String, self.transcript_topic, 10)

        self.get_logger().info(
            f"[voice_stt] Ready. Trigger: {self.start_topic} -> publish transcript on {self.transcript_topic}"
        )

    # --- callbacks ---
    def _start_cb(self, msg: Float32):
        duration = float(msg.data)
        if duration > 0:
            self.record_sec = max(0.5, duration)
            self.get_logger().info(f"[voice_stt] record_sec set to {self.record_sec:.1f}s via trigger")

        self.get_logger().info("[voice_stt] Trigger received, recording...")
        wav = self._record()
        if not wav:
            self.get_logger().warn("[voice_stt] Recording failed; skipping STT.")
            return

        transcript = self._transcribe(wav)
        self.pub_transcript.publish(String(data=transcript))
        self.get_logger().info(f"[voice_stt] Transcript: \"{transcript}\"")

    # --- audio capture ---
    def _record(self) -> Optional[str]:
        os.makedirs(os.path.dirname(self.wav_path), exist_ok=True)
        cmd = [
            self.arecord_bin,
            "-D",
            self.mic_device,
            "-f",
            "S16_LE",
            "-c",
            "1",
            "-r",
            "16000",
            "-d",
            str(int(max(1.0, self.record_sec))),
            self.wav_path,
        ]

        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=self.record_sec + 5.0)
            if r.returncode != 0:
                self.get_logger().warn(f"[voice_stt] arecord rc={r.returncode}: {r.stderr[:200]}")
                return None
            return self.wav_path
        except FileNotFoundError as e:
            self.get_logger().error(f"[voice_stt] Missing arecord: {e}")
            return None
        except Exception as e:
            self.get_logger().warn(f"[voice_stt] Recording failed: {e}")
            return None

    # --- whisper ---
    def _load_whisper(self):
        if self._whisper_model is None or self._whisper_model_name != self.whisper_model_name:
            self.get_logger().info(f"[voice_stt] Loading faster-whisper model: {self.whisper_model_name}")
            self._whisper_model = WhisperModel(
                self.whisper_model_name,
                device="cpu",
                compute_type="int8",
            )
            self._whisper_model_name = self.whisper_model_name
        return self._whisper_model

    def _transcribe(self, wav_path: str) -> str:
        if not os.path.exists(wav_path):
            return ""
        try:
            model = self._load_whisper()
            segments, _ = model.transcribe(
                wav_path,
                language=self.whisper_language if self.whisper_language else None,
                vad_filter=False,
                beam_size=5,
            )
            return " ".join(segment.text.strip() for segment in segments).strip()
        except Exception as e:
            self.get_logger().warn(f"[voice_stt] Transcription failed: {e}")
            return ""


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
