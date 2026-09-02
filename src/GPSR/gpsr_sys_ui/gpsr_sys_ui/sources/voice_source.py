"""Section: Structured Voice STT/TTS monitoring without audio ownership."""

from __future__ import annotations

import time
from typing import Any

from ..event_model import MonitorEvent


# Section: Verified voice graph identities from the installed voice package.
STT_NODE = "/voice_stt"
TTS_NODE = "/voice_tts"


def _full_node_name(name: str, namespace: str) -> str:
    """Special function: Normalize a ROS graph name and namespace."""
    prefix = str(namespace or "").rstrip("/")
    return f"{prefix}/{name}" if prefix else f"/{name}"


class VoiceStateModel:
    """Special function: Maintain hardware-free voice presence, activity, and transcript state."""

    def __init__(self) -> None:
        """Special function: Initialize voice components as absent without failing startup."""
        self.stt_present = False
        self.tts_present = False
        self.stt_diagnostic: dict[str, Any] | None = None
        self.tts_diagnostic: dict[str, Any] | None = None
        self.transcript = ""
        self.transcript_timestamp = 0.0

    def set_presence(self, stt_present: bool, tts_present: bool) -> None:
        """Special function: Update independently discovered STT and TTS node presence."""
        self.stt_present = bool(stt_present)
        self.tts_present = bool(tts_present)

    def set_diagnostics(
        self,
        stt_diagnostic: dict[str, Any] | None,
        tts_diagnostic: dict[str, Any] | None,
    ) -> None:
        """Special function: Store only diagnostics emitted by the voice owners."""
        self.stt_diagnostic = dict(stt_diagnostic) if stt_diagnostic else None
        self.tts_diagnostic = dict(tts_diagnostic) if tts_diagnostic else None

    def set_transcript(self, transcript: str, timestamp: float | None = None) -> None:
        """Special function: Record the actual STT transcript and local receipt time."""
        self.transcript = str(transcript)
        self.transcript_timestamp = time.time() if timestamp is None else float(timestamp)

    @staticmethod
    def _component(present: bool, diagnostic: dict[str, Any] | None, kind: str) -> dict[str, Any]:
        """Special function: Normalize one independently monitored voice component."""
        if not present:
            return {"present": False, "health": "MISSING", "state": "MISSING", "message": "node not discovered"}
        if diagnostic is None:
            return {"present": True, "health": "WARN", "state": "IDLE", "message": "diagnostics not received"}
        message = str(diagnostic.get("message", ""))
        lowered = message.lower()
        if kind == "stt" and "recording" in lowered:
            state = "LISTENING"
        elif kind == "stt" and ("processing" in lowered or "transcrib" in lowered):
            state = "PROCESSING"
        elif kind == "tts" and "speaking" in lowered:
            state = "SPEAKING"
        else:
            state = "IDLE"
        return {
            "present": True,
            "health": str(diagnostic.get("health", "UNKNOWN")).upper(),
            "state": state,
            "message": message,
            "hardware_id": str(diagnostic.get("hardware_id", "")),
            "last_update": float(diagnostic.get("last_update", 0.0) or 0.0),
        }

    def snapshot(self, now: float | None = None) -> dict[str, Any]:
        """Special function: Return UI-ready state without marking old transcripts unhealthy."""
        current_time = time.time() if now is None else float(now)
        age = max(0.0, current_time - self.transcript_timestamp) if self.transcript_timestamp else None
        return {
            "stt": self._component(self.stt_present, self.stt_diagnostic, "stt"),
            "tts": self._component(self.tts_present, self.tts_diagnostic, "tts"),
            "transcript": {
                "text": self.transcript,
                "source_topic": "/voice/stt/transcript",
                "timestamp": self.transcript_timestamp,
                "age_sec": age,
            },
        }


class VoicePresenceTracker:
    """Special function: Convert voice graph changes into deduplicated transition events."""

    def __init__(self) -> None:
        """Special function: Initialize without treating normal startup absence as disappearance."""
        self.previous: tuple[bool, bool] | None = None

    def update(self, presence: tuple[bool, bool]) -> list[MonitorEvent]:
        """Special function: Return events only when STT or TTS presence changes."""
        emitted: list[MonitorEvent] = []
        if self.previous is not None:
            labels = (("STT", STT_NODE), ("TTS", TTS_NODE))
            for index, (label, node_name) in enumerate(labels):
                if self.previous[index] and not presence[index]:
                    emitted.append(MonitorEvent("ERROR", "VOICE", f"VOICE_{label}_MISSING", "voice node disappeared", node=node_name))
                elif not self.previous[index] and presence[index]:
                    emitted.append(MonitorEvent("INFO", "VOICE", f"VOICE_{label}_AVAILABLE", "voice node discovered", node=node_name))
        self.previous = presence
        return emitted

class VoiceSource:
    """Special function: Observe voice graph, diagnostics, and transcript interfaces."""

    def __init__(self, node, cache, events, transcript_topic: str, rate_hz: float) -> None:
        """Special function: Create a transcript subscription and low-rate discovery timer."""
        self.node = node
        self.cache = cache
        self.events = events
        self.model = VoiceStateModel()
        self.presence_tracker = VoicePresenceTracker()
        # Section: Import ROS messages only when constructing the runtime source.
        from std_msgs.msg import String
        self.sub = node.create_subscription(String, transcript_topic, self._on_transcript, 10)
        self.timer = node.create_timer(1.0 / max(0.1, float(rate_hz)), self.poll)
        self.cache.set("voice", self.model.snapshot())

    @staticmethod
    def _diagnostic(rows: list[dict[str, Any]], kind: str) -> dict[str, Any] | None:
        """Special function: Match verified voice diagnostic name or hardware identifier."""
        needles = ("speech-to-text", "default_microphone") if kind == "stt" else ("text-to-speech", "default_audio_output")
        for row in rows:
            blob = f"{row.get('name', '')} {row.get('hardware_id', '')}".lower()
            if any(needle in blob for needle in needles):
                return row
        return None

    def _on_transcript(self, msg: Any) -> None:
        """Special function: Cache the actual transcript exactly as published by STT."""
        self.model.set_transcript(msg.data)
        self.cache.set("voice", self.model.snapshot())

    def poll(self) -> None:
        """Special function: Refresh independent voice presence and diagnostic state."""
        try:
            names = {
                _full_node_name(name, namespace)
                for name, namespace in self.node.get_node_names_and_namespaces()
            }
        except Exception as exc:
            self.events.emit(MonitorEvent("WARN", "VOICE", "VOICE_GRAPH_QUERY_FAILED", str(exc), node=self.node.get_name()))
            return
        presence = (STT_NODE in names, TTS_NODE in names)
        rows = self.cache.get("diagnostics", [])
        rows = rows if isinstance(rows, list) else []
        self.model.set_presence(*presence)
        self.model.set_diagnostics(self._diagnostic(rows, "stt"), self._diagnostic(rows, "tts"))
        self.cache.set("voice", self.model.snapshot())

        # Section: Emit only presence transitions so persistent absence does not flood Events.
        for event in self.presence_tracker.update(presence):
            self.events.emit(event)
