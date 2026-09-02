"""Section: Hardware-free Voice UI state tests."""

from gpsr_sys_ui.sources.voice_source import VoicePresenceTracker, VoiceStateModel


def test_voice_components_are_independent_and_transcript_is_exact() -> None:
    """Special function: STT can listen while TTS remains healthy and idle."""
    model = VoiceStateModel()
    model.set_presence(True, True)
    model.set_diagnostics(
        {"health": "OK", "message": "Recording audio", "hardware_id": "default_microphone"},
        {"health": "OK", "message": "Idle", "hardware_id": "default_audio_output"},
    )
    model.set_transcript("go to the living room", timestamp=100.0)
    state = model.snapshot(now=104.5)
    assert state["stt"]["state"] == "LISTENING"
    assert state["tts"]["state"] == "IDLE"
    assert state["transcript"]["text"] == "go to the living room"
    assert state["transcript"]["age_sec"] == 4.5


def test_old_transcript_never_changes_voice_health() -> None:
    """Special function: Transcript age is metadata and not a false stale alarm."""
    model = VoiceStateModel()
    model.set_presence(True, True)
    model.set_diagnostics(
        {"health": "OK", "message": "Idle"},
        {"health": "OK", "message": "Speaking"},
    )
    model.set_transcript("hello", timestamp=1.0)
    state = model.snapshot(now=10001.0)
    assert state["transcript"]["age_sec"] == 10000.0
    assert state["stt"]["health"] == "OK"
    assert state["tts"]["health"] == "OK"
    assert state["tts"]["state"] == "SPEAKING"


def test_voice_startup_absence_and_later_presence_are_safe() -> None:
    """Special function: Missing startup nodes remain displayable and can recover."""
    model = VoiceStateModel()
    missing = model.snapshot(now=10.0)
    assert missing["stt"]["health"] == "MISSING"
    assert missing["tts"]["health"] == "MISSING"
    model.set_presence(True, False)
    recovered = model.snapshot(now=11.0)
    assert recovered["stt"]["present"] is True
    assert recovered["stt"]["health"] == "WARN"
    assert recovered["tts"]["health"] == "MISSING"


def test_voice_missing_event_occurs_once_per_transition() -> None:
    """Special function: Polling does not repeat a persistent STT disappearance event."""
    tracker = VoicePresenceTracker()
    assert tracker.update((True, True)) == []
    first = tracker.update((False, True))
    assert [event.event_code for event in first] == ["VOICE_STT_MISSING"]
    assert tracker.update((False, True)) == []
