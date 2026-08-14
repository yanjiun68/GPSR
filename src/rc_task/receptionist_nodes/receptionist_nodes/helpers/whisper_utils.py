#!/usr/bin/env python3

# Import Optional so helper return types can describe unavailable models cleanly.
from typing import Optional


# Load a Whisper implementation lazily and return None when no implementation exists.
def lazy_load_whisper(model_name: str):
    # Try faster-whisper first because other voice nodes in this workspace already use it.
    try:
        from faster_whisper import WhisperModel

        # Load the selected model on CPU by default so this works on non-GPU machines.
        return ("faster_whisper", WhisperModel(model_name or "base", device="cpu"))
    except Exception:
        pass

    # Try OpenAI Whisper as a second option for machines that installed that package.
    try:
        import whisper

        # Load the selected model name using the whisper package API.
        return ("whisper", whisper.load_model(model_name or "base"))
    except Exception:
        pass

    # Return None so callers can continue with an empty transcription instead of crashing.
    return None


# Transcribe one wav file with the selected lazy-loaded model.
def transcribe_file(model_handle, wav_path: str, language: Optional[str] = "en") -> str:
    # Return empty text when no model was available.
    if model_handle is None:
        return ""

    # Split the tuple produced by lazy_load_whisper into provider and model.
    provider, model = model_handle

    # Use faster-whisper's segment iterator when that provider is active.
    if provider == "faster_whisper":
        segments, _info = model.transcribe(wav_path, language=language or "en")
        return " ".join(segment.text.strip() for segment in segments).strip()

    # Use OpenAI Whisper's dictionary output when that provider is active.
    if provider == "whisper":
        result = model.transcribe(wav_path, language=language or "en")
        return str(result.get("text", "")).strip()

    # Return empty text for unknown providers so callers can safely fail closed.
    return ""
