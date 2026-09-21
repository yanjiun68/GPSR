# VAD complete-utterance transcription

`voice_stt` now has an optional continuous microphone listener. It is disabled by default, so the existing topic, service, and action STT APIs continue to behave as before.

Enable it with:

```bash
ros2 launch voice voice.launch.py enable_vad_utterance:=true
```

The listener owns its configured ALSA microphone while enabled; one-shot requests through `/voice/stt/start`, `/voice/stt/transcribe`, and `/voice/stt/transcribe_action` are therefore rejected rather than competing for that device.

It publishes rolling partial text to `/voice/stt/partial_transcript` and each completed utterance to `/voice/stt/utterance_transcript`.

Default configuration:

| Setting | Value |
| --- | --- |
| faster-whisper model | `small.en` |
| device / compute type | `cuda` / `float16` |
| VAD decision frame | 100 ms |
| partial update | 900 ms |
| rolling partial context | 5 s |
| final boundary | 600 ms silence |

The implementation uses `webrtcvad` for speech boundaries and splits each 100 ms policy frame into WebRTC-compatible 10 ms checks. A CUDA-capable faster-whisper installation is required when this opt-in listener is enabled because its requested compute type is `float16`.
