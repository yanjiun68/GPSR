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
| OpenAI Whisper model | `turbo` (shared with one-shot STT) |
| device / precision | `cuda` / `fp16` |
| VAD decision frame | 100 ms |
| partial update | 900 ms |
| rolling partial context | 5 s |
| final boundary | 600 ms silence |

The implementation uses `webrtcvad` for speech boundaries and splits each 100 ms policy frame into WebRTC-compatible 10 ms checks. Each policy frame uses a voiced-subframe ratio and two consecutive qualifying frames before activation, avoiding starts from an isolated 10 ms noise spike. It shares the process's single OpenAI Whisper cache with one-shot STT.
