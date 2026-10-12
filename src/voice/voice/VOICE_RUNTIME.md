# Voice runtime dependencies

Install the system tools and Python packages before launching the voice stack:

```bash
sudo apt update
sudo apt install -y ffmpeg alsa-utils
python3 -m pip install -U openai-whisper piper-tts webrtcvad numpy torch
mkdir -p ~/piper_models
python3 -m piper.download_voices --data-dir ~/piper_models en_US-lessac-medium
```

`voice_stt` defaults to OpenAI Whisper `turbo` on CUDA. It reports an error instead of silently running that requested CUDA configuration on CPU. Piper remains CPU based and defaults to `~/piper_models/en_US-lessac-medium.onnx`.

Do not start Receptionist without `voice_stt`: Receptionist uses `/voice/stt/transcribe_action` and deliberately does not load Whisper itself.

Two-way, one-shot dialogues serialize playback and listening through the host-local
`/tmp/putrabot_audio_turn.lock`. Both `/voice/tts/say` and
`/voice/tts/say_action`, plus Receptionist's direct Piper playback, hold the turn
until playback ends. All one-shot STT entry points (`/voice/stt/start`,
`/voice/stt/transcribe`, and `/voice/stt/transcribe_action`) wait for that turn and
the configured `post_tts_guard_ms` interval before recording. They retain ownership
through transcription and transcript publication, so another TTS request waits.
The guard uses a shared completion timestamp and does not depend on the order in
which ROS delivers `/voice/tts/finished` and listening requests. A caller timeout
does not release an audio operation that is still running.

```mermaid
flowchart LR
    TTS[Complete TTS playback] --> Guard[Post-TTS echo guard]
    Guard --> STT[Record and transcribe]
    STT --> Reply[Next TTS response]
```

This coordination covers the local `voice` runtime. Continuous VAD mode is a
separate streaming mode and continues to reject one-shot STT requests; use
`enable_vad_utterance:=false` for alternating prompt/answer dialogues. The existing
process-lifetime microphone and speaker ownership locks remain separate from the
per-operation turn lock. Do not delete lock files while the voice nodes are running.
