# Voice runtime dependencies

Install the system tools and Python packages before launching the voice stack:

```bash
sudo apt update
sudo apt install -y ffmpeg alsa-utils
python3 -m pip install -U openai-whisper piper-tts webrtcvad numpy torch
mkdir -p ~/piper_models
python3 -m piper.download_voices --data-dir ~/piper_models en_US-lessac-high
```

`voice_stt` defaults to OpenAI Whisper `turbo` on CUDA. It reports an error instead of silently running that requested CUDA configuration on CPU. Piper remains CPU based and defaults to `~/piper_models/en_US-lessac-high.onnx`.

Do not start Receptionist without `voice_stt`: Receptionist uses `/voice/stt/transcribe_action` and deliberately does not load Whisper itself.
