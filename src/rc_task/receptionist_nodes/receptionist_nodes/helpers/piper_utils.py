#!/usr/bin/env python3

# Import subprocess so this helper can run Piper and aplay without shell interpolation.
import subprocess


# Speak one text string through Piper and return whether playback completed successfully.
def speak_text_with_piper(text: str, model_path: str) -> bool:
    # Reject empty speech text because Piper cannot produce useful output from it.
    if not text or not text.strip():
        return False

    # Reject an empty model path so callers can fall back to another TTS path.
    if not model_path or not str(model_path).strip():
        return False

    # Keep the generated wav in /tmp so repeated calls do not dirty the workspace.
    output_wav = "/tmp/receptionist_piper_output.wav"

    # Build the Piper command as a list so no shell expansion is involved.
    piper_command = ["piper", "--model", str(model_path), "--output_file", output_wav]

    # Run Piper with the requested text on stdin.
    piper_result = subprocess.run(
        piper_command,
        input=(text.strip() + "\n").encode("utf-8"),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    # Report failure when Piper could not synthesize the wav file.
    if piper_result.returncode != 0:
        return False

    # Play the synthesized wav through aplay.
    play_result = subprocess.run(
        ["aplay", "-q", output_wav],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    # Return true only when audio playback completed successfully.
    return play_result.returncode == 0
