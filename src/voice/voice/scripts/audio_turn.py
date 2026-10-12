"""Section: Serialize alternating speech and listening across local voice processes."""

# Section: Use a kernel lock so coordination does not depend on ROS callback delivery.
import fcntl  # Share exclusive audio turns between processes and threads.
import time  # Measure the echo guard with a monotonic clock.
from functools import wraps  # Preserve the decorated ROS operation's metadata.
from pathlib import Path  # Open one stable lock file for all voice endpoints.

AUDIO_TURN_LOCK_PATH = Path("/tmp/putrabot_audio_turn.lock")  # Never unlink an active lock file.


def audio_turn(kind):
    """Special function: Guard a complete TTS or one-shot STT operation."""
    if kind not in ("tts", "stt"):  # Reject accidental use for an unsupported operation.
        raise ValueError("Audio turn must be tts or stt.")  # Fail before registering callbacks.

    def decorate(operation):
        """Special function: Wrap a shared service/action implementation once."""
        @wraps(operation)  # Retain introspection and the original operation's documentation.
        def serialized(self, *args, **kwargs):
            """Special function: Wait for the previous turn before touching audio."""
            with AUDIO_TURN_LOCK_PATH.open("a+", encoding="utf-8") as lease:  # A fresh descriptor also serializes threads.
                fcntl.flock(lease.fileno(), fcntl.LOCK_EX)  # Wait until playback or transcription has really ended.
                try:  # Always release ownership, including on synthesis or recording errors.
                    if kind == "stt":  # Only microphone capture needs the post-playback guard.
                        lease.seek(0)  # Read the last playback completion while holding the lock.
                        recorded = lease.read().strip()  # Empty files mean no completed playback yet.
                        finished = float(recorded) if recorded else None  # Keep the completion timestamp shared across nodes.
                        guard = max(0.0, float(self.post_tts_guard_sec))  # Respect the existing STT guard setting.
                        elapsed = time.monotonic() - finished if finished is not None else guard  # Ignore a new file.
                        if 0.0 <= elapsed < guard:  # Ignore stale timestamps from a previous monotonic clock epoch.
                            time.sleep(guard - elapsed)  # Start recording only after the room's echo has decayed.
                    return operation(self, *args, **kwargs)  # Hold the turn until the synchronous operation returns.
                finally:  # Persist completion before another process can acquire the turn.
                    try:  # Release the kernel lock even if recording the timestamp fails.
                        if kind == "tts":  # Failed playback may still have emitted partial speech.
                            lease.seek(0)  # Replace the previous completion timestamp.
                            lease.truncate()  # Avoid retaining digits from an older timestamp.
                            lease.write(str(time.monotonic()))  # Mark actual completion, never request submission.
                            lease.flush()  # Make the deadline visible to the next STT process.
                    finally:  # Closing the descriptor also releases the lock on process exit.
                        fcntl.flock(lease.fileno(), fcntl.LOCK_UN)  # Permit the next speech or listening turn.
        return serialized  # Install the synchronized operation as the node method.
    return decorate  # Bind the requested direction to its operation wrapper.
