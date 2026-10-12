"""Section: Exercise audio ordering across real processes without microphone hardware."""

# Section: Load the same standalone helper that the installed voice nodes import.
import multiprocessing  # Run competing turns in independent address spaces.
import sys  # Expose the standalone node helper for this source-tree test.
import time  # Check echo-guard timing using the same monotonic clock.
from pathlib import Path  # Locate the voice scripts independently of the test working directory.

import pytest  # Parameterize both directions and verify failure cleanup.

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))  # Match installed sibling imports.
import audio_turn as coordinator  # Import the production coordination implementation.


def run_turn(path, kind, entered, release, finished, guard):
    """Special function: Execute a controllable speech or listening turn in a child."""
    coordinator.AUDIO_TURN_LOCK_PATH = Path(path)  # Isolate tests from the robot's real lock.

    class Node:
        """Section: Represent only the guard setting consumed by the real decorator."""
        post_tts_guard_sec = guard  # Match the production STT node's configuration contract.

        @coordinator.audio_turn(kind)  # Use the actual cross-process synchronization code.
        def perform(self):
            """Special function: Hold the turn until the test permits completion."""
            entered.set()  # Signal when audio would actually begin.
            if not release.wait(5.0):  # Bound a failed test rather than leaving a stuck child.
                raise RuntimeError("Test did not release the audio turn.")  # Report the failed coordination.
    Node().perform()  # Finish the guarded operation before advertising completion.
    finished.set()  # Signal that the lock and completion timestamp are available.


@pytest.mark.parametrize("first,second", [("tts", "stt"), ("stt", "tts"), ("tts", "tts")])  # Cover both alternating directions and queued playback.
def test_competing_process_waits_for_complete_turn(tmp_path, first, second):
    """Special function: Neither an early STT request nor the next TTS may overlap."""
    context = multiprocessing.get_context("spawn")  # Avoid inheriting a parent's open lock descriptors.
    events = [context.Event() for _ in range(6)]  # Give each process independent entry, release, and completion events.
    entered_a, release_a, finished_a, entered_b, release_b, finished_b = events  # Name each synchronization boundary.
    path = str(tmp_path / "turn.lock")  # Keep this scenario independent from parallel tests.
    process_a = context.Process(target=run_turn, args=(path, first, entered_a, release_a, finished_a, 0.0))  # Start the active turn.
    process_b = context.Process(target=run_turn, args=(path, second, entered_b, release_b, finished_b, 0.0))  # Prepare the competing request.
    try:  # Clean up child processes even when an assertion fails.
        process_a.start()  # Let the first process acquire the audio turn.
        assert entered_a.wait(5.0)  # Ensure playback or listening has actually begun.
        process_b.start()  # Submit the next turn while the first is still active.
        assert not entered_b.wait(0.3)  # No audio from the second operation may start yet.
        release_a.set()  # Simulate completed playback or transcription.
        assert finished_a.wait(5.0)  # Confirm the first turn released its lock.
        assert entered_b.wait(5.0)  # The pending request must now proceed.
        release_b.set()  # Allow the second operation to complete normally.
        assert finished_b.wait(5.0)  # Confirm the waiting request was not lost.
    finally:  # Bound resource cleanup on both success and failure.
        release_a.set()  # Unblock the first child if the test failed early.
        release_b.set()  # Unblock the second child if the test failed early.
        for process in (process_a, process_b):  # Reap every child that was actually started.
            if process.pid is not None:  # Joining an unstarted process would mask the original error.
                process.join(5.0)  # Allow normal lock cleanup before forceful shutdown.
                if process.is_alive():  # Prevent a regression from hanging the test runner.
                    process.terminate()  # Stop a child that failed to honor the release event.
                    process.join(5.0)  # Reap the terminated child.
    assert process_a.exitcode == process_b.exitcode == 0  # Require successful completion in both processes.


@pytest.mark.parametrize("fail", [False, True])  # Partial failed speech needs the same echo guard as success.
def test_completion_guard_and_exception_cleanup(tmp_path, monkeypatch, fail):
    """Special function: Guard from actual completion and release ownership on errors."""
    path = tmp_path / "turn.lock"  # Isolate timestamp persistence from other tests.
    monkeypatch.setattr(coordinator, "AUDIO_TURN_LOCK_PATH", path)  # Use the production decorator with a temporary lease.
    guard = 0.08  # Keep the test fast while checking a measurable quiet interval.

    class Node:
        """Section: Model synchronous playback and microphone entry."""
        post_tts_guard_sec = guard  # Configure the guard just as the shared STT server does.

        @coordinator.audio_turn("tts")  # Persist completion even when the operation raises.
        def speak(self):
            """Special function: Simulate successful or interrupted playback."""
            if fail:  # Exercise the cleanup path after partial speech.
                raise RuntimeError("playback failed")  # Simulate an audio backend failure.
            return "spoken"  # Preserve the wrapped method's normal result.

        @coordinator.audio_turn("stt")  # Wait for the timestamp rather than a ROS completion callback.
        def listen(self):
            """Special function: Expose when recording would begin."""
            return time.monotonic()  # Observe the start inside the protected operation.

    node = Node()  # Use one configuration for both guarded operations.
    if fail:  # Verify the original failure is still visible to the caller.
        with pytest.raises(RuntimeError, match="playback failed"):  # Do not swallow failed speech.
            node.speak()  # Release the turn through the exceptional path.
    else:  # Verify successful speech still returns its original response.
        assert node.speak() == "spoken"  # Preserve the service implementation's return value.
    completion = float(path.read_text())  # Read the actual shared completion timestamp.
    assert node.listen() - completion >= guard  # No completion-topic callback is needed to enforce the guard.


def test_stt_failure_does_not_block_next_tts(tmp_path, monkeypatch):
    """Special function: Release a failed listening turn so recovery speech can run."""
    monkeypatch.setattr(coordinator, "AUDIO_TURN_LOCK_PATH", tmp_path / "turn.lock")  # Isolate ownership.

    class Node:
        """Section: Model a failed transcription followed by recovery speech."""
        post_tts_guard_sec = 0.0  # No preceding playback needs a guard in this scenario.

        @coordinator.audio_turn("stt")  # Exercise the listening-side cleanup path.
        def listen(self):
            """Special function: Simulate a microphone or transcription exception."""
            raise RuntimeError("capture failed")  # Force release before another speech turn.

        @coordinator.audio_turn("tts")  # Acquire the same lease after failed STT.
        def speak(self):
            """Special function: Confirm recovery speech can proceed."""
            return "retry"  # Surface successful lock acquisition to the assertion.

    node = Node()  # Share one test guard configuration.
    with pytest.raises(RuntimeError, match="capture failed"):  # Preserve the original recording error.
        node.listen()  # Trigger exceptional lock cleanup.
    assert node.speak() == "retry"  # Recovery must not remain blocked by the failed turn.
