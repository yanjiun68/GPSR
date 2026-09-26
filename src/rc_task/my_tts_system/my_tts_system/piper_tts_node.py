#!/usr/bin/env python3

# Section: Standard-library imports provide a process-lifetime exclusive audio lease.
import fcntl
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import TextIO

import rclpy
from robot_ai_interfaces.action import SpeakText
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node


# Section: Match voice_tts so every Piper-backed TTS endpoint shares one audio owner.
AUDIO_PLAYBACK_LOCK_PATH = Path('/tmp/putrabot_audio_playback.lock')


def acquire_audio_playback_lock() -> tuple[TextIO | None, str]:
    """Special function: Claim exclusive host audio playback ownership for this Piper server."""
    # Section: Open a stable lock file whose advisory lock is released automatically on process exit.
    lock_handle = AUDIO_PLAYBACK_LOCK_PATH.open('a+', encoding='utf-8')
    try:
        # Section: Reject concurrent speakers rather than allowing two Piper processes to use ALSA.
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        # Section: Return the recorded owner PID so the rejected launch identifies its conflict.
        lock_handle.seek(0)
        owner_pid = lock_handle.read().strip() or 'unknown'
        lock_handle.close()
        return None, owner_pid
    # Section: Record the current process after it owns the exclusive audio lease.
    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write(str(os.getpid()))
    lock_handle.flush()
    return lock_handle, str(os.getpid())


def release_audio_playback_lock(lock_handle: TextIO | None) -> None:
    """Special function: Release the Piper audio lease during shutdown."""
    # Section: A missing lock means startup was intentionally rejected.
    if lock_handle is None:
        return
    try:
        # Section: Unlock before closing so a replacement TTS server can start immediately.
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
    finally:
        # Section: Close the descriptor even when an unlock report fails.
        lock_handle.close()


class PiperTTSNode(Node):
    def __init__(self):
        super().__init__('piper_tts_node')

        home_dir = Path.home()
        default_piper_executable = str(home_dir / 'piper-venv' / 'bin' / 'piper')
        default_model_path = str(home_dir / 'piper_models' / 'en_US-lessac-medium.onnx')
        default_config_path = str(home_dir / 'piper_models' / 'en_US-lessac-medium.onnx.json')

        self.declare_parameter('action_name', '/tts/speak')
        self.declare_parameter('piper_executable', default_piper_executable)
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('config_path', default_config_path)
        self.declare_parameter('sample_rate', 22050)
        self.declare_parameter('speaker_id', 0)
        self.declare_parameter('use_cuda', False)

        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.piper_executable = self.get_parameter('piper_executable').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.speaker_id = self.get_parameter('speaker_id').get_parameter_value().integer_value
        self.use_cuda = self.get_parameter('use_cuda').get_parameter_value().bool_value

        # Section: Refuse a duplicate TTS server before it creates a second action endpoint.
        self._audio_playback_lock, owner_pid = acquire_audio_playback_lock()
        if self._audio_playback_lock is None:
            self.get_logger().error(
                'Audio playback is already owned by PID '
                f'{owner_pid}; disable the secondary TTS launch.'
            )
            raise RuntimeError('duplicate audio playback owner')

        self._goal_lock = threading.Lock()
        self._process_lock = threading.Lock()
        self._goal_in_progress = False
        self._active_piper_proc = None
        self._active_aplay_proc = None

        self._action_server = ActionServer(
            self,
            SpeakText,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(f'Accepting TTS action goals on: {self.action_name}')
        self.get_logger().info(f'Using Piper model: {self.model_path}')

    def goal_callback(self, goal_request) -> GoalResponse:
        text = goal_request.text.strip()
        if not text:
            self.get_logger().warn('Rejected empty TTS goal')
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._goal_in_progress:
                self.get_logger().warn('Rejected TTS goal because Piper is already speaking')
                return GoalResponse.REJECT
            self._goal_in_progress = True

        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info('Received request to cancel active TTS goal')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = SpeakText.Result()
        feedback = SpeakText.Feedback()
        text = goal_handle.request.text.strip()

        try:
            status, message = self.speak_text(goal_handle, text, feedback)
        except FileNotFoundError as exc:
            status = 'aborted'
            message = f'Missing executable or file: {exc}'
        except Exception as exc:
            status = 'aborted'
            message = f'Failed to generate speech: {exc}'
        finally:
            self._clear_active_processes()
            with self._goal_lock:
                self._goal_in_progress = False

        result.success = status == 'succeeded'
        result.message = message

        if status == 'succeeded':
            goal_handle.succeed()
            self.get_logger().info(message)
        elif status == 'canceled':
            goal_handle.canceled()
            self.get_logger().info(message)
        else:
            goal_handle.abort()
            self.get_logger().error(message)

        return result

    def speak_text(self, goal_handle, text: str, feedback: SpeakText.Feedback):
        piper_cmd = [
            self.piper_executable,
            '--model', self.model_path,
            '--config', self.config_path,
            '--output-raw'
        ]

        if self.speaker_id >= 0:
            piper_cmd.extend(['--speaker', str(self.speaker_id)])

        if self.use_cuda:
            piper_cmd.append('--cuda')

        aplay_cmd = [
            'aplay',
            '-r', str(self.sample_rate),
            '-f', 'S16_LE',
            '-t', 'raw'
        ]

        feedback.state = 'starting'
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f'Speaking action goal: {text}')

        piper_proc = subprocess.Popen(
            piper_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=False
        )

        aplay_proc = subprocess.Popen(
            aplay_cmd,
            stdin=piper_proc.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE
        )

        self._set_active_processes(piper_proc, aplay_proc)

        piper_proc.stdin.write((text + '\n').encode('utf-8'))
        piper_proc.stdin.close()
        piper_proc.stdin = None
        piper_proc.stdout.close()

        feedback.state = 'playing'
        goal_handle.publish_feedback(feedback)

        while True:
            if goal_handle.is_cancel_requested:
                self._terminate_active_processes()
                return 'canceled', 'Speech goal canceled.'

            if piper_proc.poll() is not None and aplay_proc.poll() is not None:
                break

            time.sleep(0.05)

        aplay_stderr = aplay_proc.stderr.read() if aplay_proc.stderr is not None else b''
        piper_stderr = piper_proc.stderr.read() if piper_proc.stderr is not None else b''

        if piper_proc.returncode not in (0, None):
            stderr_text = piper_stderr.decode('utf-8', errors='ignore').strip()
            return 'aborted', f'Piper failed: {stderr_text or "unknown Piper error"}'

        if aplay_proc.returncode not in (0, None):
            stderr_text = aplay_stderr.decode('utf-8', errors='ignore').strip()
            return 'aborted', f'aplay failed: {stderr_text or "unknown aplay error"}'

        return 'succeeded', f'Completed speech request: {text}'

    def _set_active_processes(self, piper_proc, aplay_proc) -> None:
        with self._process_lock:
            self._active_piper_proc = piper_proc
            self._active_aplay_proc = aplay_proc

    def _clear_active_processes(self) -> None:
        with self._process_lock:
            self._active_piper_proc = None
            self._active_aplay_proc = None

    def _terminate_active_processes(self) -> None:
        with self._process_lock:
            piper_proc = self._active_piper_proc
            aplay_proc = self._active_aplay_proc

        self._terminate_process(aplay_proc)
        self._terminate_process(piper_proc)

    def _terminate_process(self, process) -> None:
        if process is None or process.poll() is not None:
            return

        process.terminate()

        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def destroy_node(self) -> bool:
        """Special function: Release exclusive audio ownership before ROS shutdown."""
        # Section: Keep the host-wide audio lock for the full lifetime of this action server.
        release_audio_playback_lock(getattr(self, '_audio_playback_lock', None))
        self._audio_playback_lock = None
        return super().destroy_node()


def main(args=None):
    """Special function: Start one Piper action server only when it owns audio playback."""
    # Section: Initialize ROS before constructing the guarded action-server node.
    rclpy.init(args=args)
    node = None
    try:
        # Section: Construct the server only after it acquires the shared host audio lease.
        node = PiperTTSNode()
        rclpy.spin(node)
    except RuntimeError as exc:
        # Section: Exit a rejected secondary TTS launch without exposing an action server.
        rclpy.logging.get_logger('piper_tts_node').error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        # Section: Destroy the node only when this process successfully became the TTS owner.
        if node is not None:
            node.destroy_node()
        # Section: Always release the ROS context after normal or rejected startup.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
