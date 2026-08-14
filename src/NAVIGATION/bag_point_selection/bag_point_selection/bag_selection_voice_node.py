#!/usr/bin/env python3
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from bag_point_selection.path_utils import resolve_package_model_path


class BagSelectionVoiceNode(Node):
    ALLOWED_STATES = {"WAIT_FOR_SELECTION", "SELECT_BAG", "EXTRACT_CAMERA_CLUSTER"}

    def __init__(self):
        super().__init__("bag_selection_voice_node")
        self.declare_parameter("bag_selection_voice_enabled", True)
        self.declare_parameter("bag_selection_voice_piper_bin", "piper")
        self.declare_parameter("bag_selection_voice_piper_model_path", "")
        self.declare_parameter("bag_selection_voice_aplay_bin", "aplay")
        self.declare_parameter("bag_selection_left_text", "Left bag selected")
        self.declare_parameter("bag_selection_right_text", "Right bag selected")

        self.voice_enabled = bool(self.get_parameter("bag_selection_voice_enabled").value)
        self.session_announce_enabled = True
        self.current_state = "IDLE"
        self.session_active = False
        self.last_spoken_side = ""
        self.is_speaking = False
        self.piper_bin = str(self.get_parameter("bag_selection_voice_piper_bin").value)
        self.piper_model_path = resolve_package_model_path(
            "bag_point_selection", self.get_parameter("bag_selection_voice_piper_model_path").value
        )
        self.aplay_bin = str(self.get_parameter("bag_selection_voice_aplay_bin").value)
        self.left_text = str(self.get_parameter("bag_selection_left_text").value)
        self.right_text = str(self.get_parameter("bag_selection_right_text").value)

        self.create_subscription(String, "/selected_side", self.side_callback, 10)
        self.create_subscription(String, "/selection_state", self.state_callback, 10)
        self.create_subscription(Bool, "/bag_selection/reset", self.reset_callback, 10)
        self.create_subscription(
            Bool, "/bag_selection/announce_selection_enabled", self.announce_enabled_callback, 10
        )

    def announce_enabled_callback(self, msg: Bool) -> None:
        self.session_announce_enabled = bool(msg.data)

    def state_callback(self, msg: String) -> None:
        self.current_state = msg.data
        self.session_active = self.current_state != "IDLE"
        if not self.session_active:
            self.last_spoken_side = ""

    def reset_callback(self, msg: Bool) -> None:
        if bool(msg.data):
            self.session_active = False
            self.current_state = "IDLE"
            self.session_announce_enabled = True
            self.last_spoken_side = ""

    def side_callback(self, msg: String) -> None:
        side = msg.data.strip().lower()
        if side not in ("left", "right"):
            return
        if not self.voice_enabled or not self.session_announce_enabled:
            return
        if not self.session_active or self.current_state not in self.ALLOWED_STATES:
            return
        if side == self.last_spoken_side or self.is_speaking:
            return

        text = self.left_text if side == "left" else self.right_text
        self.last_spoken_side = side
        self.is_speaking = True
        threading.Thread(target=self._speak_thread, args=(text,), daemon=True).start()

    def _speak_thread(self, text: str) -> None:
        try:
            ok, message = self._speak(text)
            if not ok:
                self.get_logger().warn(message)
        finally:
            self.is_speaking = False

    def _speak(self, text: str) -> tuple[bool, str]:
        if not self.piper_model_path:
            return False, "bag selection voice disabled: piper model path is empty"

        cmd = [
            self.piper_bin,
            "--model",
            self.piper_model_path,
            "--output_file",
            "-",
        ]

        try:
            p1 = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            assert p1.stdin is not None
            assert p1.stdout is not None
            p1.stdin.write((text + "\n").encode("utf-8"))
            p1.stdin.close()

            p2 = subprocess.run([self.aplay_bin, "-q"], stdin=p1.stdout, capture_output=True)
            _, stderr = p1.communicate(timeout=30)

            if p1.returncode != 0:
                err = stderr.decode("utf-8", "ignore").strip()
                return False, f"piper failed rc={p1.returncode}: {err[:200]}"
            if p2.returncode != 0:
                err = p2.stderr.decode("utf-8", "ignore").strip()
                return False, f"aplay failed rc={p2.returncode}: {err[:200]}"
            return True, "announcement completed"
        except FileNotFoundError as exc:
            return False, f"missing binary: {exc}"
        except subprocess.TimeoutExpired:
            return False, "piper timed out"
        except Exception as exc:
            return False, f"tts failed: {exc}"


def main(args=None):
    rclpy.init(args=args)
    node = BagSelectionVoiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
