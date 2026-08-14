#!/usr/bin/env python3

import subprocess

import rclpy
from rclpy.node import Node
from voice_interfaces.srv import SayText
from ament_index_python.packages import get_package_share_directory


class TTSNode(Node):
    """Thin TTS wrapper: exposes a SayText service."""

    def __init__(self):
        super().__init__("voice_tts")

        # Parameters (configurable by launch)
        self.declare_parameter("service_name", "/voice/tts/say")
        self.declare_parameter("piper_bin", "piper")
        self.declare_parameter("piper_model_path", get_package_share_directory("voice") + "/models/en_US-lessac-medium.onnx")

        self.service_name = self.get_parameter("service_name").get_parameter_value().string_value
        self.piper_bin = self.get_parameter("piper_bin").get_parameter_value().string_value
        self.model_path = self.get_parameter("piper_model_path").get_parameter_value().string_value or get_package_share_directory("voice") + "/models/en_US-lessac-medium.onnx"

        # Service server
        self._srv = self.create_service(SayText, self.service_name, self._handle_say)

        self.get_logger().info(
            f"[voice_tts] Ready. Service: {self.service_name} (model: {self.model_path or 'SKIP'})"
        )

    def _handle_say(self, request: SayText.Request, response: SayText.Response):
        text = request.text.strip()
        if not text:
            response.success = False
            response.message = "empty text"
            self.get_logger().warn("[voice_tts] Received empty text, skipping.")
            return response
        if not self.model_path:
            response.success = False
            response.message = "piper_model_path is empty"
            self.get_logger().warn("[voice_tts] piper_model_path is empty, skipping TTS.")
            return response

        cmd = [
            self.piper_bin,
            "--model",
            self.model_path,
            "--output_file",
            "-",  # stdout WAV
        ]

        try:
            self.get_logger().info(f"[voice_tts] Speaking: \"{text}\"")
            p1 = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            assert p1.stdin is not None
            assert p1.stdout is not None
            assert p1.stderr is not None
            p1.stdin.write((text + "\n").encode("utf-8"))
            p1.stdin.close()

            subprocess.run(["aplay", "-q"], stdin=p1.stdout, check=False)
            stderr = p1.stderr.read()
            p1.wait(timeout=30)

            if p1.returncode not in (0, None):
                msg = f"Piper returned {p1.returncode}: {stderr.decode('utf-8', 'ignore')[:200]}"
                self.get_logger().warn(f"[voice_tts] {msg}")
                response.success = False
                response.message = msg
                return response

            response.success = True
            response.message = "spoken"
            return response
        except FileNotFoundError as e:
            response.success = False
            response.message = f"missing binary: {e}"
            self.get_logger().error(f"[voice_tts] Missing binary: {e}")
            return response
        except Exception as e:
            response.success = False
            response.message = f"TTS failed: {e}"
            self.get_logger().warn(f"[voice_tts] TTS failed: {e}")
            return response


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
