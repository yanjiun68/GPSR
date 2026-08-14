#!/usr/bin/env python3

# Import the ROS 2 Python client library.
import rclpy
# Import the Node base class used to build ROS 2 nodes.
from rclpy.node import Node
# Import the simple string topic type used by the legacy speech topic.
from std_msgs.msg import String

# Import the explicit speech service used by SMACC2 and other nodes.
from receptionist_interfaces.srv import SpeakText
# Import the helper that performs Piper playback.
from receptionist_nodes.helpers.piper_utils import speak_text_with_piper


# Define one ROS 2 node that bridges both topic-based and service-based speech requests.
class TtsBridgeNode(Node):
    def __init__(self):
        # Give the ROS 2 node a stable name so clients can find its service.
        super().__init__("tts_bridge_node")

        # Declare the Piper model path so launch files can configure it.
        self.declare_parameter("piper_model_path", "/home/yanjiun68/piper_models/en_US-lessac-medium.onnx")
        # Cache the configured Piper model path for later speech playback.
        self.piper_model_path = self.get_parameter("piper_model_path").value

        # Keep the legacy topic bridge so existing tools can still publish plain text.
        self.create_subscription(String, "/speech/say", self.on_say, 10)
        # Expose one explicit service so SMACC2 can request speech deterministically.
        self.create_service(SpeakText, "/speech/speak_text", self.speak_text_cb)

    # Play one line of speech and return whether playback succeeded.
    def _speak(self, text: str) -> bool:
        # Delegate the actual Piper playback to the shared helper function.
        return speak_text_with_piper(text, self.piper_model_path)

    # Handle the legacy topic-based speech request.
    def on_say(self, msg: String):
        # Forward the topic payload directly to the Piper helper.
        self._speak(msg.data)

    # Handle the explicit service-based speech request.
    def speak_text_cb(self, request, response):
        # Ask Piper to speak the requested text string.
        success = self._speak(request.text)
        # Return whether the playback helper reported success.
        response.success = success
        # Return a short status message for logs and debugging.
        response.message = "Speech played." if success else "Speech playback failed."
        # Return the populated response object to ROS 2.
        return response


# Provide the standard ROS 2 Python entry point.
def main(args=None):
    # Initialize the ROS 2 client library.
    rclpy.init(args=args)
    # Construct the TTS bridge node.
    node = TtsBridgeNode()
    # Spin the node so it can answer speech requests until shutdown.
    rclpy.spin(node)
    # Destroy the node explicitly during shutdown.
    node.destroy_node()
    # Shut down the ROS 2 client library.
    rclpy.shutdown()


# Run the entry point when the file is executed as a script.
if __name__ == "__main__":
    main()
