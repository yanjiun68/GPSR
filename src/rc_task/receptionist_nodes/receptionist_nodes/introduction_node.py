#!/usr/bin/env python3

# Import the ROS 2 Python client library.
import rclpy
from my_tts_system_interfaces.action import SpeakText
from rclpy.action import ActionClient
# Import the Node base class used to build ROS 2 nodes.
from rclpy.node import Node

# Import the memory lookup and introduction services used by this node.
from receptionist_interfaces.srv import GetProfile, SpeakGuestIntroduction
# Import the helper that converts profiles into simple introduction sentences.
from receptionist_nodes.helpers.profile_formatter import build_introduction_lines
# Import the helper that performs Piper playback.
from receptionist_nodes.helpers.piper_utils import speak_text_with_piper


# Define one ROS 2 node that speaks receptionist introductions.
class IntroductionNode(Node):
    def __init__(self):
        # Give the ROS 2 node a stable name so clients can find its service.
        super().__init__("introduction_node")

        # Declare the Piper model path so launch files can configure it.
        self.declare_parameter("piper_model_path", "/home/yanjiun68/piper_models/en_US-lessac-medium.onnx")
        self.declare_parameter("prefer_tts_action", True)
        self.declare_parameter("tts_action_name", "/tts/speak")
        self.declare_parameter("tts_action_timeout_sec", 1.0)
        # Cache the configured Piper model path for later speech playback.
        self.piper_model_path = self.get_parameter("piper_model_path").value
        self.prefer_tts_action = bool(self.get_parameter("prefer_tts_action").value)
        self.tts_action_name = str(self.get_parameter("tts_action_name").value)
        self.tts_action_timeout_sec = float(self.get_parameter("tts_action_timeout_sec").value)

        # Create one client used to read profiles from the memory node.
        self.get_profile_client = self.create_client(GetProfile, "/guest_memory/get_profile")
        self.tts_action_client = ActionClient(self, SpeakText, self.tts_action_name)

        # Expose one service that speaks the current guest introduction.
        self.create_service(
            SpeakGuestIntroduction,
            "/introduction/speak_guest_introduction",
            self.speak_intro_cb,
        )

    def speak_line(self, line):
        if self.prefer_tts_action and self.tts_action_client.wait_for_server(
            timeout_sec=self.tts_action_timeout_sec
        ):
            goal = SpeakText.Goal()
            goal.text = line
            goal_future = self.tts_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()
            if goal_handle and goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                action_result = result_future.result()
                return bool(action_result and action_result.result.success)

        return speak_text_with_piper(line, self.piper_model_path)

    # Fetch one stored profile from the memory node by id.
    def fetch_profile(self, profile_id):
        # Wait briefly for the memory service to appear when the system is starting up.
        if not self.get_profile_client.wait_for_service(timeout_sec=2.0):
            return None

        # Build the request that asks for one specific profile id.
        request = GetProfile.Request()
        # Store the requested id in the service request.
        request.profile_id = profile_id
        # Send the request asynchronously so ROS 2 can process it.
        future = self.get_profile_client.call_async(request)
        # Spin the node until the service call completes.
        rclpy.spin_until_future_complete(self, future)
        # Read the completed service response.
        result = future.result()
        # Return None when the service call failed or the profile was missing.
        if result is None or not result.success:
            return None
        # Return the fetched profile on success.
        return result.profile

    # Speak one introduction request using memory lookups and Piper.
    def speak_intro_cb(self, request, response):
        # Fetch the source profile that represents the guest being introduced.
        source_profile = self.fetch_profile(request.source_profile_id)
        # Fail immediately when the source profile does not exist.
        if source_profile is None:
            response.success = False
            response.message = "Source profile not found."
            response.spoken_lines = []
            return response

        # Prepare a list that will hold the fetched target profiles.
        target_profiles = []
        # Loop over each requested target id in the order provided by the caller.
        for target_profile_id in request.target_profile_ids:
            # Fetch the target profile from the memory node.
            target_profile = self.fetch_profile(target_profile_id)
            # Skip missing targets so one bad id does not crash the whole callback.
            if target_profile is None:
                continue
            # Keep the successfully fetched target profile.
            target_profiles.append(target_profile)

        # Fail when no target profiles could be resolved.
        if not target_profiles:
            response.success = False
            response.message = "No valid introduction targets were found."
            response.spoken_lines = []
            return response

        # Build the spoken introduction lines from the fetched profiles.
        spoken_lines = build_introduction_lines(
            source_profile=source_profile,
            target_profiles=target_profiles,
            include_description_bonus=request.include_description_bonus,
        )

        # Speak each line in order so the introduction sounds natural.
        for line in spoken_lines:
            # Ask the Piper helper to speak the current line.
            if not self.speak_line(line):
                # Report failure when Piper playback could not complete.
                response.success = False
                response.message = f"Failed to speak introduction line: {line}"
                response.spoken_lines = spoken_lines
                return response

        # Report success when every introduction line was spoken.
        response.success = True
        # Return a short status string for logs and debugging.
        response.message = "Introduction spoken."
        # Return the spoken lines so clients can inspect what was said.
        response.spoken_lines = spoken_lines
        # Return the populated response object to ROS 2.
        return response


# Provide the standard ROS 2 Python entry point.
def main(args=None):
    # Initialize the ROS 2 client library.
    rclpy.init(args=args)
    # Construct the introduction node.
    node = IntroductionNode()
    # Spin the node so it can answer service calls until shutdown.
    rclpy.spin(node)
    # Destroy the node explicitly during shutdown.
    node.destroy_node()
    # Shut down the ROS 2 client library.
    rclpy.shutdown()


# Run the entry point when the file is executed as a script.
if __name__ == "__main__":
    main()
