#!/usr/bin/env python3

# Import regular expressions so the node can parse simple spoken answers safely.
import re
# Import subprocess so the node can call Piper and the ALSA recorder.
import subprocess
# Import Optional so helper functions can return either a path or None.
from typing import Optional

# Import the ROS 2 Python client library.
import rclpy
# Import the ActionServer class used to expose the dialogue action.
from rclpy.action import ActionServer
# Import the Node base class used to build ROS 2 nodes.
from rclpy.node import Node

# Import the action definition used for guest-profile collection.
from receptionist_interfaces.action import CollectGuestProfile
# Import the profile message that will be returned to SMACC2.
from receptionist_interfaces.msg import PersonProfile
# Import the helper used to load Whisper lazily.
from receptionist_nodes.helpers.whisper_utils import lazy_load_whisper, transcribe_file


# Define one node that owns the Receptionist dialogue flow.
class DialogueActionServer(Node):
    def __init__(self):
        # Give the ROS 2 node a stable name so clients can find its action.
        super().__init__("dialogue_action_server")

        # Declare the Piper model path so a launch file can configure speech output.
        self.declare_parameter("piper_model_path", "/home/yanjiun68/piper_models/en_US-lessac-medium.onnx")
        # Declare the Whisper model name so a launch file can configure speech recognition.
        self.declare_parameter("whisper_model", "base")
        # Declare the Whisper language so recognition uses the intended vocabulary.
        self.declare_parameter("whisper_language", "en")
        # Declare the audio input device used by arecord.
        self.declare_parameter("mic_device", "default")
        # Declare the recording duration for each short answer capture.
        self.declare_parameter("record_sec", 4.0)

        # Keep one lazy-loaded Whisper model so repeated calls stay lightweight.
        self._whisper_model = None

        # Expose the profile-collection action used by the receptionist SMACC2 machine.
        self._action_server = ActionServer(
            self,
            CollectGuestProfile,
            "/dialogue/collect_guest_profile",
            execute_callback=self.execute_cb,
        )

    # Resolve one request field against the node parameter of the same purpose.
    def _goal_or_param(self, goal_value, parameter_name):
        # Return the goal value when the caller provided a non-empty string.
        if isinstance(goal_value, str) and goal_value:
            return goal_value
        # Return the goal value when the caller provided a non-zero numeric override.
        if isinstance(goal_value, float) and goal_value > 0.0:
            return goal_value
        # Fall back to the node parameter so launch configuration still works.
        return self.get_parameter(parameter_name).value

    # Speak one text prompt through Piper.
    def speak(self, text: str, model_path: str):
        # Warn and stop early when no Piper model path is configured.
        if not model_path:
            self.get_logger().warn("No Piper model path configured for dialogue output.")
            return

        # Build the Piper command that writes raw audio to standard output.
        command = ["piper", "--model", model_path, "--output_file", "-"]

        try:
            # Start Piper and keep stdin/stdout pipes so the node can stream text and audio.
            piper_process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            # Send the text prompt to Piper on stdin.
            piper_process.stdin.write((text + "\n").encode())
            # Close Piper stdin so the synthesis process knows the input is complete.
            piper_process.stdin.close()
            # Pipe the synthesized audio directly into aplay for playback.
            subprocess.run(["aplay", "-q"], stdin=piper_process.stdout, check=False)
        except Exception as exc:
            # Log the playback error while still allowing the action to continue if possible.
            self.get_logger().error(f"Dialogue TTS failed: {exc}")

    # Record one short answer to a temporary wav file.
    def record(self, mic_device: str, record_sec: float, wav_path: str) -> Optional[str]:
        # Enforce a small minimum duration so Whisper receives enough audio to work with.
        duration = max(1, int(record_sec))
        # Build the arecord command used to capture single-channel 16 kHz audio.
        command = [
            "arecord",
            "-D",
            mic_device,
            "-f",
            "S16_LE",
            "-c",
            "1",
            "-r",
            "16000",
            "-d",
            str(duration),
            wav_path,
        ]

        # Run the audio capture command without raising an exception on failure.
        result = subprocess.run(command, check=False)
        # Return None when recording failed so the caller can handle the missing answer safely.
        if result.returncode != 0:
            return None
        # Return the recorded wav path when capture succeeded.
        return wav_path

    # Turn one wav recording into text using Whisper.
    def transcribe(self, wav_path: str, model_name: str, language: str) -> str:
        # Load the Whisper model only on first use so startup stays lightweight.
        if self._whisper_model is None:
            self._whisper_model = lazy_load_whisper(model_name)
        # Ask Whisper to transcribe the wav file in the configured language.
        return transcribe_file(self._whisper_model, wav_path, language)

    # Ask one spoken question and return the transcribed answer text.
    def ask_and_transcribe(
        self,
        question: str,
        mic_device: str,
        record_sec: float,
        wav_path: str,
        piper_model_path: str,
        whisper_model: str,
        whisper_language: str,
    ) -> str:
        # Speak the question aloud before recording the answer.
        self.speak(question, piper_model_path)
        # Record the guest answer to a temporary wav file.
        recorded_path = self.record(mic_device, record_sec, wav_path)
        # Return an empty string when recording failed.
        if recorded_path is None:
            return ""
        # Transcribe and return the recorded answer text.
        return self.transcribe(recorded_path, whisper_model, whisper_language)

    # Parse a likely name from a simple spoken sentence.
    def parse_name(self, text: str) -> str:
        # Normalize whitespace before applying the simple patterns.
        cleaned_text = text.strip()
        # Keep a lowercase copy for case-insensitive matching.
        lowered_text = cleaned_text.lower()
        # Try a few predictable sentence patterns used in simple introductions.
        patterns = [
            r"my name is\s+([a-zA-Z]+)",
            r"i am\s+([a-zA-Z]+)",
            r"i'm\s+([a-zA-Z]+)",
        ]
        # Test each pattern until one matches.
        for pattern in patterns:
            # Apply the regular expression to the lowercased answer.
            match = re.search(pattern, lowered_text)
            # Return the captured word as the name when a pattern matches.
            if match:
                return match.group(1).capitalize()
        # Fall back to the last spoken token when the answer did not match a pattern.
        words = cleaned_text.split()
        # Return the last token capitalized, or an empty string if no words were heard.
        return words[-1].capitalize() if words else ""

    # Parse a coarse gender label without ever treating uncertainty as an error.
    def parse_gender(self, text: str) -> str:
        # Normalize the answer so keyword checks stay simple.
        lowered_text = text.strip().lower()
        # Return "female" when the answer contains a direct female keyword.
        if any(keyword in lowered_text for keyword in ["female", "woman", "girl"]):
            return "female"
        # Return "male" when the answer contains a direct male keyword.
        if any(keyword in lowered_text for keyword in ["male", "man", "boy"]):
            return "male"
        # Fall back to "unknown" when the answer was missing or unclear.
        return "unknown"

    # Parse the guest's favourite drink from a short spoken sentence.
    def parse_favorite_drink(self, text: str) -> str:
        # Normalize whitespace and casing before pattern matching.
        lowered_text = text.strip().lower()
        # Try a few direct sentence patterns that guests commonly use.
        patterns = [
            r"my favourite drink is\s+(.+)",
            r"my favorite drink is\s+(.+)",
            r"i like\s+(.+)",
            r"i drink\s+(.+)",
        ]
        # Test each pattern until one matches.
        for pattern in patterns:
            # Apply the regular expression to the lowercased answer.
            match = re.search(pattern, lowered_text)
            # Return the captured phrase when a pattern matches.
            if match:
                return match.group(1).strip(" .,!?\t")
        # Fall back to the raw cleaned answer when no pattern matched.
        return lowered_text.strip(" .,!?\t")

    # Execute one full receptionist dialogue action goal.
    async def execute_cb(self, goal_handle):
        # Keep a local handle to the goal request for readability.
        goal = goal_handle.request
        # Prepare one feedback message instance that will be updated across stages.
        feedback = CollectGuestProfile.Feedback()
        # Prepare the final action result object.
        result = CollectGuestProfile.Result()

        # Resolve the Piper path from either the goal override or the node parameter.
        piper_model_path = self._goal_or_param(goal.piper_model_path, "piper_model_path")
        # Resolve the Whisper model from either the goal override or the node parameter.
        whisper_model = self._goal_or_param(goal.whisper_model, "whisper_model")
        # Resolve the Whisper language from either the goal override or the node parameter.
        whisper_language = self._goal_or_param(goal.whisper_language, "whisper_language")
        # Resolve the microphone device from either the goal override or the node parameter.
        mic_device = self._goal_or_param(goal.mic_device, "mic_device")
        # Resolve the recording duration from either the goal override or the node parameter.
        record_sec = self._goal_or_param(goal.record_sec, "record_sec")

        # Publish feedback before asking for the guest's name.
        feedback.stage = "ASK_NAME"
        # Clear the heard text field before the first prompt.
        feedback.heard_text = ""
        # Send the feedback to any action client watching progress.
        goal_handle.publish_feedback(feedback)
        # Ask for the guest's name and transcribe the answer.
        name_text = self.ask_and_transcribe(
            "What is your name?",
            mic_device,
            record_sec,
            "/tmp/receptionist_name.wav",
            piper_model_path,
            whisper_model,
            whisper_language,
        )

        # Publish feedback before asking for the guest's favourite drink.
        feedback.stage = "ASK_FAVORITE_DRINK"
        # Store the previous transcription in the feedback for debugging.
        feedback.heard_text = name_text
        # Send the updated feedback to any action client watching progress.
        goal_handle.publish_feedback(feedback)
        # Ask for the guest's favourite drink and transcribe the answer.
        drink_text = self.ask_and_transcribe(
            "What is your favourite drink?",
            mic_device,
            record_sec,
            "/tmp/receptionist_favorite_drink.wav",
            piper_model_path,
            whisper_model,
            whisper_language,
        )

        # Publish feedback before asking the optional gender question.
        feedback.stage = "ASK_GENDER_OPTIONAL"
        # Store the previous transcription in the feedback for debugging.
        feedback.heard_text = drink_text
        # Send the updated feedback to any action client watching progress.
        goal_handle.publish_feedback(feedback)
        # Ask the optional gender question and transcribe the answer.
        gender_text = self.ask_and_transcribe(
            "If you would like to answer, are you male or female?",
            mic_device,
            record_sec,
            "/tmp/receptionist_gender.wav",
            piper_model_path,
            whisper_model,
            whisper_language,
        )

        # Build the returned guest profile from the parsed answers.
        profile = PersonProfile()
        # Leave the final stable id empty because the memory node will assign it.
        profile.profile_id = ""
        # Mark the role as guest because this action is only used for guest intake.
        profile.role = "guest"
        # Store the parsed guest name.
        profile.name = self.parse_name(name_text)
        # Keep age at zero because age is intentionally out of scope for this prototype.
        profile.age = 0
        # Store the optional gender estimate while allowing unknown safely.
        profile.gender = self.parse_gender(gender_text)
        # Store the parsed favourite drink.
        profile.favorite_drink = self.parse_favorite_drink(drink_text)
        # Leave hometown empty because this prototype does not collect it.
        profile.hometown = ""
        # Mark the profile as confirmed only when both required dialogue fields were obtained.
        profile.is_confirmed = bool(profile.name and profile.favorite_drink)
        # Store one simple confidence value for downstream logging and debugging.
        profile.confidence_score = 0.8 if profile.is_confirmed else 0.4

        # Report success only when the required name and drink were obtained.
        result.success = profile.is_confirmed
        # Return a short status string for logs and action clients.
        result.message = (
            "Guest profile collected."
            if result.success
            else "Failed to collect the required guest name and favourite drink."
        )
        # Return the collected profile to the caller.
        result.profile = profile

        # Mark the goal as succeeded when the required fields were collected.
        if result.success:
            goal_handle.succeed()
        else:
            # Mark the goal as aborted when the required fields were missing.
            goal_handle.abort()
        # Return the final action result object.
        return result


# Provide the standard ROS 2 Python entry point.
def main(args=None):
    # Initialize the ROS 2 client library.
    rclpy.init(args=args)
    # Construct the dialogue action server node.
    node = DialogueActionServer()
    # Spin the node so it can handle action goals until shutdown.
    rclpy.spin(node)
    # Destroy the node explicitly during shutdown.
    node.destroy_node()
    # Shut down the ROS 2 client library.
    rclpy.shutdown()


# Run the entry point when the file is executed as a script.
if __name__ == "__main__":
    main()
