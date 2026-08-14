import rclpy
import yasmin
from yasmin import StateMachine, Blackboard
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import TIMEOUT, CANCEL
from yasmin_viewer import YasminViewerPub

# Example message type.
# Replace this with your actual detection result message type.
# For example:
# from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String


class ReadDetectionState(MonitorState):
    """
    This state listens to the detection result topic.

    When a message is received, the monitor_handler() callback is called.
    The callback saves the received detection data into the YASMIN blackboard,
    which acts like global shared data for the state machine.
    """

    def __init__(self):
        super().__init__(
            String,                  # Message type published by the detection node
            "/detection_result",      # Topic where detection results are published
            ["detected", "no_detection"],  # Custom outcomes from this state
            self.monitor_handler,     # Callback function called when a message arrives
            msg_queue=10,             # Subscriber queue size
            timeout=5.0,              # Max time to wait for a message before TIMEOUT
        )

    def monitor_handler(self, blackboard: Blackboard, msg: String) -> str:
        """
        Callback function executed whenever a detection message is received.

        Args:
            blackboard:
                Shared YASMIN global data storage.
                Other states can read values saved here.

            msg:
                The incoming detection result message from the ROS 2 topic.

        Returns:
            "detected" if useful detection data is received.
            "no_detection" if the message does not contain valid detection data.
        """

        # Debug print: show the raw incoming message
        yasmin.YASMIN_LOG_INFO(f"Received detection message: {msg}")

        # Save the entire detection message into the blackboard.
        # This allows later states to access the full message.
        blackboard["latest_detection"] = msg

        # Save only the useful detection data separately.
        # For String messages, the detected result is stored in msg.data.
        blackboard["detected_label"] = msg.data

        # Debug print: confirm that data was saved into the blackboard
        yasmin.YASMIN_LOG_INFO(
            f"Saved detected_label to blackboard: {blackboard['detected_label']}"
        )

        # Check whether the detection message contains useful data.
        # If msg.data is not empty, treat it as a valid detection.
        if msg.data:
            return "detected"

        # If the message is empty, stay in the detection-reading loop.
        return "no_detection"


class UseDetectionState(yasmin.State):
    """
    This state reads the detection result from the blackboard.

    It runs after ReadDetectionState successfully receives and stores
    detection data.
    """

    def __init__(self):
        super().__init__(["done", "missing"])

    def execute(self, blackboard: Blackboard) -> str:
        """
        Main logic of this state.

        Args:
            blackboard:
                Shared global data from the state machine.

        Returns:
            "done" if detection data exists.
            "missing" if no detection data was found.
        """

        # Check whether the previous state saved detection data.
        if "latest_detection" not in blackboard:
            yasmin.YASMIN_LOG_WARN("No latest_detection found in blackboard.")
            return "missing"

        # Read the saved detection message from the blackboard.
        detection_msg = blackboard["latest_detection"]

        # Read the simplified detected label from the blackboard.
        detected_label = blackboard.get("detected_label", None)

        # Debug print: show data retrieved from the blackboard
        yasmin.YASMIN_LOG_INFO(f"Retrieved detection message: {detection_msg}")
        yasmin.YASMIN_LOG_INFO(f"Retrieved detected label: {detected_label}")

        # Add your robot behavior here.
        # Example:
        # - move toward detected object
        # - pick detected object
        # - make a decision based on detected label

        return "done"


def main():
    """
    Main entry point for the YASMIN state machine node.
    """

    # Initialize ROS 2.
    rclpy.init()

    # Create the state machine.
    # "finished" and "failed" are the final outcomes of the whole state machine.
    sm = StateMachine(outcomes=["finished", "failed"], handle_sigint=True)

    # Add the state that subscribes to the detection result topic.
    sm.add_state(
        "READ_DETECTION",
        ReadDetectionState(),
        transitions={
            # If valid detection data is received, move to the next state.
            "detected": "USE_DETECTION",

            # If no useful detection data is received, keep listening.
            "no_detection": "READ_DETECTION",

            # If no message is received before timeout, fail the state machine.
            TIMEOUT: "failed",

            # If the state is cancelled, fail the state machine.
            CANCEL: "failed",
        },
    )

    # Add the state that uses the saved detection result.
    sm.add_state(
        "USE_DETECTION",
        UseDetectionState(),
        transitions={
            # Successfully used the detection result.
            "done": "finished",

            # Detection data was expected but missing.
            "missing": "failed",
        },
    )

    # Optional: publish the state machine structure to YASMIN Viewer.
    # This helps visualize and debug the state transitions.
    YasminViewerPub(sm, "DETECTION_STATE_MACHINE")

    # Execute the state machine.
    outcome = sm()

    # Debug print: show final state machine outcome.
    yasmin.YASMIN_LOG_INFO(f"State machine finished with outcome: {outcome}")

    # Shutdown ROS 2 cleanly.
    rclpy.shutdown()


if __name__ == "__main__":
    main()
