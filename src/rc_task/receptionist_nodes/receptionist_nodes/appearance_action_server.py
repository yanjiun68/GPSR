#!/usr/bin/env python3

# Import copy so action results are independent snapshots of the latest appearance.
import copy

# Import time so the execute loop can publish feedback at a controlled rate.
import time

# Import ROS 2 Python APIs used by this action server.
import rclpy

# Import ActionServer plus goal/cancel response enums for controlled action handling.
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# Import MultiThreadedExecutor so subscriptions and action execution can run together.
from rclpy.executors import MultiThreadedExecutor

# Import the ROS 2 Node base class.
from rclpy.node import Node

# Import the action and appearance message used by this wrapper.
from robot_ai_interfaces.action import TrackGuestAppearance

# Import GuestAppearance so the node can keep an unknown default estimate.
from robot_ai_interfaces.msg import GuestAppearance


# Wrap the streaming appearance estimate topic as a cancelable action.
class AppearanceActionServer(Node):
    # Construct all publishers, subscribers, and action server resources.
    def __init__(self):
        # Give the node the executable name expected by launch files.
        super().__init__("appearance_action_server")

        # Declare the input topic for appearance estimates.
        self.declare_parameter("appearance_topic", "/appearance/current_estimate")

        # Declare how often feedback should be published while tracking is active.
        self.declare_parameter("feedback_period_sec", 0.5)

        # Cache the configured topic name.
        self.appearance_topic = str(self.get_parameter("appearance_topic").value)

        # Cache the configured feedback period with a safe lower bound.
        self.feedback_period_sec = max(
            0.05,
            float(self.get_parameter("feedback_period_sec").value),
        )

        # Keep the latest appearance estimate for feedback and final result.
        self.latest_appearance = self._unknown_appearance()

        # Subscribe to semantic appearance estimates from the perception node.
        self.create_subscription(
            GuestAppearance,
            self.appearance_topic,
            self._appearance_cb,
            10,
        )

        # Expose the action name expected by RobotClients and SMACC2.
        self.action_server = ActionServer(
            self,
            TrackGuestAppearance,
            "/appearance/track_guest",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        # Log readiness after all ROS endpoints have been created.
        self.get_logger().info(
            f"Appearance action ready on /appearance/track_guest using {self.appearance_topic}."
        )

    # Build a default unknown appearance message.
    def _unknown_appearance(self):
        # Create the message object.
        appearance = GuestAppearance()

        # Fill each binary attribute with UNKNOWN.
        appearance.glasses = GuestAppearance.UNKNOWN
        appearance.hat = GuestAppearance.UNKNOWN
        appearance.face_mask = GuestAppearance.UNKNOWN

        # Fill string and confidence fields with safe defaults.
        appearance.shirt_color = ""
        appearance.glasses_confidence = 0.0
        appearance.hat_confidence = 0.0
        appearance.face_mask_confidence = 0.0
        appearance.shirt_color_confidence = 0.0

        # Stamp the default message with the current node clock.
        appearance.stamp = self.get_clock().now().to_msg()

        # Return the fully initialized default estimate.
        return appearance

    # Store the latest incoming appearance estimate.
    def _appearance_cb(self, msg):
        # Copy the message because action feedback may read it while callbacks continue.
        self.latest_appearance = copy.deepcopy(msg)

    # Accept one tracking goal at a time.
    def _goal_cb(self, goal_request):
        # Reject an empty profile id because callers should name the tracked guest.
        if not goal_request.guest_profile_id.strip():
            self.get_logger().warning("Rejecting appearance tracking goal without profile id.")
            return GoalResponse.REJECT

        # Accept the goal when it names a profile.
        return GoalResponse.ACCEPT

    # Accept cancellation because the caller uses cancel to finalize tracking.
    def _cancel_cb(self, goal_handle):
        # Silence the unused handle while documenting the intentional behavior.
        del goal_handle

        # Allow the state machine to stop tracking and receive the latest estimate.
        return CancelResponse.ACCEPT

    # Execute appearance tracking until the caller cancels or ROS shuts down.
    def _execute_cb(self, goal_handle):
        # Prepare one feedback message reused across the loop.
        feedback = TrackGuestAppearance.Feedback()

        # Keep publishing the latest estimate until cancellation is requested.
        while rclpy.ok() and not goal_handle.is_cancel_requested:
            # Attach the latest estimate to feedback.
            feedback.latest_appearance = copy.deepcopy(self.latest_appearance)

            # Publish feedback so callers can update their blackboard.
            goal_handle.publish_feedback(feedback)

            # Sleep only for the configured feedback period.
            time.sleep(self.feedback_period_sec)

        # Prepare the final result with the latest estimate.
        result = TrackGuestAppearance.Result()
        result.success = True
        result.message = "Appearance tracking stopped with latest estimate."
        result.appearance = copy.deepcopy(self.latest_appearance)

        # Mark canceled when the caller requested cancellation.
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result

        # Mark succeeded only if the loop ended without an explicit cancel.
        goal_handle.succeed()
        return result


# Provide the standard ROS 2 executable entry point.
def main(args=None):
    # Initialize ROS 2 for this process.
    rclpy.init(args=args)

    # Construct the action server node.
    node = AppearanceActionServer()

    # Use a multithreaded executor so action execution does not block subscriptions.
    executor = MultiThreadedExecutor()

    # Add the node to the executor before spinning.
    executor.add_node(node)

    # Spin until shutdown.
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


# Run the entry point when the file is executed directly.
if __name__ == "__main__":
    main()
