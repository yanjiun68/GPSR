"""Hardware-free integration tests for the RX200 guarded-close action."""

import os
import threading
import time

from interbotix_xs_msgs.msg import JointSingleCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rx200_gripper_guard_interfaces.action import CloseUntilContact
from sensor_msgs.msg import JointState

from rx200_gripper_guard import constants as phase
from rx200_gripper_guard.gripper_guard_node import GripperGuardNode


# Section: Keep test waits bounded so a callback regression cannot hang CI.
def _wait_for_future(future, timeout_sec=3.0):
    """Wait for an executor-driven future and fail with a useful timeout."""
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), "ROS future did not complete before the test timeout"
    return future.result()


# Section: Build named feedback in a deliberately noncanonical joint order.
def _joint_state(motor_position, motor_velocity, aperture, load=0.0):
    """Create one synthetic Interbotix-style gripper feedback message."""
    message = JointState()
    message.name = ["right_finger", "gripper", "left_finger"]
    message.position = [-aperture, motor_position, aperture]
    message.velocity = [0.0, motor_velocity, 0.0]
    message.effort = [0.0, load, 0.0]
    return message


# Section: Exercise the real publisher, action server, and action client without hardware.
def test_contact_timeout_and_cancellation_issue_safe_results_and_stops():
    """Verify synthetic contact, timeout, and cancellation through ROS actions."""
    # Section: Keep ROS test logs in a writable, disposable CI location.
    os.environ.setdefault("ROS_LOG_DIR", "/tmp")
    rclpy.init()
    guard = GripperGuardNode(
        parameter_overrides=[
            Parameter("health.enabled", value=False),
            Parameter("contact.required_samples", value=3),
            Parameter("contact.history_length", value=3),
            Parameter("contact.velocity_threshold", value=0.01),
            Parameter("contact.position_span_threshold", value=0.001),
            Parameter("contact.aperture_margin", value=0.002),
            Parameter("calibration.empty_closed_aperture", value=0.005),
            Parameter("safety.feedback_timeout_sec", value=0.4),
            Parameter("safety.action_timeout_sec", value=1.0),
            Parameter("control.command_period_sec", value=0.02),
        ]
    )
    client_node = Node("rx200_gripper_guard_integration_client")
    feedback_publisher = client_node.create_publisher(
        JointState, "/rx200/sdk_joint_states", 10
    )
    observed_commands = []
    client_node.create_subscription(
        JointSingleCommand,
        "/rx200/commands/joint_single",
        lambda message: observed_commands.append(float(message.cmd)),
        10,
    )
    action_client = ActionClient(
        client_node,
        CloseUntilContact,
        "/rx200/gripper_guard/close_until_contact",
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(guard)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Section: Establish fresh moving feedback before the action goal is validated.
        assert action_client.wait_for_server(timeout_sec=2.0)
        for index in range(4):
            feedback_publisher.publish(
                _joint_state(0.1 - index * 0.002, -0.04, 0.02)
            )
            time.sleep(0.03)

        # Section: Persistent stalled feedback above empty aperture confirms contact.
        contact_goal = CloseUntilContact.Goal()
        contact_goal.close_pwm = -75.0
        contact_goal.hold_pwm = 0.0
        contact_goal.timeout_sec = 0.8
        contact_handle = _wait_for_future(action_client.send_goal_async(contact_goal))
        assert contact_handle.accepted
        for _ in range(5):
            feedback_publisher.publish(_joint_state(0.09, 0.0, 0.02, 2.0))
            time.sleep(0.03)
        contact_result = _wait_for_future(contact_handle.get_result_async()).result
        assert contact_result.success
        assert contact_result.contact_detected
        assert contact_result.reason == phase.REASON_CONTACT_CONFIRMED
        assert observed_commands[-1] == 0.0

        # Section: Continued free motion cannot confirm contact and must timeout safely.
        timeout_goal = CloseUntilContact.Goal()
        timeout_goal.close_pwm = -75.0
        timeout_goal.hold_pwm = 0.0
        timeout_goal.timeout_sec = 0.2
        timeout_handle = _wait_for_future(action_client.send_goal_async(timeout_goal))
        assert timeout_handle.accepted
        timeout_deadline = time.monotonic() + 0.35
        position = 0.08
        while time.monotonic() < timeout_deadline:
            position -= 0.001
            feedback_publisher.publish(_joint_state(position, -0.04, 0.02))
            time.sleep(0.02)
        timeout_result = _wait_for_future(timeout_handle.get_result_async()).result
        assert not timeout_result.success
        assert timeout_result.reason == phase.REASON_CONTACT_TIMEOUT
        assert observed_commands[-1] == 0.0

        # Section: A stall at calibrated empty aperture reports empty closure, not contact.
        feedback_publisher.publish(_joint_state(position - 0.001, -0.04, 0.004))
        time.sleep(0.03)
        empty_goal = CloseUntilContact.Goal()
        empty_goal.close_pwm = -75.0
        empty_goal.hold_pwm = 0.0
        empty_goal.timeout_sec = 0.8
        empty_handle = _wait_for_future(action_client.send_goal_async(empty_goal))
        assert empty_handle.accepted
        for _ in range(4):
            feedback_publisher.publish(_joint_state(position - 0.002, 0.0, 0.004))
            time.sleep(0.03)
        empty_result = _wait_for_future(empty_handle.get_result_async()).result
        assert not empty_result.success
        assert not empty_result.contact_detected
        assert empty_result.reason == phase.REASON_EMPTY_CLOSE
        assert observed_commands[-1] == 0.0

        # Section: Cancellation is accepted while fresh moving feedback remains available.
        cancel_goal = CloseUntilContact.Goal()
        cancel_goal.close_pwm = -75.0
        cancel_goal.hold_pwm = 0.0
        cancel_goal.timeout_sec = 0.8
        cancel_handle = _wait_for_future(action_client.send_goal_async(cancel_goal))
        assert cancel_handle.accepted
        feedback_publisher.publish(_joint_state(position - 0.001, -0.04, 0.02))
        cancel_response = _wait_for_future(cancel_handle.cancel_goal_async())
        assert cancel_response.goals_canceling
        cancel_result = _wait_for_future(cancel_handle.get_result_async()).result
        assert not cancel_result.success
        assert cancel_result.reason == phase.REASON_CANCELLED
        assert observed_commands[-1] == 0.0

        # Section: Loss of feedback during closing aborts before the action timeout.
        feedback_publisher.publish(_joint_state(position - 0.002, -0.04, 0.02))
        time.sleep(0.03)
        stale_goal = CloseUntilContact.Goal()
        stale_goal.close_pwm = -75.0
        stale_goal.hold_pwm = 0.0
        stale_goal.timeout_sec = 0.8
        stale_handle = _wait_for_future(action_client.send_goal_async(stale_goal))
        assert stale_handle.accepted
        stale_result = _wait_for_future(stale_handle.get_result_async()).result
        assert not stale_result.success
        assert stale_result.reason == phase.REASON_FEEDBACK_TIMEOUT
        assert observed_commands[-1] == 0.0
    finally:
        # Section: Tear down all in-process ROS entities even when an assertion fails.
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        action_client.destroy()
        client_node.destroy_node()
        guard.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
