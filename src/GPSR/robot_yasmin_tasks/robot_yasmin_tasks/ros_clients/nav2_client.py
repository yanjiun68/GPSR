"""Section: Cancel-aware NavigateNamedPose client with deterministic test fallback."""

from __future__ import annotations

import threading
import time
from typing import Any

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

from receptionist_interfaces.action import NavigateNamedPose
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, FAILED, SUCCESS, TIMEOUT


class Nav2Client:
    """Special function: Navigate to recorded named poses through the ROS action boundary."""

    def __init__(
        self,
        node: Any | None = None,
        simulate: bool = True,
        action_name: str = "/nav/navigate_named_pose",
        server_timeout_sec: float = 5.0,
        default_result_timeout_sec: float = 60.0,
    ) -> None:
        """Special function: Configure real named navigation or deterministic unit-test mode."""
        self.node = node
        self.simulate = bool(simulate)
        self.action_name = str(action_name)
        self.server_timeout_sec = max(0.1, float(server_timeout_sec))
        self.default_result_timeout_sec = max(0.1, float(default_result_timeout_sec))
        self.client = (
            ActionClient(node, NavigateNamedPose, self.action_name)
            if node is not None and not self.simulate
            else None
        )

    @staticmethod
    def wait_for_future(
        future: Any,
        timeout_sec: float,
        cancel_requested: Any = None,
    ) -> Any:
        """Special function: Wait for an externally spun ROS future with cancellation polling."""
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() < deadline:
            if completed.wait(timeout=0.05):
                return future.result()
            if callable(cancel_requested) and cancel_requested():
                return None
        return future.result() if future.done() else None

    @staticmethod
    def is_cancel_requested(blackboard: Any) -> bool:
        """Special function: Read top-level queue cancellation from the shared blackboard."""
        callback = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        return bool(callback()) if callable(callback) else False

    def navigate_to_pose(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Send a named-pose goal and require confirmed action success."""
        # Section: Resolve only recorded semantic locations at this client boundary.
        target = str(
            action.target_location
            or action.parameters.get("target_location", "")
            or action.parameters.get("location", "")
            or bb_get(blackboard, bb.TARGET_POSE, "")
        ).strip()
        if not target:
            return make_skill_result(
                action.action_type,
                FAILED,
                "Navigation target missing.",
                recoverable=True,
            )

        # Section: Preserve deterministic behavior only when explicitly selected for tests.
        if self.simulate:
            return make_skill_result(
                action.action_type,
                SUCCESS,
                "Simulated named navigation completed.",
                {"target": target, "simulated": True},
            )
        if self.client is None:
            return make_skill_result(
                action.action_type,
                FAILED,
                "NavigateNamedPose client requires a ROS executor node.",
                recoverable=True,
            )

        # Section: Bound server discovery separately from the complete navigation deadline.
        timeout_sec = float(
            action.timeout_sec
            or action.parameters.get("timeout_sec", self.default_result_timeout_sec)
            or self.default_result_timeout_sec
        )
        timeout_sec = max(0.1, timeout_sec)
        if not self.client.wait_for_server(
            timeout_sec=min(self.server_timeout_sec, timeout_sec)
        ):
            return make_skill_result(
                action.action_type,
                FAILED,
                f"NavigateNamedPose action unavailable: {self.action_name}",
                {"target": target},
                recoverable=True,
            )

        # Section: Submit the named target while honoring queue cancellation.
        goal = NavigateNamedPose.Goal()
        goal.target_name = target
        cancel_requested = lambda: self.is_cancel_requested(blackboard)
        goal_handle = self.wait_for_future(
            self.client.send_goal_async(goal),
            min(self.server_timeout_sec, timeout_sec),
            cancel_requested,
        )
        if self.is_cancel_requested(blackboard):
            return make_skill_result(
                action.action_type,
                CANCELED,
                f"Navigation to '{target}' canceled before goal acceptance.",
                {"target": target},
                fatal=True,
            )
        if goal_handle is None or not goal_handle.accepted:
            return make_skill_result(
                action.action_type,
                FAILED,
                f"NavigateNamedPose rejected or timed out for '{target}'.",
                {"target": target},
                recoverable=True,
            )

        # Section: Wait for Nav2-backed arrival confirmation rather than assuming movement.
        wrapped = self.wait_for_future(
            goal_handle.get_result_async(),
            timeout_sec,
            cancel_requested,
        )
        if self.is_cancel_requested(blackboard):
            goal_handle.cancel_goal_async()
            return make_skill_result(
                action.action_type,
                CANCELED,
                f"Navigation to '{target}' canceled.",
                {"target": target},
                fatal=True,
            )
        if wrapped is None:
            goal_handle.cancel_goal_async()
            return make_skill_result(
                action.action_type,
                TIMEOUT,
                f"Navigation to '{target}' timed out.",
                {"target": target},
                recoverable=True,
            )

        # Section: Require both successful transport status and server result semantics.
        result = wrapped.result
        succeeded = (
            int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED
            and bool(getattr(result, "success", False))
        )
        message = str(getattr(result, "message", ""))
        if succeeded:
            blackboard[bb.TARGET_POSE] = target
            return make_skill_result(
                action.action_type,
                SUCCESS,
                message or f"Reached named pose '{target}'.",
                {"target": target, "simulated": False},
            )
        return make_skill_result(
            action.action_type,
            FAILED,
            message or f"Failed to reach named pose '{target}'.",
            {"target": target, "status": int(wrapped.status)},
            recoverable=True,
        )
