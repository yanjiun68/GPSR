"""Section: GPSR client for the production PersonFollow action adapter."""

from __future__ import annotations

import threading
import time
from typing import Any

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

from putrabot_interfaces.action import PersonFollow
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import (
    CANCELED,
    FAILED,
    NOT_FOUND,
    PERSON_LOST,
    SUCCESS,
    TIMEOUT,
)


class FollowPersonActionClient:
    """Special function: Forward FOLLOW_PERSON ActionSteps to person_follower_master."""

    def __init__(
        self,
        node: Any,
        action_name: str = "/person_follow/control",
    ) -> None:
        """Special function: Create one persistent action client owned by the GPSR executor."""
        self.action_name = str(action_name)
        self.client = ActionClient(node, PersonFollow, self.action_name)

    def follow_person(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Execute a bounded follow goal and classify its terminal result."""
        timeout_sec = max(
            0.1,
            float(
                action.timeout_sec
                or action.parameters.get("timeout_sec", 120.0)
                or 120.0
            ),
        )
        if not self.client.wait_for_server(timeout_sec=min(timeout_sec, 10.0)):
            return make_skill_result(
                action.action_type,
                FAILED,
                f"FollowPerson action unavailable: {self.action_name}",
                recoverable=True,
            )

        # Section: The master stack enrolls the centered person; target names remain GPSR metadata.
        goal = PersonFollow.Goal()
        goal.auto_select_if_needed = bool(
            action.parameters.get("auto_select_if_needed", True)
        )
        goal.follow_distance_m = float(
            action.parameters.get("follow_distance_m", 1.2) or 1.2
        )
        goal.face_hold_sec = float(
            action.parameters.get("face_hold_sec", 5.0) or 5.0
        )
        cancel_requested = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        latest_feedback: dict[str, Any] = {}

        def feedback_callback(feedback_message: Any) -> None:
            """Special function: Cache the latest standard follow feedback for the skill result."""
            feedback = feedback_message.feedback
            latest_feedback.update(
                {
                    "selected_track_id": int(feedback.selected_track_id),
                    "target_visible": bool(feedback.target_visible),
                    "target_age_sec": float(feedback.target_age_sec),
                    "facing_robot": bool(feedback.facing_robot),
                    "status": str(feedback.stop_reason),
                }
            )

        # Section: Goal acceptance and execution both honor queue-level cancellation.
        goal_handle = self.wait_for_future(
            self.client.send_goal_async(goal, feedback_callback=feedback_callback),
            min(timeout_sec, 10.0),
            cancel_requested,
        )
        if goal_handle is None:
            outcome = CANCELED if self.is_cancel_requested(cancel_requested) else TIMEOUT
            return make_skill_result(
                action.action_type,
                outcome,
                "FollowPerson goal acceptance was canceled or timed out.",
                latest_feedback,
                recoverable=outcome == TIMEOUT,
                fatal=outcome == CANCELED,
            )
        if not goal_handle.accepted:
            return make_skill_result(
                action.action_type,
                FAILED,
                "FollowPerson goal was rejected.",
                latest_feedback,
                recoverable=True,
            )

        wrapped_result = self.wait_for_future(
            goal_handle.get_result_async(),
            timeout_sec,
            cancel_requested,
        )
        if wrapped_result is None:
            goal_handle.cancel_goal_async()
            outcome = CANCELED if self.is_cancel_requested(cancel_requested) else TIMEOUT
            message = (
                "FollowPerson canceled by the GPSR queue."
                if outcome == CANCELED
                else "FollowPerson exceeded its ActionStep timeout and was canceled."
            )
            return make_skill_result(
                action.action_type,
                outcome,
                message,
                latest_feedback,
                recoverable=outcome == TIMEOUT,
                fatal=outcome == CANCELED,
            )

        # Section: Preserve action-server reason prefixes as deterministic GPSR outcomes.
        result = wrapped_result.result
        message = str(result.message)
        outcome = self.classify_result(
            bool(result.success),
            message,
            int(wrapped_result.status),
        )
        payload = {
            "target_person": str(action.target_person),
            "follow_distance_m": float(goal.follow_distance_m),
            "face_hold_sec": float(goal.face_hold_sec),
            **latest_feedback,
        }
        return make_skill_result(
            action.action_type,
            outcome,
            message,
            payload,
            recoverable=outcome in {FAILED, NOT_FOUND, PERSON_LOST, TIMEOUT},
            fatal=outcome == CANCELED,
        )

    @staticmethod
    def classify_result(success: bool, message: str, status: int) -> str:
        """Special function: Map the compact PersonFollow result into GPSR outcomes."""
        if bool(success):
            return SUCCESS
        if int(status) == GoalStatus.STATUS_CANCELED:
            return CANCELED
        reason = str(message).strip().casefold()
        if reason.startswith("person_lost:"):
            return PERSON_LOST
        if reason.startswith(("person_not_found:", "person_not_selected:")):
            return NOT_FOUND
        return FAILED

    @staticmethod
    def wait_for_future(
        future: Any,
        timeout_sec: float,
        cancel_requested: Any = None,
    ) -> Any:
        """Special function: Wait for a ROS future without spinning the executor-owned node."""
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while time.monotonic() < deadline:
            if completed.wait(timeout=0.05):
                return future.result()
            if FollowPersonActionClient.is_cancel_requested(cancel_requested):
                return None
        return future.result() if future.done() else None

    @staticmethod
    def is_cancel_requested(cancel_requested: Any) -> bool:
        """Special function: Safely evaluate the queue cancellation callback."""
        return bool(cancel_requested()) if callable(cancel_requested) else False
