"""Section: Thin executor-side wrapper for the lifecycle FIND_OBJECT action."""

from __future__ import annotations

import threading
import time
from typing import Any

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from yasmin import Blackboard, State, StateMachine

from robot_ai_interfaces.action import FindObject
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import make_skill_result, normalize_action_step
from robot_yasmin_tasks.gpsr.outcomes import (
    CANCELED,
    FAILED,
    INVALID_INPUT,
    NOT_FOUND,
    RECOVERABLE_FAILURE,
    SKILL_OUTCOMES,
    SUCCESS,
    TIMEOUT,
)
from robot_yasmin_tasks.object_knowledge import ObjectKnowledge, ObjectKnowledgeError, TargetKind


# Section: Private blackboard keys shared by the three wrapper states.
FIND_OBJECT_GOAL = "find_object_goal"
FIND_OBJECT_GOAL_HANDLE = "find_object_goal_handle"
FIND_OBJECT_RESULT_FUTURE = "find_object_result_future"


def wait_future(
    future: Any,
    timeout_sec: float,
    canceled: Any,
) -> Any:
    """Special function: Wait for a ROS future while polling queue cancellation."""
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while time.monotonic() < deadline:
        if done.wait(timeout=0.05):
            return future.result()
        if callable(canceled) and canceled():
            return None
    return future.result() if future.done() else None


def pose_to_dict(pose: Any) -> dict[str, Any]:
    """Special function: Convert PoseStamped to JSON-safe GPSR result metadata."""
    return {
        "frame_id": str(pose.header.frame_id),
        "stamp": {
            "sec": int(pose.header.stamp.sec),
            "nanosec": int(pose.header.stamp.nanosec),
        },
        "position": {
            "x": float(pose.pose.position.x),
            "y": float(pose.pose.position.y),
            "z": float(pose.pose.position.z),
        },
        "orientation": {
            "x": float(pose.pose.orientation.x),
            "y": float(pose.pose.orientation.y),
            "z": float(pose.pose.orientation.z),
            "w": float(pose.pose.orientation.w),
        },
    }


class BuildGoalState(State):
    """Special function: Translate CURRENT_ACTION into the authoritative action goal."""

    def __init__(self, knowledge: ObjectKnowledge | None = None) -> None:
        """Special function: Declare outcomes and load authoritative object semantics."""
        super().__init__([SUCCESS, INVALID_INPUT, CANCELED])
        self.knowledge = knowledge or ObjectKnowledge.from_file(ObjectKnowledge.default_path())

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Build the complete compound FIND_OBJECT request."""
        cancel_requested = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        if callable(cancel_requested) and cancel_requested():
            return CANCELED
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        object_name = str(
            action.target_object or action.parameters.get("object_name", "")
        ).strip()
        category = str(
            action.target_category or action.parameters.get("category", "")
        ).strip()
        if not object_name and not category:
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
                action.action_type,
                INVALID_INPUT,
                "FIND_OBJECT requires object_name or category.",
                fatal=True,
            ).to_dict()
            return INVALID_INPUT
        # Section: Resolve aliases and reject unknown categories before lifecycle or navigation work.
        try:
            resolved = self.knowledge.resolve(object_name, category)
        except ObjectKnowledgeError as exc:
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
                action.action_type,
                INVALID_INPUT,
                str(exc),
                {"reason": "UNSUPPORTED_TARGET"},
                fatal=True,
            ).to_dict()
            return INVALID_INPUT
        object_name = (
            resolved.canonical_name
            if resolved.kind == TargetKind.EXACT_OBJECT
            else ""
        )
        category = (
            resolved.canonical_name
            if resolved.kind != TargetKind.EXACT_OBJECT
            else ""
        )

        # Section: Explicit location is strict; omission authorizes global fallback.
        location = str(
            action.target_location
            or action.parameters.get("location", "")
            or action.parameters.get("target_location", "")
        ).strip()
        scope = str(
            action.parameters.get(
                "search_scope",
                "STRICT_LOCATION" if location else "GLOBAL",
            )
        ).upper()
        if scope not in {"GLOBAL", "STRICT_LOCATION"} or (
            scope == "STRICT_LOCATION" and not location
        ):
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
                action.action_type,
                INVALID_INPUT,
                "FIND_OBJECT search_scope/location combination is invalid.",
                fatal=True,
            ).to_dict()
            return INVALID_INPUT

        goal = FindObject.Goal()
        goal.object_name = object_name
        goal.category = category
        goal.location = location
        goal.search_scope = scope
        goal.approach_target = bool(action.parameters.get("approach_target", True))
        goal.stand_off_distance = float(
            action.parameters.get("stand_off_distance", 0.8) or 0.8
        )
        goal.timeout_sec = float(
            action.timeout_sec
            or action.parameters.get("timeout_sec", 300.0)
            or 300.0
        )
        blackboard[FIND_OBJECT_GOAL] = goal
        return SUCCESS


class SendGoalState(State):
    """Special function: Send the prepared goal to the lifecycle action server."""

    def __init__(self, client: ActionClient | None, server_timeout_sec: float) -> None:
        """Special function: Store action client and bounded server wait."""
        super().__init__([SUCCESS, RECOVERABLE_FAILURE, CANCELED])
        self.client = client
        self.server_timeout_sec = server_timeout_sec

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Send one action goal after lifecycle PREPARE has completed."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        cancel_requested = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        if self.client is None:
            return self.fail(
                blackboard,
                action,
                "FindObject action client requires a ROS executor node.",
            )
        if not self.client.wait_for_server(timeout_sec=self.server_timeout_sec):
            return self.fail(
                blackboard,
                action,
                "FindObject action server /gpsr/find_object is unavailable.",
            )
        goal_handle = wait_future(
            self.client.send_goal_async(blackboard[FIND_OBJECT_GOAL]),
            self.server_timeout_sec,
            cancel_requested,
        )
        if callable(cancel_requested) and cancel_requested():
            return CANCELED
        if goal_handle is None or not goal_handle.accepted:
            return self.fail(
                blackboard,
                action,
                "FindObject goal was rejected or timed out.",
            )
        blackboard[FIND_OBJECT_GOAL_HANDLE] = goal_handle
        blackboard[FIND_OBJECT_RESULT_FUTURE] = goal_handle.get_result_async()
        return SUCCESS

    @staticmethod
    def fail(blackboard: Blackboard, action: Any, message: str) -> str:
        """Special function: Store a recoverable infrastructure failure."""
        blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
            action.action_type,
            RECOVERABLE_FAILURE,
            message,
            recoverable=True,
        ).to_dict()
        return RECOVERABLE_FAILURE


class WaitResultState(State):
    """Special function: Wait for and map the complete semantic action result."""

    def __init__(self) -> None:
        """Special function: Declare every GPSR wrapper outcome."""
        super().__init__(SKILL_OUTCOMES)

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Store map pose and semantic result only after mission termination."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        cancel_requested = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        goal_handle = bb_get(blackboard, FIND_OBJECT_GOAL_HANDLE, None)
        result_future = bb_get(blackboard, FIND_OBJECT_RESULT_FUTURE, None)
        timeout_sec = float(action.timeout_sec or 300.0) + 5.0
        wrapped = wait_future(result_future, timeout_sec, cancel_requested)
        if callable(cancel_requested) and cancel_requested():
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            return self.store_outcome(
                blackboard,
                action,
                CANCELED,
                "FindObject goal canceled.",
                {},
                fatal=True,
            )
        if wrapped is None:
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            return self.store_outcome(
                blackboard,
                action,
                TIMEOUT,
                "FindObject action result timed out.",
                {},
                recoverable=True,
            )

        # Section: Preserve action semantics even when the transport status is aborted.
        result = wrapped.result
        payload = {
            "found": bool(result.found),
            "approached": bool(result.approached),
            "localized": bool(result.object_pose.header.frame_id),
            "result_code": int(result.result_code),
            "object_pose": pose_to_dict(result.object_pose),
            "observation_pose": pose_to_dict(result.observation_pose),
            "object_id": str(result.object_id),
            "found_room": str(result.found_room),
            "found_viewpoint": str(result.found_viewpoint),
            "viewpoints_searched": list(result.viewpoints_searched),
            "navigation_failures": int(result.navigation_failures),
        }
        code = int(result.result_code)
        if code == int(FindObject.Result.SUCCESS):
            # Section: Retain the detector identity even when RGB-only mode has no valid map pose.
            blackboard[bb.TARGET_OBJECT] = str(result.object_id)
            # Section: Publish spatial blackboard data only when depth produced a framed 3D pose.
            if payload["localized"]:
                blackboard[bb.TARGET_POSE] = result.object_pose
                memory = dict(bb_get(blackboard, bb.ENTITY_MEMORY, {}))
                memory[str(result.object_id)] = {
                    "type": "object",
                    "pose": payload["object_pose"],
                    "room": str(result.found_room),
                    "viewpoint": str(result.found_viewpoint),
                }
                blackboard[bb.ENTITY_MEMORY] = memory
            return self.store_outcome(
                blackboard,
                action,
                SUCCESS,
                str(result.message),
                payload,
            )
        if code == int(FindObject.Result.NOT_FOUND):
            # Section: Exhaustive NOT_FOUND is terminal semantic data, not mission restart.
            return self.store_outcome(
                blackboard,
                action,
                NOT_FOUND,
                str(result.message),
                payload,
            )
        if code == int(FindObject.Result.TIMEOUT):
            return self.store_outcome(
                blackboard,
                action,
                TIMEOUT,
                str(result.message),
                payload,
                recoverable=True,
            )
        if code == int(FindObject.Result.CANCELED) or int(wrapped.status) == GoalStatus.STATUS_CANCELED:
            return self.store_outcome(
                blackboard,
                action,
                CANCELED,
                str(result.message),
                payload,
                fatal=True,
            )
        if code in {
            int(FindObject.Result.NAVIGATION_FAILED),
            int(FindObject.Result.PERCEPTION_FAILED),
        }:
            return self.store_outcome(
                blackboard,
                action,
                RECOVERABLE_FAILURE,
                str(result.message),
                payload,
                recoverable=True,
            )
        # Section: Localization and stand-off failures preserve their result_code.
        return self.store_outcome(
            blackboard,
            action,
            FAILED,
            str(result.message),
            payload,
        )

    @staticmethod
    def store_outcome(
        blackboard: Blackboard,
        action: Any,
        outcome: str,
        message: str,
        payload: dict[str, Any],
        recoverable: bool = False,
        fatal: bool = False,
    ) -> str:
        """Special function: Commit one wrapper result for ActionQueue cleanup handling."""
        blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
            action.action_type,
            outcome,
            message,
            payload,
            recoverable=recoverable,
            fatal=fatal,
        ).to_dict()
        blackboard[bb.DISPATCH_OUTCOME] = outcome
        return outcome


class FindObjectSkillSM(StateMachine):
    """Special function: Build, send, and await the lifecycle compound skill action."""

    def __init__(
        self,
        node: Any | None = None,
        action_name: str = "/gpsr/find_object",
        server_timeout_sec: float = 5.0,
    ) -> None:
        """Special function: Build the thin executor-side YASMIN wrapper."""
        super().__init__(outcomes=SKILL_OUTCOMES)
        self.set_name("FIND_OBJECT_ACTION_WRAPPER")
        client = ActionClient(node, FindObject, action_name) if node is not None else None
        self.add_state(
            "BUILD_GOAL",
            BuildGoalState(),
            transitions={SUCCESS: "SEND_GOAL", INVALID_INPUT: INVALID_INPUT, CANCELED: CANCELED},
        )
        self.add_state(
            "SEND_GOAL",
            SendGoalState(client, server_timeout_sec),
            transitions={
                SUCCESS: "WAIT_RESULT",
                RECOVERABLE_FAILURE: RECOVERABLE_FAILURE,
                CANCELED: CANCELED,
            },
        )
        self.add_state(
            "WAIT_RESULT",
            WaitResultState(),
            transitions={outcome: outcome for outcome in SKILL_OUTCOMES},
        )
