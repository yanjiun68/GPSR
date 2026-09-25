"""Section: Cancel-aware NavigateNamedPose client with deterministic test fallback."""

from __future__ import annotations

import threading
import time
import math
from typing import Any

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient

from robot_ai_interfaces.action import NavigateNamedPose
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
        pose_action_name: str = "/navigate_to_pose",
        server_timeout_sec: float = 5.0,
        default_result_timeout_sec: float = 60.0,
        spin_action_name: str = "/spin",
    ) -> None:
        """Special function: Configure real named navigation or deterministic unit-test mode."""
        self.node = node
        self.simulate = bool(simulate)
        self.action_name = str(action_name)
        self.pose_action_name = str(pose_action_name)
        self.spin_action_name = str(spin_action_name)
        self.server_timeout_sec = max(0.1, float(server_timeout_sec))
        self.default_result_timeout_sec = max(0.1, float(default_result_timeout_sec))
        self.named_client = (
            ActionClient(node, NavigateNamedPose, self.action_name)
            if node is not None and not self.simulate
            else None
        )
        self.pose_client = (
            ActionClient(node, NavigateToPose, self.pose_action_name)
            if node is not None and not self.simulate
            else None
        )
        # Section: Nav2 Spin supplies controlled camera viewpoints during local searches.
        self.spin_client = (
            ActionClient(node, Spin, self.spin_action_name)
            if node is not None and not self.simulate
            else None
        )
        # Section: Preserve the historical test/injection attribute for named navigation.
        self.client = self.named_client

    def spin_by_angle(
        self,
        blackboard: Any,
        angle_rad: float,
        timeout_sec: float,
    ) -> Any:
        """Special function: Rotate by one relative angle with cancellation and timeout handling."""
        bounded_timeout = max(0.1, float(timeout_sec))
        if self.simulate:
            return make_skill_result(
                "NAVIGATE",
                SUCCESS,
                "Simulated search rotation completed.",
                {"angle_rad": float(angle_rad), "simulated": True},
            )
        if self.spin_client is None:
            return make_skill_result(
                "NAVIGATE",
                FAILED,
                "Nav2 Spin client requires a ROS executor node.",
                recoverable=True,
            )
        if not self.spin_client.wait_for_server(
            timeout_sec=min(self.server_timeout_sec, bounded_timeout)
        ):
            return make_skill_result(
                "NAVIGATE",
                FAILED,
                f"Nav2 Spin action unavailable: {self.spin_action_name}",
                recoverable=True,
            )

        # Section: Nav2 interprets target_yaw as a relative rotation for the Spin behavior.
        goal = Spin.Goal()
        goal.target_yaw = float(angle_rad)
        whole_seconds = int(bounded_timeout)
        goal.time_allowance = Duration(
            sec=whole_seconds,
            nanosec=int((bounded_timeout - whole_seconds) * 1_000_000_000),
        )

        def cancel_requested() -> bool:
            """Special function: Poll queue cancellation while the spin action is running."""
            return self.is_cancel_requested(blackboard)

        goal_handle = self.wait_for_future(
            self.spin_client.send_goal_async(goal),
            min(self.server_timeout_sec, bounded_timeout),
            cancel_requested,
        )
        if self.is_cancel_requested(blackboard):
            return make_skill_result(
                "NAVIGATE",
                CANCELED,
                "Search rotation canceled before goal acceptance.",
                fatal=True,
            )
        if goal_handle is None or not goal_handle.accepted:
            return make_skill_result(
                "NAVIGATE",
                FAILED,
                "Search rotation goal was rejected or timed out.",
                recoverable=True,
            )
        wrapped = self.wait_for_future(
            goal_handle.get_result_async(),
            bounded_timeout,
            cancel_requested,
        )
        if self.is_cancel_requested(blackboard):
            goal_handle.cancel_goal_async()
            return make_skill_result(
                "NAVIGATE",
                CANCELED,
                "Search rotation canceled.",
                fatal=True,
            )
        if wrapped is None:
            goal_handle.cancel_goal_async()
            return make_skill_result(
                "NAVIGATE",
                TIMEOUT,
                "Search rotation timed out.",
                recoverable=True,
            )
        if int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED:
            return make_skill_result(
                "NAVIGATE",
                SUCCESS,
                "Search rotation completed.",
                {"angle_rad": float(angle_rad), "simulated": False},
            )
        return make_skill_result(
            "NAVIGATE",
            FAILED,
            "Search rotation failed.",
            {"angle_rad": float(angle_rad), "status": int(wrapped.status)},
            recoverable=True,
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
        """Special function: Route mutually exclusive named or raw-pose navigation."""
        target_location = str(
            action.target_location
            or action.parameters.get("target_location", "")
            or action.parameters.get("location", "")
        ).strip()
        target_pose_data = action.parameters.get("target_pose")
        if target_location and target_pose_data:
            return make_skill_result(
                action.action_type,
                FAILED,
                "NAVIGATE requires exactly one of target_location or target_pose.",
                recoverable=False,
                fatal=True,
            )
        if target_pose_data:
            target_pose = self.pose_from_dict(target_pose_data)
            if target_pose is None:
                return make_skill_result(
                    action.action_type,
                    FAILED,
                    "Raw navigation target_pose is malformed.",
                    recoverable=False,
                    fatal=True,
                )
            return self.navigate_raw_pose(blackboard, action, target_pose)
        if target_location:
            return self.navigate_named_pose(blackboard, action, target_location)
        return make_skill_result(
            action.action_type,
            FAILED,
            "Navigation target missing.",
            recoverable=True,
        )

    def navigate_named_pose(
        self,
        blackboard: Any,
        action: ActionStepData,
        target: str,
    ) -> Any:
        """Special function: Send a recorded named-pose goal and require confirmed success."""
        if self.simulate:
            return make_skill_result(
                action.action_type,
                SUCCESS,
                "Simulated named navigation completed.",
                {"target": target, "simulated": True, "target_type": "named"},
            )
        # Section: Honor historical client injection while preferring the named client.
        client = self.named_client or self.client
        if client is None:
            return make_skill_result(
                action.action_type,
                FAILED,
                "NavigateNamedPose client requires a ROS executor node.",
                recoverable=True,
            )
        timeout_sec = self.navigation_timeout(action)
        if not client.wait_for_server(timeout_sec=min(self.server_timeout_sec, timeout_sec)):
            return make_skill_result(
                action.action_type,
                FAILED,
                f"NavigateNamedPose action unavailable: {self.action_name}",
                {"target": target},
                recoverable=True,
            )
        goal = NavigateNamedPose.Goal()
        goal.target_name = target
        return self.execute_navigation_goal(
            blackboard,
            action,
            client,
            goal,
            timeout_sec,
            target,
            target_type="named",
            require_semantic_success=True,
        )

    def navigate_raw_pose(
        self,
        blackboard: Any,
        action: ActionStepData,
        target_pose: PoseStamped,
    ) -> Any:
        """Special function: Send a captured map pose directly to Nav2 NavigateToPose."""
        target_dict = self.pose_to_dict(target_pose)
        if self.simulate:
            return make_skill_result(
                action.action_type,
                SUCCESS,
                "Simulated raw-pose navigation completed.",
                {"target": target_dict, "simulated": True, "target_type": "pose"},
            )
        if self.pose_client is None:
            return make_skill_result(
                action.action_type,
                FAILED,
                "NavigateToPose client requires a ROS executor node.",
                recoverable=True,
            )
        timeout_sec = self.navigation_timeout(action)
        if not self.pose_client.wait_for_server(
            timeout_sec=min(self.server_timeout_sec, timeout_sec)
        ):
            return make_skill_result(
                action.action_type,
                FAILED,
                f"NavigateToPose action unavailable: {self.pose_action_name}",
                {"target": target_dict},
                recoverable=True,
            )
        # Section: Refresh the historical AMCL stamp immediately before Nav2 goal submission.
        if self.node is not None:
            target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = target_pose
        return self.execute_navigation_goal(
            blackboard,
            action,
            self.pose_client,
            goal,
            timeout_sec,
            target_dict,
            target_type="pose",
            require_semantic_success=False,
        )

    def execute_navigation_goal(
        self,
        blackboard: Any,
        action: ActionStepData,
        client: Any,
        goal: Any,
        timeout_sec: float,
        target: Any,
        target_type: str,
        require_semantic_success: bool,
    ) -> Any:
        """Special function: Apply shared cancellation, timeout, and status handling."""
        def cancel_requested() -> bool:
            """Special function: Poll the shared queue cancellation callback."""
            return self.is_cancel_requested(blackboard)
        goal_handle = self.wait_for_future(
            client.send_goal_async(goal),
            min(self.server_timeout_sec, timeout_sec),
            cancel_requested,
        )
        if self.is_cancel_requested(blackboard):
            return make_skill_result(
                action.action_type,
                CANCELED,
                "Navigation canceled before goal acceptance.",
                {"target": target, "target_type": target_type},
                fatal=True,
            )
        if goal_handle is None or not goal_handle.accepted:
            return make_skill_result(
                action.action_type,
                FAILED,
                "Navigation goal was rejected or timed out.",
                {"target": target, "target_type": target_type},
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
                "Navigation goal canceled.",
                {"target": target, "target_type": target_type},
                fatal=True,
            )
        if wrapped is None:
            goal_handle.cancel_goal_async()
            return make_skill_result(
                action.action_type,
                TIMEOUT,
                "Navigation result timed out.",
                {"target": target, "target_type": target_type},
                recoverable=True,
            )
        result = wrapped.result
        transport_success = int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED
        semantic_success = bool(getattr(result, "success", False))
        succeeded = transport_success and (
            semantic_success if require_semantic_success else True
        )
        message = str(getattr(result, "message", ""))
        if succeeded:
            blackboard[bb.TARGET_POSE] = target
            return make_skill_result(
                action.action_type,
                SUCCESS,
                message or "Navigation goal reached.",
                {"target": target, "simulated": False, "target_type": target_type},
            )
        return make_skill_result(
            action.action_type,
            FAILED,
            message or "Failed to reach navigation goal.",
            {"target": target, "target_type": target_type, "status": int(wrapped.status)},
            recoverable=True,
        )

    def navigation_timeout(self, action: ActionStepData) -> float:
        """Special function: Resolve a bounded result timeout for either navigation route."""
        return max(
            0.1,
            float(
                action.timeout_sec
                or action.parameters.get("timeout_sec", self.default_result_timeout_sec)
                or self.default_result_timeout_sec
            ),
        )

    @staticmethod
    def pose_from_dict(raw: Any) -> PoseStamped | None:
        """Special function: Validate and convert a JSON raw-pose target."""
        if not isinstance(raw, dict):
            return None
        try:
            frame_id = str(raw.get("frame_id", "")).strip().lstrip("/")
            position = dict(raw.get("position", {}))
            orientation = dict(raw.get("orientation", {}))
            values = [
                float(position[key]) for key in ("x", "y", "z")
            ] + [
                float(orientation[key]) for key in ("x", "y", "z", "w")
            ]
        except (KeyError, TypeError, ValueError):
            return None
        if frame_id != "map" or not all(math.isfinite(value) for value in values):
            return None
        quaternion_norm = sum(value * value for value in values[3:]) ** 0.5
        if quaternion_norm <= 1e-9:
            return None
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = values[:3]
        pose.pose.orientation.x = values[3] / quaternion_norm
        pose.pose.orientation.y = values[4] / quaternion_norm
        pose.pose.orientation.z = values[5] / quaternion_norm
        pose.pose.orientation.w = values[6] / quaternion_norm
        return pose

    @staticmethod
    def pose_to_dict(pose: PoseStamped) -> dict[str, Any]:
        """Special function: Serialize a PoseStamped navigation target for results."""
        return {
            "frame_id": str(pose.header.frame_id),
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
