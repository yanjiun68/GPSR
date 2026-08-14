"""Section: High-level client for the central GPSR lifecycle manager."""

from __future__ import annotations

import threading
from typing import Any, Callable

from robot_ai_interfaces.srv import ManageLifecycleSkill
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get


class NullSkillResourceManager:
    """Special function: Keep pure unit tests independent of a running ROS graph."""

    def start(self, blackboard: Any, action_type: str) -> tuple[bool, str]:
        """Special function: Treat test resources as immediately prepared."""
        blackboard[bb.ACTIVE_LIFECYCLE_SKILL] = str(action_type).strip().upper()
        return True, "no lifecycle manager in unit test"

    def stop(self, blackboard: Any) -> tuple[bool, str]:
        """Special function: Clear logical ownership in non-ROS tests."""
        blackboard[bb.ACTIVE_LIFECYCLE_SKILL] = ""
        return True, "no lifecycle manager in unit test"


class LifecycleSkillResourceClient:
    """Special function: Prepare and clean ActionStep groups through high-level services only."""

    def __init__(
        self,
        node: Any,
        prepare_service_name: str = "/gpsr_lifecycle/prepare_skill",
        cleanup_service_name: str = "/gpsr_lifecycle/cleanup_skill",
        transition_timeout_sec: float = 15.0,
    ) -> None:
        """Special function: Create fixed clients for the central lifecycle manager."""
        self.node = node
        self.transition_timeout_sec = max(0.1, float(transition_timeout_sec))
        self.prepare_client = node.create_client(
            ManageLifecycleSkill,
            prepare_service_name,
        )
        self.cleanup_client = node.create_client(
            ManageLifecycleSkill,
            cleanup_service_name,
        )

    @staticmethod
    def _wait_for_future(
        future: Any,
        timeout_sec: float,
        cancel_requested: Callable[[], bool] | None = None,
    ) -> bool:
        """Special function: Wait on an externally spun future with bounded cancellation polling."""
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        remaining = max(0.1, float(timeout_sec))
        while remaining > 0.0:
            interval = min(0.1, remaining)
            if completed.wait(timeout=interval):
                return True
            if cancel_requested is not None and cancel_requested():
                return False
            remaining -= interval
        return future.done()

    @staticmethod
    def _cancel_requested(blackboard: Any) -> bool:
        """Special function: Read the queue cancellation callback from blackboard state."""
        callback = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        return bool(callback()) if callable(callback) else False

    def start(self, blackboard: Any, action_type: str) -> tuple[bool, str]:
        """Special function: Prepare and verify the selected ActionStep lifecycle group."""
        skill_name = str(action_type).strip().upper()
        blackboard[bb.START_CLEANUP_CONFIRMED] = True
        blackboard[bb.ACTIVE_LIFECYCLE_SKILL] = ""
        if self._cancel_requested(blackboard):
            return False, "queue canceled before lifecycle prepare"

        # Section: The central manager owns conflict resolution and low-level transitions.
        if not self.prepare_client.wait_for_service(timeout_sec=min(5.0, self.transition_timeout_sec)):
            return False, "GPSR lifecycle prepare service is unavailable"
        request = ManageLifecycleSkill.Request()
        request.skill_name = skill_name
        request.timeout_sec = float(self.transition_timeout_sec)
        future = self.prepare_client.call_async(request)
        if not self._wait_for_future(
            future,
            self.transition_timeout_sec,
            lambda: self._cancel_requested(blackboard),
        ):
            blackboard[bb.START_CLEANUP_CONFIRMED] = False
            return False, f"Lifecycle prepare timed out or was canceled: {skill_name}"
        response = future.result()
        if response is None or not bool(response.success):
            return False, str(getattr(response, "message", "null lifecycle prepare response"))
        blackboard[bb.ACTIVE_LIFECYCLE_SKILL] = skill_name
        self.node.get_logger().info(f"[GPSR_EXECUTOR] {skill_name} lifecycle group prepared")
        return True, str(response.message)

    def stop(self, blackboard: Any) -> tuple[bool, str]:
        """Special function: Deactivate, clean, and verify the current ActionStep lifecycle group."""
        skill_name = str(bb_get(blackboard, bb.ACTIVE_LIFECYCLE_SKILL, "")).strip().upper()
        if not skill_name:
            return True, "no prepared lifecycle skill"

        # Section: Cleanup response is the queue advancement barrier.
        if not self.cleanup_client.wait_for_service(timeout_sec=min(5.0, self.transition_timeout_sec)):
            return False, "GPSR lifecycle cleanup service is unavailable"
        request = ManageLifecycleSkill.Request()
        request.skill_name = skill_name
        request.timeout_sec = float(self.transition_timeout_sec)
        future = self.cleanup_client.call_async(request)
        if not self._wait_for_future(future, self.transition_timeout_sec):
            return False, f"Lifecycle cleanup timed out: {skill_name}"
        response = future.result()
        if response is None or not bool(response.success):
            return False, str(getattr(response, "message", "null lifecycle cleanup response"))
        blackboard[bb.ACTIVE_LIFECYCLE_SKILL] = ""
        blackboard[bb.START_CLEANUP_CONFIRMED] = True
        return True, str(response.message)
