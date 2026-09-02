"""Section: High-level client for the central GPSR lifecycle manager."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable

from robot_ai_interfaces.srv import ManageLifecycleSkill
from robot_yasmin_tasks.gpsr.action_types import (
    GPSR_CLEANUP_SKILL_SERVICE,
    GPSR_PREPARE_SKILL_SERVICE,
    GPSR_RECOVER_SKILL_SERVICE,
)
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
        prepare_service_name: str = GPSR_PREPARE_SKILL_SERVICE,
        cleanup_service_name: str = GPSR_CLEANUP_SKILL_SERVICE,
        recover_service_name: str = GPSR_RECOVER_SKILL_SERVICE,
        transition_timeout_sec: float = 60.0,
        operation_timeout_margin_sec: float = 10.0,
    ) -> None:
        """Special function: Create fixed clients for the central lifecycle manager."""
        self.node = node
        self.transition_timeout_sec = max(0.1, float(transition_timeout_sec))
        # Section: Client waits longer than the manager's absolute transition deadline.
        self.operation_timeout_margin_sec = max(1.0, float(operation_timeout_margin_sec))
        self.prepare_client = node.create_client(
            ManageLifecycleSkill,
            prepare_service_name,
        )
        self.cleanup_client = node.create_client(
            ManageLifecycleSkill,
            cleanup_service_name,
        )
        self.recover_client = node.create_client(
            ManageLifecycleSkill,
            recover_service_name,
        )

    def wait_until_ready(self, timeout_sec: float) -> tuple[bool, str]:
        """Special function: Verify every manager service before exposing queue intake."""
        # Section: One shared deadline prevents three service waits multiplying startup delay.
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        services = (
            ("prepare", self.prepare_client),
            ("cleanup", self.cleanup_client),
            ("recover", self.recover_client),
        )
        for service_name, client in services:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not client.wait_for_service(timeout_sec=remaining):
                return False, f"GPSR lifecycle {service_name} service is unavailable"
        return True, "GPSR lifecycle manager services are ready"

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
            self.operation_timeout_sec,
            lambda: self._cancel_requested(blackboard),
        ):
            # Section: A timed-out service remains live in ROS, so queue explicit cleanup behind it.
            cleaned, cleanup_message = self.cleanup_named_skill(skill_name)
            blackboard[bb.START_CLEANUP_CONFIRMED] = cleaned
            return False, (
                f"Lifecycle prepare timed out or was canceled: {skill_name}; "
                f"cleanup={cleanup_message}"
            )
        response = future.result()
        if response is None or not bool(response.success):
            reason = str(getattr(response, "message", "null lifecycle prepare response"))
            self.node.get_logger().error(
                f"[GPSR_EXECUTOR] lifecycle prepare failed: skill={skill_name} reason={reason}"
            )
            # Section: Retry exactly once only for a fault in this skill's conflict group.
            if response is None or not self.has_matching_integrity_fault(response):
                return False, reason
            recovered, recovery_message, cleanup_confirmed = self.recover_once(skill_name)
            blackboard[bb.START_CLEANUP_CONFIRMED] = cleanup_confirmed
            if not recovered:
                return False, (
                    f"Lifecycle recovery failed for {skill_name}: {recovery_message}; "
                    f"initial_prepare={reason}"
                )
            blackboard[bb.ACTIVE_LIFECYCLE_SKILL] = skill_name
            self.node.get_logger().info(
                f"[GPSR_EXECUTOR] {skill_name} lifecycle group recovered and prepared"
            )
            return True, recovery_message
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
        if not self._wait_for_future(future, self.operation_timeout_sec):
            return False, f"Lifecycle cleanup timed out: {skill_name}"
        response = future.result()
        if response is None or not bool(response.success):
            return False, str(getattr(response, "message", "null lifecycle cleanup response"))
        blackboard[bb.ACTIVE_LIFECYCLE_SKILL] = ""
        blackboard[bb.START_CLEANUP_CONFIRMED] = True
        return True, str(response.message)

    @property
    def operation_timeout_sec(self) -> float:
        """Special function: Add response-delivery margin to the manager transition deadline."""
        return self.transition_timeout_sec + self.operation_timeout_margin_sec

    def cleanup_named_skill(self, skill_name: str) -> tuple[bool, str]:
        """Special function: Restore an explicit skill to UNCONFIGURED after uncertain prepare."""
        if not self.cleanup_client.wait_for_service(
            timeout_sec=min(5.0, self.uncertain_cleanup_timeout_sec)
        ):
            return False, "GPSR lifecycle cleanup service is unavailable"
        request = ManageLifecycleSkill.Request()
        request.skill_name = str(skill_name).strip().upper()
        request.timeout_sec = float(self.transition_timeout_sec)
        future = self.cleanup_client.call_async(request)
        if not self._wait_for_future(future, self.uncertain_cleanup_timeout_sec):
            return False, f"Lifecycle cleanup timed out: {request.skill_name}"
        response = future.result()
        if response is None or not bool(response.success):
            return False, str(getattr(response, "message", "null lifecycle cleanup response"))
        return True, str(response.message)

    @staticmethod
    def has_matching_integrity_fault(response: Any) -> bool:
        """Special function: Detect only a fault in the requested skill's resource domain."""
        try:
            payload = json.loads(str(getattr(response, "state_json", "{}")) or "{}")
        except (TypeError, ValueError, json.JSONDecodeError):
            return False
        group = str(payload.get("conflict_group", ""))
        faults = payload.get("integrity_faults", {})
        return bool(group and isinstance(faults, dict) and faults.get(group))

    def recover_once(self, skill_name: str) -> tuple[bool, str, bool]:
        """Special function: Request one bounded clean-baseline-and-prepare recovery."""
        if not self.recover_client.wait_for_service(
            timeout_sec=min(5.0, self.transition_timeout_sec)
        ):
            return False, "GPSR lifecycle recovery service is unavailable", False
        request = ManageLifecycleSkill.Request()
        request.skill_name = str(skill_name).strip().upper()
        request.timeout_sec = float(self.transition_timeout_sec)
        future = self.recover_client.call_async(request)
        if not self._wait_for_future(future, self.operation_timeout_sec):
            return False, f"Lifecycle recovery timed out: {request.skill_name}", False
        response = future.result()
        if response is None:
            return False, "null lifecycle recovery response", False
        success = bool(response.success)
        # Section: A remaining matching fault means teardown safety was not proven.
        cleanup_confirmed = success or not self.has_matching_integrity_fault(response)
        return success, str(response.message), cleanup_confirmed

    @property
    def uncertain_cleanup_timeout_sec(self) -> float:
        """Special function: Allow a queued cleanup to wait behind the original prepare deadline."""
        return (2.0 * self.transition_timeout_sec) + self.operation_timeout_margin_sec
