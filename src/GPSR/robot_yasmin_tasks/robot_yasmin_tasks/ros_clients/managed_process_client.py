"""Section: Logical managed-skill start and stop client for the GPSR executor."""

import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional

from rclpy.action import ActionClient

from putrabot_interfaces.action import ManagedProcess
from putrabot_interfaces.srv import KillManagedProcess

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.action_types import COUNT_OBJECTS
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get


@dataclass(frozen=True)
class ManagedTarget:
    """Special function: Map one symbolic action type to one logical supervisor target."""

    action_type: str
    target: str
    start_action: str


# Section: Extensible registry; actions absent from this map use persistent/no-op resources.
MANAGED_TARGETS: dict[str, ManagedTarget] = {
    COUNT_OBJECTS: ManagedTarget(
        action_type=COUNT_OBJECTS,
        target="gpsr_count_objects",
        start_action="/bringup/gpsr_count_objects",
    ),
}


class NullSkillResourceManager:
    """Special function: Keep pure unit tests and non-ROS execution resource-neutral."""

    def start(self, blackboard: Any, action_type: str) -> tuple[bool, str]:
        """Special function: Treat all resources as persistent in non-ROS mode."""
        del blackboard, action_type
        return True, "no managed resources"

    def stop(self, blackboard: Any) -> tuple[bool, str]:
        """Special function: Complete cleanup immediately in non-ROS mode."""
        del blackboard
        return True, "no managed resources"


class ManagedSkillResourceClient:
    """Special function: Start and stop supervisor targets without exposing OS identities."""

    def __init__(
        self,
        node: Any,
        targets: Optional[dict[str, ManagedTarget]] = None,
        stop_service_name: str = "/kill",
        start_timeout_sec: float = 40.0,
        stop_timeout_sec: float = 20.0,
    ) -> None:
        """Special function: Create fixed action clients and the logical stop client."""
        self.node = node
        self.targets = dict(targets or MANAGED_TARGETS)
        self.stop_service_name = stop_service_name
        self.start_timeout_sec = start_timeout_sec
        self.stop_timeout_sec = stop_timeout_sec
        self.start_clients = {
            action_type: ActionClient(node, ManagedProcess, target.start_action)
            for action_type, target in self.targets.items()
        }
        self.stop_client = node.create_client(KillManagedProcess, stop_service_name)

    @staticmethod
    def _wait_for_future(
        future: Any,
        timeout_sec: float,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> bool:
        """Special function: Wait on an externally spun ROS future with cancellation polling."""
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        remaining = max(0.1, timeout_sec)
        while remaining > 0.0:
            if completed.wait(timeout=min(0.1, remaining)):
                return True
            if cancel_requested is not None and cancel_requested():
                return False
            remaining -= 0.1
        return future.done()

    @staticmethod
    def _cancel_requested(blackboard: Any) -> bool:
        """Special function: Read the top-level cancellation callback from the blackboard."""
        callback = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        return bool(callback()) if callable(callback) else False

    def start(self, blackboard: Any, action_type: str) -> tuple[bool, str]:
        """Special function: Start one mapped target and retain its returned generation."""
        blackboard[bb.START_CLEANUP_CONFIRMED] = True
        target = self.targets.get(action_type)
        if target is None:
            blackboard[bb.ACTIVE_SKILL_TARGET] = ""
            blackboard[bb.ACTIVE_SKILL_GENERATION] = ""
            return True, "action uses persistent or no managed resources"

        # Section: Wait for the fixed supervisor endpoint before requesting a fresh generation.
        client = self.start_clients[action_type]
        if not client.wait_for_server(timeout_sec=min(5.0, self.start_timeout_sec)):
            return False, f"ManagedProcess action unavailable: {target.start_action}"
        if self._cancel_requested(blackboard):
            return False, "queue canceled before managed startup"

        # Section: Force restart guarantees this queue attempt receives a fresh generation.
        goal = ManagedProcess.Goal()
        goal.force_restart = True
        goal_future = client.send_goal_async(goal)
        blackboard[bb.START_CLEANUP_CONFIRMED] = False
        if not self._wait_for_future(
            goal_future,
            self.start_timeout_sec,
        ):
            return False, f"ManagedProcess goal response timed out: {target.start_action}"
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            blackboard[bb.START_CLEANUP_CONFIRMED] = True
            return False, f"ManagedProcess goal rejected: {target.start_action}"

        # Section: An accepted start may own a partial workload until its result arrives.
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(
            result_future,
            self.start_timeout_sec,
            lambda: self._cancel_requested(blackboard),
        ):
            # Section: Forward cancellation, then wait for the supervisor cleanup barrier.
            cancel_future = goal_handle.cancel_goal_async()
            cancel_confirmed = self._wait_for_future(
                cancel_future,
                min(5.0, self.stop_timeout_sec),
            )
            result_confirmed = self._wait_for_future(
                result_future,
                self.stop_timeout_sec,
            )
            if not result_confirmed:
                return (
                    False,
                    f"ManagedProcess startup cleanup unconfirmed: {target.target}; "
                    f"cancel_response_received={cancel_confirmed}",
                )
            cleanup_result = result_future.result().result
            cleanup_message = str(cleanup_result.message)
            cleanup_generation = str(
                getattr(cleanup_result, "generation", "")
            ).strip()

            # Section: A READY-versus-cancel race is closed by stopping that exact generation.
            if bool(cleanup_result.success):
                if not cleanup_generation:
                    return False, (
                        f"ManagedProcess became ready without a generation during cancel: "
                        f"{target.target}"
                    )
                blackboard[bb.ACTIVE_SKILL_TARGET] = target.target
                blackboard[bb.ACTIVE_SKILL_GENERATION] = cleanup_generation
                stopped, stop_message = self.stop(blackboard)
                blackboard[bb.START_CLEANUP_CONFIRMED] = stopped
                return False, (
                    f"ManagedProcess startup canceled after READY; {stop_message}"
                )

            # Section: Failed/canceled supervisor results already include partial cleanup.
            cleanup_failed = "cleanup failed" in cleanup_message.lower()
            cleanup_failed = cleanup_failed or "FAILED_TEARDOWN" in cleanup_message
            blackboard[bb.START_CLEANUP_CONFIRMED] = not cleanup_failed
            return False, cleanup_message
        result = result_future.result().result
        blackboard[bb.START_CLEANUP_CONFIRMED] = True
        generation = str(getattr(result, "generation", "")).strip()
        if not bool(result.success):
            return False, str(result.message)
        if not generation:
            return False, f"ManagedProcess returned no generation for {target.target}"

        # Section: YASMIN retains only logical target and generation, never PID or PGID.
        blackboard[bb.ACTIVE_SKILL_TARGET] = target.target
        blackboard[bb.ACTIVE_SKILL_GENERATION] = generation
        return True, str(result.message)

    def stop(self, blackboard: Any) -> tuple[bool, str]:
        """Special function: Stop exactly the generation started for the current queue step."""
        target = str(bb_get(blackboard, bb.ACTIVE_SKILL_TARGET, "")).strip()
        generation = str(bb_get(blackboard, bb.ACTIVE_SKILL_GENERATION, "")).strip()
        if not target:
            return True, "action has no managed resources"
        if not generation:
            return False, f"managed target {target} has no generation"

        # Section: The service response is itself the process and graph teardown barrier.
        if not self.stop_client.wait_for_service(timeout_sec=min(5.0, self.stop_timeout_sec)):
            return False, f"KillManagedProcess service unavailable: {self.stop_service_name}"
        request = KillManagedProcess.Request()
        request.target = target
        if hasattr(request, "generation"):
            request.generation = generation
        response_future = self.stop_client.call_async(request)
        if not self._wait_for_future(response_future, self.stop_timeout_sec):
            return False, f"ManagedProcess teardown timed out: {target} generation={generation}"
        response = response_future.result()
        if response is None or not bool(response.success):
            message = getattr(response, "message", "null stop response")
            return False, str(message)
        blackboard[bb.ACTIVE_SKILL_TARGET] = ""
        blackboard[bb.ACTIVE_SKILL_GENERATION] = ""
        return True, str(response.message)
