"""Section: Central registry-driven lifecycle manager for GPSR skill groups."""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_ai_interfaces.srv import GetLifecycleSkillState, ManageLifecycleSkill
from robot_yasmin_tasks.gpsr.action_types import ALLOWED_ACTION_TYPES


@dataclass(frozen=True, slots=True)
class SkillLifecycleSpec:
    """Special function: Hold one immutable ActionStep-to-node registry entry."""

    name: str
    conflict_group: str
    lifecycle_nodes: tuple[str, ...]
    execution_interface: str


class GpsrLifecycleManager(Node):
    """Special function: Configure, activate, deactivate, and clean GPSR skill groups."""

    def __init__(self) -> None:
        """Special function: Load the registry and expose serialized lifecycle operations."""
        super().__init__("gpsr_lifecycle_manager")

        # Section: Declare the one authoritative registry and bounded transition timeout.
        self.declare_parameter("registry_file", self.default_registry_path())
        self.declare_parameter("transition_timeout_sec", 10.0)

        # Section: Serialize complete group operations so conflicts cannot race.
        self._operation_lock = threading.RLock()
        self._integrity_fault = ""
        self._prepared_skill = ""
        self._callback_group = ReentrantCallbackGroup()
        self.registry = self.load_registry(str(self.get_parameter("registry_file").value))

        # Section: Build fixed lifecycle service clients from registry node names.
        node_names = sorted({
            node_name
            for spec in self.registry.values()
            for node_name in spec.lifecycle_nodes
        })
        self._change_clients = {
            node_name: self.create_client(
                ChangeState,
                f"{node_name}/change_state",
                callback_group=self._callback_group,
            )
            for node_name in node_names
        }
        self._state_clients = {
            node_name: self.create_client(
                GetState,
                f"{node_name}/get_state",
                callback_group=self._callback_group,
            )
            for node_name in node_names
        }

        # Section: Expose high-level operations; transition details stay inside this node.
        self._prepare_service = self.create_service(
            ManageLifecycleSkill,
            "/gpsr_lifecycle/prepare_skill",
            self.prepare_callback,
            callback_group=self._callback_group,
        )
        self._cleanup_service = self.create_service(
            ManageLifecycleSkill,
            "/gpsr_lifecycle/cleanup_skill",
            self.cleanup_callback,
            callback_group=self._callback_group,
        )
        self._recover_service = self.create_service(
            ManageLifecycleSkill,
            "/gpsr_lifecycle/recover_skill",
            self.recover_callback,
            callback_group=self._callback_group,
        )
        self._state_service = self.create_service(
            GetLifecycleSkillState,
            "/gpsr_lifecycle/get_skill_state",
            self.get_state_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"[GPSR_LIFECYCLE] manager ready with {len(self.registry)} ActionStep entries"
        )

    def default_registry_path(self) -> str:
        """Special function: Resolve the installed registry with a source-tree fallback."""
        try:
            return str(
                Path(get_package_share_directory("robot_yasmin_tasks"))
                / "config"
                / "skill_registry.yaml"
            )
        except Exception:
            return str(Path(__file__).resolve().parents[2] / "config" / "skill_registry.yaml")

    def load_registry(self, registry_file: str) -> dict[str, SkillLifecycleSpec]:
        """Special function: Load and validate the JSON-compatible YAML skill registry."""
        path = Path(registry_file)
        payload = json.loads(path.read_text(encoding="utf-8"))
        raw_skills = payload.get("skills", {})
        if not isinstance(raw_skills, dict) or not raw_skills:
            raise RuntimeError(f"Lifecycle registry has no skills: {path}")

        # Section: Normalize names once so the executor and manager share exact keys.
        registry: dict[str, SkillLifecycleSpec] = {}
        for raw_name, raw_spec in raw_skills.items():
            name = str(raw_name).strip().upper()
            nodes = tuple(str(item).strip().rstrip("/") for item in raw_spec.get("lifecycle_nodes", []))
            if not name or any(not node.startswith("/") for node in nodes):
                raise RuntimeError(f"Invalid lifecycle registry entry: {raw_name}")
            registry[name] = SkillLifecycleSpec(
                name=name,
                conflict_group=str(raw_spec.get("conflict_group", "")).strip(),
                lifecycle_nodes=nodes,
                execution_interface=str(raw_spec.get("execution_interface", "")).strip(),
            )

        # Section: Fail startup if any dispatchable state machine can bypass manager policy.
        missing = sorted(ALLOWED_ACTION_TYPES - set(registry))
        unexpected = sorted(set(registry) - ALLOWED_ACTION_TYPES)
        if missing or unexpected:
            raise RuntimeError(
                f"Lifecycle registry/action mismatch: missing={missing}, unexpected={unexpected}"
            )
        return registry

    def prepare_callback(
        self,
        request: ManageLifecycleSkill.Request,
        response: ManageLifecycleSkill.Response,
    ) -> ManageLifecycleSkill.Response:
        """Special function: Prepare a group after cleaning every conflicting active group."""
        timeout_sec = self.request_timeout(request)
        with self._operation_lock:
            success, message = self.prepare_skill(str(request.skill_name), timeout_sec)
            return self.fill_manage_response(response, str(request.skill_name), success, message, timeout_sec)

    def cleanup_callback(
        self,
        request: ManageLifecycleSkill.Request,
        response: ManageLifecycleSkill.Response,
    ) -> ManageLifecycleSkill.Response:
        """Special function: Deactivate and clean one skill group in reverse dependency order."""
        timeout_sec = self.request_timeout(request)
        with self._operation_lock:
            success, message = self.cleanup_skill(str(request.skill_name), timeout_sec)
            return self.fill_manage_response(response, str(request.skill_name), success, message, timeout_sec)

    def recover_callback(
        self,
        request: ManageLifecycleSkill.Request,
        response: ManageLifecycleSkill.Response,
    ) -> ManageLifecycleSkill.Response:
        """Special function: Restore a known clean baseline and prepare the requested group once."""
        timeout_sec = self.request_timeout(request)
        skill_name = str(request.skill_name).strip().upper()
        with self._operation_lock:
            spec = self.registry.get(skill_name)
            if spec is None:
                return self.fill_manage_response(
                    response,
                    skill_name,
                    False,
                    f"Unknown GPSR skill: {skill_name}",
                    timeout_sec,
                )
            self._integrity_fault = ""

            # Section: Recovery first cleans the requested group and every conflict peer.
            cleaned, cleanup_message = self.cleanup_conflict_group_for_spec(
                spec,
                timeout_sec,
            )
            if not cleaned:
                self._integrity_fault = cleanup_message
                return self.fill_manage_response(
                    response,
                    skill_name,
                    False,
                    cleanup_message,
                    timeout_sec,
                )
            success, message = self.prepare_skill(skill_name, timeout_sec)
            return self.fill_manage_response(response, skill_name, success, message, timeout_sec)

    def get_state_callback(
        self,
        request: GetLifecycleSkillState.Request,
        response: GetLifecycleSkillState.Response,
    ) -> GetLifecycleSkillState.Response:
        """Special function: Return actual lifecycle states for a registry group."""
        skill_name = str(request.skill_name).strip().upper()
        timeout_sec = float(self.get_parameter("transition_timeout_sec").value)
        with self._operation_lock:
            spec = self.registry.get(skill_name)
            if spec is None:
                response.success = False
                response.message = f"Unknown GPSR skill: {skill_name}"
                response.state_json = "{}"
                return response
            states, error = self.group_states(spec, timeout_sec)
            response.success = not error
            response.message = error or "state query complete"
            response.state_json = json.dumps(self.state_payload(spec, states), sort_keys=True)
            return response

    def prepare_skill(self, skill_name: str, timeout_sec: float) -> tuple[bool, str]:
        """Special function: Resolve conflicts, configure, activate, and verify one group."""
        name = str(skill_name).strip().upper()
        spec = self.registry.get(name)
        if spec is None:
            return False, f"Unknown GPSR skill: {name}"
        if self._integrity_fault:
            return False, f"Lifecycle integrity fault requires recover_skill: {self._integrity_fault}"
        if not spec.lifecycle_nodes:
            self._prepared_skill = ""
            return True, f"{name} uses persistent or no managed lifecycle resources"

        # Section: A conflicting group must reach UNCONFIGURED before this group can start.
        conflicts_clean, conflict_message = self.cleanup_conflict_group(name, timeout_sec)
        if not conflicts_clean:
            self._integrity_fault = conflict_message
            return False, conflict_message

        # Section: Configure group dependencies in registry order.
        configured: list[str] = []
        for node_name in spec.lifecycle_nodes:
            state_id, _label, error = self.get_node_state(node_name, timeout_sec)
            if error:
                return self.rollback_prepare(name, spec, configured, error, timeout_sec)
            if state_id == State.PRIMARY_STATE_UNCONFIGURED:
                self.log_transition(name, node_name, "configuring")
                changed, change_error = self.change_state(
                    node_name,
                    Transition.TRANSITION_CONFIGURE,
                    timeout_sec,
                )
                if not changed:
                    return self.rollback_prepare(name, spec, configured, change_error, timeout_sec)
                configured.append(node_name)
            elif state_id == State.PRIMARY_STATE_INACTIVE:
                configured.append(node_name)
            elif state_id != State.PRIMARY_STATE_ACTIVE:
                return self.rollback_prepare(
                    name,
                    spec,
                    configured,
                    f"{node_name} cannot configure from state {state_id}",
                    timeout_sec,
                )

        # Section: Activate dependencies in registry order only after all configure steps pass.
        for node_name in spec.lifecycle_nodes:
            state_id, _label, error = self.get_node_state(node_name, timeout_sec)
            if error:
                return self.rollback_prepare(name, spec, configured, error, timeout_sec)
            if state_id == State.PRIMARY_STATE_INACTIVE:
                self.log_transition(name, node_name, "activating")
                changed, change_error = self.change_state(
                    node_name,
                    Transition.TRANSITION_ACTIVATE,
                    timeout_sec,
                )
                if not changed:
                    return self.rollback_prepare(name, spec, configured, change_error, timeout_sec)

        # Section: Verify every node is ACTIVE before exposing readiness to the executor.
        states, error = self.group_states(spec, timeout_sec)
        if error or any(state_id != State.PRIMARY_STATE_ACTIVE for state_id, _label in states.values()):
            message = error or f"{name} activation verification failed: {states}"
            return self.rollback_prepare(name, spec, configured, message, timeout_sec)
        self._prepared_skill = name
        self.get_logger().info(f"[GPSR_LIFECYCLE] {name}: ACTIVE")
        return True, f"{name} ACTIVE"

    def cleanup_skill(self, skill_name: str, timeout_sec: float) -> tuple[bool, str]:
        """Special function: Deactivate, clean, and verify a skill group."""
        name = str(skill_name).strip().upper()
        spec = self.registry.get(name)
        if spec is None:
            return False, f"Unknown GPSR skill: {name}"
        if not spec.lifecycle_nodes:
            return True, f"{name} has no managed lifecycle resources"

        # Section: Reverse order prevents servers from outliving their dependencies during cleanup.
        for node_name in reversed(spec.lifecycle_nodes):
            state_id, _label, error = self.get_node_state(node_name, timeout_sec)
            if error:
                return self.mark_integrity_failure(name, error)
            if state_id == State.PRIMARY_STATE_ACTIVE:
                self.log_transition(name, node_name, "deactivating")
                changed, change_error = self.change_state(
                    node_name,
                    Transition.TRANSITION_DEACTIVATE,
                    timeout_sec,
                )
                if not changed:
                    return self.mark_integrity_failure(name, change_error)

        # Section: Cleanup only INACTIVE nodes and tolerate already-clean nodes.
        for node_name in reversed(spec.lifecycle_nodes):
            state_id, _label, error = self.get_node_state(node_name, timeout_sec)
            if error:
                return self.mark_integrity_failure(name, error)
            if state_id == State.PRIMARY_STATE_INACTIVE:
                self.log_transition(name, node_name, "cleaning")
                changed, change_error = self.change_state(
                    node_name,
                    Transition.TRANSITION_CLEANUP,
                    timeout_sec,
                )
                if not changed:
                    return self.mark_integrity_failure(name, change_error)
            elif state_id != State.PRIMARY_STATE_UNCONFIGURED:
                return self.mark_integrity_failure(
                    name,
                    f"{node_name} cleanup blocked in state {state_id}",
                )

        # Section: The final UNCONFIGURED query is the queue advancement barrier.
        states, error = self.group_states(spec, timeout_sec)
        if error or any(state_id != State.PRIMARY_STATE_UNCONFIGURED for state_id, _label in states.values()):
            return self.mark_integrity_failure(
                name,
                error or f"{name} cleanup verification failed: {states}",
            )
        if self._prepared_skill == name:
            self._prepared_skill = ""
        self.get_logger().info(f"[GPSR_LIFECYCLE] {name}: UNCONFIGURED")
        return True, f"{name} UNCONFIGURED"

    def cleanup_conflict_group(self, requested_skill: str, timeout_sec: float) -> tuple[bool, str]:
        """Special function: Clean only tracked or actually active skills in the same conflict group."""
        requested = self.registry.get(requested_skill)
        if requested is None or not requested.conflict_group:
            return True, "no conflict group"

        # Section: Prefer manager ownership while still detecting externally activated groups.
        candidates: list[str] = []
        if self._prepared_skill and self._prepared_skill != requested_skill:
            active_spec = self.registry.get(self._prepared_skill)
            if active_spec and active_spec.conflict_group == requested.conflict_group:
                candidates.append(self._prepared_skill)
        for name, spec in self.registry.items():
            if name == requested_skill or name in candidates:
                continue
            if spec.conflict_group != requested.conflict_group or not spec.lifecycle_nodes:
                continue
            states, error = self.group_states(spec, timeout_sec)
            if error:
                return False, error
            if any(state_id == State.PRIMARY_STATE_ACTIVE for state_id, _label in states.values()):
                candidates.append(name)

        # Section: Never activate the requested group if any conflicting cleanup fails.
        for conflict_name in candidates:
            cleaned, message = self.cleanup_skill(conflict_name, timeout_sec)
            if not cleaned:
                return False, f"Cannot prepare {requested_skill}; conflict {conflict_name}: {message}"
        return True, "conflicts clean"

    def rollback_prepare(
        self,
        skill_name: str,
        spec: SkillLifecycleSpec,
        configured: list[str],
        failure: str,
        timeout_sec: float,
    ) -> tuple[bool, str]:
        """Special function: Best-effort rollback after configure or activate failure."""
        del configured

        # Section: Reuse the normal teardown path to close partial activation races.
        cleaned, cleanup_message = self.cleanup_skill(skill_name, timeout_sec)
        message = f"{skill_name} prepare failed: {failure}; rollback={cleanup_message}"
        if not cleaned:
            self._integrity_fault = message
        self.get_logger().error(f"[GPSR_LIFECYCLE] {message}")
        return False, message

    def cleanup_conflict_group_for_spec(
        self,
        spec: SkillLifecycleSpec,
        timeout_sec: float,
    ) -> tuple[bool, str]:
        """Special function: Clean every registry skill sharing a supplied conflict group."""
        for name, candidate in self.registry.items():
            if candidate.conflict_group != spec.conflict_group or not candidate.lifecycle_nodes:
                continue
            cleaned, message = self.cleanup_skill(name, timeout_sec)
            if not cleaned:
                return False, message
        return True, "conflict group clean"

    def group_states(
        self,
        spec: SkillLifecycleSpec,
        timeout_sec: float,
    ) -> tuple[dict[str, tuple[int, str]], str]:
        """Special function: Query every node state without using readiness sleeps."""
        states: dict[str, tuple[int, str]] = {}
        for node_name in spec.lifecycle_nodes:
            state_id, label, error = self.get_node_state(node_name, timeout_sec)
            if error:
                return states, error
            states[node_name] = (state_id, label)
        return states, ""

    def get_node_state(self, node_name: str, timeout_sec: float) -> tuple[int, str, str]:
        """Special function: Call one lifecycle GetState service with a bounded wait."""
        client = self._state_clients[node_name]
        if not client.wait_for_service(timeout_sec=min(2.0, timeout_sec)):
            return 0, "unknown", f"Lifecycle state service unavailable: {node_name}"
        future = client.call_async(GetState.Request())
        if not self.wait_for_future(future, timeout_sec):
            return 0, "unknown", f"Lifecycle state query timed out: {node_name}"
        response = future.result()
        if response is None:
            return 0, "unknown", f"Lifecycle state query returned no response: {node_name}"
        return int(response.current_state.id), str(response.current_state.label), ""

    def change_state(self, node_name: str, transition_id: int, timeout_sec: float) -> tuple[bool, str]:
        """Special function: Request one transition and return its service-level result."""
        client = self._change_clients[node_name]
        if not client.wait_for_service(timeout_sec=min(2.0, timeout_sec)):
            return False, f"Lifecycle transition service unavailable: {node_name}"
        request = ChangeState.Request()
        request.transition.id = int(transition_id)
        future = client.call_async(request)
        if not self.wait_for_future(future, timeout_sec):
            return False, f"Lifecycle transition timed out: {node_name} transition={transition_id}"
        response = future.result()
        if response is None or not bool(response.success):
            return False, f"Lifecycle transition rejected: {node_name} transition={transition_id}"
        return True, "transition complete"

    @staticmethod
    def wait_for_future(future: Any, timeout_sec: float) -> bool:
        """Special function: Wait on an externally spun future without arbitrary readiness sleeps."""
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        return bool(completed.wait(timeout=max(0.1, timeout_sec)) or future.done())

    def state_payload(
        self,
        spec: SkillLifecycleSpec,
        states: dict[str, tuple[int, str]],
    ) -> dict[str, Any]:
        """Special function: Build a stable state JSON contract for clients and operators."""
        return {
            "skill_name": spec.name,
            "conflict_group": spec.conflict_group,
            "execution_interface": spec.execution_interface,
            "prepared_skill": self._prepared_skill,
            "integrity_fault": self._integrity_fault,
            "nodes": {
                node_name: {"id": state_id, "label": label}
                for node_name, (state_id, label) in states.items()
            },
        }

    def fill_manage_response(
        self,
        response: ManageLifecycleSkill.Response,
        skill_name: str,
        success: bool,
        message: str,
        timeout_sec: float,
    ) -> ManageLifecycleSkill.Response:
        """Special function: Fill operation response with a best-effort actual state snapshot."""
        spec = self.registry.get(str(skill_name).strip().upper())
        states: dict[str, tuple[int, str]] = {}
        if spec is not None:
            states, _error = self.group_states(spec, timeout_sec)
        response.success = bool(success)
        response.message = str(message)
        response.state_json = json.dumps(
            {} if spec is None else self.state_payload(spec, states),
            sort_keys=True,
        )
        return response

    def mark_integrity_failure(self, skill_name: str, message: str) -> tuple[bool, str]:
        """Special function: Latch teardown uncertainty so conflicting activation fails closed."""
        self._integrity_fault = f"{skill_name}: {message}"
        self.get_logger().error(f"[GPSR_LIFECYCLE] integrity failure: {self._integrity_fault}")
        return False, self._integrity_fault

    def request_timeout(self, request: ManageLifecycleSkill.Request) -> float:
        """Special function: Resolve a positive request timeout against the configured default."""
        requested = float(request.timeout_sec)
        configured = float(self.get_parameter("transition_timeout_sec").value)
        return max(0.1, requested if requested > 0.0 else configured)

    def log_transition(self, skill_name: str, node_name: str, operation: str) -> None:
        """Special function: Emit one concise state-change log per node transition."""
        self.get_logger().info(
            f"[GPSR_LIFECYCLE] {skill_name}: {operation} {node_name}"
        )


def main(args: list[str] | None = None) -> None:
    """Special function: Run the persistent central GPSR lifecycle manager."""
    rclpy.init(args=args)
    node = GpsrLifecycleManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
