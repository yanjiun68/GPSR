"""Section: Central registry-driven lifecycle manager for GPSR skill groups."""

from __future__ import annotations

import json
import threading
import time
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
from robot_yasmin_tasks.gpsr.action_types import (
    GPSR_CLEANUP_SKILL_SERVICE,
    GPSR_GET_SKILL_STATE_SERVICE,
    GPSR_PREPARE_SKILL_SERVICE,
    GPSR_RECOVER_SKILL_SERVICE,
    load_primitive_registry,
)


@dataclass(frozen=True, slots=True)
class SkillLifecycleSpec:
    """Special function: Hold one immutable ActionStep-to-node registry entry."""

    name: str
    conflict_group: str
    lifecycle_nodes: tuple[str, ...]
    execution_interface: str
    interface_type: str = ""
    implementation: str = "real"
    production_enabled: bool = True
    legacy_aliases: tuple[str, ...] = ()


class GpsrLifecycleManager(Node):
    """Special function: Configure, activate, deactivate, and clean GPSR skill groups."""

    def __init__(self) -> None:
        """Special function: Load the registry and expose serialized lifecycle operations."""
        super().__init__("gpsr_lifecycle_manager")

        # Section: Declare the one authoritative registry and bounded transition timeout.
        self.declare_parameter("registry_file", self.default_registry_path())
        self.declare_parameter("transition_timeout_sec", 60.0)
        self.declare_parameter("allow_simulated_primitives", False)

        # Section: Serialize complete group operations so conflicts cannot race.
        self._operation_lock = threading.RLock()
        # Section: Teardown uncertainty blocks only skills in the affected resource domain.
        self._integrity_faults: dict[str, str] = {}
        self._prepared_skill = ""
        # Section: Report each absent optional conflict peer at most once per manager process.
        self._warned_unlaunched_peers: set[str] = set()
        self._callback_group = ReentrantCallbackGroup()
        self.allow_simulated_primitives = bool(
            self.get_parameter("allow_simulated_primitives").value
        )
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
            GPSR_PREPARE_SKILL_SERVICE,
            self.prepare_callback,
            callback_group=self._callback_group,
        )
        self._cleanup_service = self.create_service(
            ManageLifecycleSkill,
            GPSR_CLEANUP_SKILL_SERVICE,
            self.cleanup_callback,
            callback_group=self._callback_group,
        )
        self._recover_service = self.create_service(
            ManageLifecycleSkill,
            GPSR_RECOVER_SKILL_SERVICE,
            self.recover_callback,
            callback_group=self._callback_group,
        )
        self._state_service = self.create_service(
            GetLifecycleSkillState,
            GPSR_GET_SKILL_STATE_SERVICE,
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
        capabilities = load_primitive_registry(registry_file)

        # Section: Adapt the authoritative capability records to lifecycle transition policy.
        return {
            name: SkillLifecycleSpec(
                name=spec.name,
                conflict_group=spec.conflict_group,
                lifecycle_nodes=spec.lifecycle_nodes,
                execution_interface=spec.execution_interface,
                interface_type=spec.interface_type,
                implementation=spec.implementation,
                production_enabled=spec.production_enabled,
                legacy_aliases=spec.legacy_aliases,
            )
            for name, spec in capabilities.items()
        }

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
        deadline = time.monotonic() + timeout_sec
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
            fault_group = spec.conflict_group

            # Section: Recovery first cleans the requested group and present conflict peers.
            cleaned, cleanup_message = self.cleanup_conflict_group_for_spec(
                spec,
                timeout_sec,
                deadline=deadline,
            )
            if not cleaned:
                return self.fill_manage_response(
                    response,
                    skill_name,
                    False,
                    cleanup_message,
                    timeout_sec,
                )
            # Section: A proven clean baseline clears only this resource domain's fault.
            self._integrity_faults.pop(fault_group, None)
            remaining = self.remaining_timeout(deadline)
            if remaining <= 0.0:
                return self.fill_manage_response(
                    response,
                    skill_name,
                    False,
                    "Lifecycle recovery deadline expired before prepare.",
                    timeout_sec,
                )
            success, message = self.prepare_skill(skill_name, remaining)
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
        # Section: One absolute deadline bounds the complete group operation.
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        name = str(skill_name).strip().upper()
        spec = self.registry.get(name)
        if spec is None:
            return False, f"Unknown GPSR skill: {name}"
        production_available = spec.implementation == "real" and spec.production_enabled
        test_simulation_available = (
            bool(getattr(self, "allow_simulated_primitives", False))
            and spec.implementation == "simulated"
        )
        if not production_available and not test_simulation_available:
            return False, (
                f"Primitive unavailable: {name} implementation={spec.implementation} "
                f"production_enabled={spec.production_enabled}"
            )
        fault_message = self._integrity_faults.get(spec.conflict_group, "")
        if fault_message:
            return False, (
                "Lifecycle integrity fault requires recover_skill: "
                f"group={spec.conflict_group} reason={fault_message}"
            )
        if not spec.lifecycle_nodes:
            self._prepared_skill = ""
            return True, f"{name} uses persistent or no managed lifecycle resources"

        # Section: Prove every requested interface and initial state before any transition.
        initial_states, preflight_error = self.preflight_required_group(spec, deadline)
        if preflight_error:
            return False, f"{name} unavailable: {preflight_error}"
        active_nodes = {
            node_name
            for node_name, (state_id, _label) in initial_states.items()
            if state_id == State.PRIMARY_STATE_ACTIVE
        }
        owner_spec = self.registry.get(self._prepared_skill)
        active_nodes_owned = bool(
            owner_spec
            and owner_spec.conflict_group == spec.conflict_group
            and active_nodes.issubset(set(owner_spec.lifecycle_nodes))
        )
        if active_nodes and self._prepared_skill != name and not active_nodes_owned:
            message = f"{name} has ACTIVE lifecycle resources without manager ownership"
            self.mark_integrity_failure(name, message)
            return False, message

        # Section: A conflicting group must reach UNCONFIGURED before this group can start.
        conflicts_clean, conflict_message = self.cleanup_conflict_group(
            name,
            timeout_sec,
            deadline=deadline,
        )
        if not conflicts_clean:
            return False, conflict_message

        # Section: Configure group dependencies in registry order.
        # Section: Pre-existing INACTIVE resources also require rollback if prepare later fails.
        affected = [
            node_name
            for node_name, (state_id, _label) in initial_states.items()
            if state_id != State.PRIMARY_STATE_UNCONFIGURED
        ]
        for node_name in spec.lifecycle_nodes:
            remaining = self.remaining_timeout(deadline)
            if remaining <= 0.0:
                return self.rollback_prepare(name, spec, affected, "prepare deadline expired during configure", 0.1)
            state_id, _label, error = self.get_node_state(node_name, remaining)
            if error:
                return self.rollback_prepare(name, spec, affected, error, max(0.1, self.remaining_timeout(deadline)))
            if state_id == State.PRIMARY_STATE_UNCONFIGURED:
                self.log_transition(name, node_name, "configuring")
                changed, change_error = self.change_state(
                    node_name,
                    Transition.TRANSITION_CONFIGURE,
                    remaining,
                )
                if not changed:
                    return self.rollback_prepare(name, spec, affected, change_error, max(0.1, self.remaining_timeout(deadline)))
                if node_name not in affected:
                    affected.append(node_name)
            elif state_id == State.PRIMARY_STATE_INACTIVE:
                continue
            elif state_id != State.PRIMARY_STATE_ACTIVE:
                return self.rollback_prepare(
                    name,
                    spec,
                    affected,
                    f"{node_name} cannot configure from state {state_id}",
                    max(0.1, self.remaining_timeout(deadline)),
                )

        # Section: Activate dependencies in registry order only after all configure steps pass.
        for node_name in spec.lifecycle_nodes:
            remaining = self.remaining_timeout(deadline)
            if remaining <= 0.0:
                return self.rollback_prepare(name, spec, affected, "prepare deadline expired during activation", 0.1)
            state_id, _label, error = self.get_node_state(node_name, remaining)
            if error:
                return self.rollback_prepare(name, spec, affected, error, max(0.1, self.remaining_timeout(deadline)))
            if state_id == State.PRIMARY_STATE_INACTIVE:
                self.log_transition(name, node_name, "activating")
                changed, change_error = self.change_state(
                    node_name,
                    Transition.TRANSITION_ACTIVATE,
                    remaining,
                )
                if not changed:
                    return self.rollback_prepare(name, spec, affected, change_error, max(0.1, self.remaining_timeout(deadline)))
                if node_name not in affected:
                    affected.append(node_name)

        # Section: Verify every node is ACTIVE before exposing readiness to the executor.
        remaining = self.remaining_timeout(deadline)
        if remaining <= 0.0:
            return self.rollback_prepare(name, spec, affected, "prepare deadline expired before verification", 0.1)
        states, error = self.group_states(spec, remaining, deadline=deadline)
        if error or any(state_id != State.PRIMARY_STATE_ACTIVE for state_id, _label in states.values()):
            message = error or f"{name} activation verification failed: {states}"
            return self.rollback_prepare(name, spec, affected, message, max(0.1, self.remaining_timeout(deadline)))
        self._prepared_skill = name
        self.get_logger().info(f"[GPSR_LIFECYCLE] {name}: ACTIVE")
        return True, f"{name} ACTIVE"

    def cleanup_skill(
        self,
        skill_name: str,
        timeout_sec: float,
        deadline: float | None = None,
    ) -> tuple[bool, str]:
        """Special function: Deactivate, clean, and verify a skill group."""
        # Section: Nested conflict cleanup shares the caller's absolute deadline.
        operation_deadline = deadline or (time.monotonic() + max(0.1, float(timeout_sec)))
        name = str(skill_name).strip().upper()
        spec = self.registry.get(name)
        if spec is None:
            return False, f"Unknown GPSR skill: {name}"
        if not spec.lifecycle_nodes:
            return True, f"{name} has no managed lifecycle resources"

        # Section: Reverse order prevents servers from outliving their dependencies during cleanup.
        for node_name in reversed(spec.lifecycle_nodes):
            remaining = self.remaining_timeout(operation_deadline)
            if remaining <= 0.0:
                return self.mark_integrity_failure(name, "cleanup deadline expired during deactivate")
            state_id, _label, error = self.get_node_state(node_name, remaining)
            if error:
                return self.mark_integrity_failure(name, error)
            if state_id == State.PRIMARY_STATE_ACTIVE:
                self.log_transition(name, node_name, "deactivating")
                changed, change_error = self.change_state(
                    node_name,
                    Transition.TRANSITION_DEACTIVATE,
                    remaining,
                )
                if not changed:
                    return self.mark_integrity_failure(name, change_error)

        # Section: Cleanup only INACTIVE nodes and tolerate already-clean nodes.
        for node_name in reversed(spec.lifecycle_nodes):
            remaining = self.remaining_timeout(operation_deadline)
            if remaining <= 0.0:
                return self.mark_integrity_failure(name, "cleanup deadline expired during cleanup")
            state_id, _label, error = self.get_node_state(node_name, remaining)
            if error:
                return self.mark_integrity_failure(name, error)
            if state_id == State.PRIMARY_STATE_INACTIVE:
                self.log_transition(name, node_name, "cleaning")
                changed, change_error = self.change_state(
                    node_name,
                    Transition.TRANSITION_CLEANUP,
                    remaining,
                )
                if not changed:
                    return self.mark_integrity_failure(name, change_error)
            elif state_id != State.PRIMARY_STATE_UNCONFIGURED:
                return self.mark_integrity_failure(
                    name,
                    f"{node_name} cleanup blocked in state {state_id}",
                )

        # Section: The final UNCONFIGURED query is the queue advancement barrier.
        remaining = self.remaining_timeout(operation_deadline)
        if remaining <= 0.0:
            return self.mark_integrity_failure(name, "cleanup deadline expired before verification")
        states, error = self.group_states(spec, remaining, deadline=operation_deadline)
        if error or any(state_id != State.PRIMARY_STATE_UNCONFIGURED for state_id, _label in states.values()):
            return self.mark_integrity_failure(
                name,
                error or f"{name} cleanup verification failed: {states}",
            )
        if self._prepared_skill == name:
            self._prepared_skill = ""
        self.get_logger().info(f"[GPSR_LIFECYCLE] {name}: UNCONFIGURED")
        return True, f"{name} UNCONFIGURED"
    def cleanup_conflict_group(
        self,
        requested_skill: str,
        timeout_sec: float,
        deadline: float | None = None,
    ) -> tuple[bool, str]:
        """Special function: Clean only tracked or actually active skills in the same conflict group."""
        operation_deadline = deadline or (time.monotonic() + max(0.1, float(timeout_sec)))
        requested = self.registry.get(requested_skill)
        if requested is None or not requested.conflict_group:
            return True, "no conflict group"

        # Section: Strictly clean a conflicting group that the manager knows it owns.
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
            presence = {
                node_name: self.state_service_is_ready(node_name)
                for node_name in spec.lifecycle_nodes
            }
            reachable = [node_name for node_name, ready in presence.items() if ready]
            if not reachable:
                self.log_unlaunched_peer_once(name)
                continue
            states, error = self.reachable_states(reachable, operation_deadline)
            if error:
                return False, f"Cannot inspect conflict peer {name}: {error}"
            missing = [node_name for node_name, ready in presence.items() if not ready]
            non_clean = {
                node_name: state
                for node_name, state in states.items()
                if state[0] != State.PRIMARY_STATE_UNCONFIGURED
            }
            if missing and non_clean:
                message = (
                    f"Cannot prepare {requested_skill}; partial conflict peer {name} "
                    f"has missing nodes {missing} and non-clean states {non_clean}"
                )
                self.mark_integrity_failure(name, message)
                return False, message
            if missing:
                self.log_unlaunched_peer_once(name, partial=True)
                continue
            if non_clean:
                candidates.append(name)

        # Section: Never activate the requested group if any conflicting cleanup fails.
        for conflict_name in candidates:
            remaining = self.remaining_timeout(operation_deadline)
            if remaining <= 0.0:
                return False, "conflict cleanup deadline expired before teardown"
            cleaned, message = self.cleanup_skill(
                conflict_name,
                remaining,
                deadline=operation_deadline,
            )
            if not cleaned:
                return False, f"Cannot prepare {requested_skill}; conflict {conflict_name}: {message}"
        return True, "conflicts clean"

    def rollback_prepare(
        self,
        skill_name: str,
        spec: SkillLifecycleSpec,
        affected: list[str],
        failure: str,
        timeout_sec: float,
    ) -> tuple[bool, str]:
        """Special function: Best-effort rollback after configure or activate failure."""
        # Section: Pre-transition availability failures require no destructive rollback.
        if not affected:
            message = f"{skill_name} prepare failed before transition: {failure}"
            self.get_logger().error(f"[GPSR_LIFECYCLE] {message}")
            return False, message

        # Section: Reuse normal teardown only after this prepare changed lifecycle state.
        cleaned, cleanup_message = self.cleanup_skill(skill_name, timeout_sec)
        message = f"{skill_name} prepare failed: {failure}; rollback={cleanup_message}"
        self.get_logger().error(f"[GPSR_LIFECYCLE] {message}")
        return False, message

    def cleanup_conflict_group_for_spec(
        self,
        spec: SkillLifecycleSpec,
        timeout_sec: float,
        deadline: float | None = None,
    ) -> tuple[bool, str]:
        """Special function: Clean the requested group and present peers for bounded recovery."""
        operation_deadline = deadline or (time.monotonic() + max(0.1, float(timeout_sec)))
        # Section: Requested resources are mandatory even during recovery.
        _states, preflight_error = self.preflight_required_group(
            spec,
            operation_deadline,
            allow_non_primary=False,
        )
        if preflight_error:
            return False, f"Recovery unavailable for {spec.name}: {preflight_error}"
        remaining = self.remaining_timeout(operation_deadline)
        cleaned, message = self.cleanup_skill(spec.name, remaining, deadline=operation_deadline)
        if not cleaned:
            return False, message

        # Section: Presence-aware discovery ignores absent or inert optional peers.
        cleaned, message = self.cleanup_conflict_group(
            spec.name,
            self.remaining_timeout(operation_deadline),
            deadline=operation_deadline,
        )
        if not cleaned:
            return False, message
        return True, "conflict group clean"

    def preflight_required_group(
        self,
        spec: SkillLifecycleSpec,
        deadline: float,
        allow_non_primary: bool = False,
    ) -> tuple[dict[str, tuple[int, str]], str]:
        """Special function: Verify requested services and states before changing any node."""
        states: dict[str, tuple[int, str]] = {}
        allowed_states = {
            State.PRIMARY_STATE_UNCONFIGURED,
            State.PRIMARY_STATE_INACTIVE,
            State.PRIMARY_STATE_ACTIVE,
        }
        for node_name in spec.lifecycle_nodes:
            remaining = self.remaining_timeout(deadline)
            if remaining <= 0.0:
                return states, "required lifecycle preflight deadline expired"
            state_ready, change_ready = self.lifecycle_services_available(
                node_name,
                remaining,
            )
            if not state_ready:
                return states, f"required lifecycle service missing: {node_name}/get_state"
            if not change_ready:
                return states, f"required lifecycle service missing: {node_name}/change_state"
            state_id, label, error = self.get_node_state(node_name, remaining)
            if error:
                return states, error
            states[node_name] = (state_id, label)
            if not allow_non_primary and state_id not in allowed_states:
                return states, f"{node_name} has unsupported lifecycle state {state_id} ({label})"
        return states, ""

    def lifecycle_services_available(
        self,
        node_name: str,
        timeout_sec: float,
    ) -> tuple[bool, bool]:
        """Special function: Wait boundedly for both required lifecycle interfaces."""
        state_client = self._state_clients[node_name]
        change_client = self._change_clients[node_name]
        started_at = time.monotonic()
        state_ready = bool(state_client.wait_for_service(timeout_sec=min(2.0, timeout_sec)))
        remaining = max(0.0, timeout_sec - (time.monotonic() - started_at))
        change_ready = bool(
            change_client.wait_for_service(timeout_sec=min(2.0, remaining))
        )
        return state_ready, change_ready

    def state_service_is_ready(self, node_name: str) -> bool:
        """Special function: Snapshot optional peer presence without blocking preparation."""
        client = self._state_clients[node_name]
        checker = getattr(client, "service_is_ready", None)
        if callable(checker):
            return bool(checker())
        return bool(client.wait_for_service(timeout_sec=0.0))

    def reachable_states(
        self,
        node_names: list[str],
        deadline: float,
    ) -> tuple[dict[str, tuple[int, str]], str]:
        """Special function: Query only lifecycle nodes proven present during conflict discovery."""
        states: dict[str, tuple[int, str]] = {}
        for node_name in node_names:
            remaining = self.remaining_timeout(deadline)
            if remaining <= 0.0:
                return states, "conflict state-query deadline expired"
            state_id, label, error = self.get_node_state(node_name, remaining)
            if error:
                return states, error
            states[node_name] = (state_id, label)
        return states, ""

    def log_unlaunched_peer_once(self, skill_name: str, partial: bool = False) -> None:
        """Special function: Log one concise diagnostic for an absent inert optional peer."""
        key = f"{skill_name}:{'partial' if partial else 'absent'}"
        if key in self._warned_unlaunched_peers:
            return
        self._warned_unlaunched_peers.add(key)
        qualifier = "partially launched but inert" if partial else "unlaunched"
        self.get_logger().debug(
            f"[GPSR_LIFECYCLE] skipping {qualifier} conflict peer {skill_name}"
        )

    def group_states(
        self,
        spec: SkillLifecycleSpec,
        timeout_sec: float,
        deadline: float | None = None,
    ) -> tuple[dict[str, tuple[int, str]], str]:
        """Special function: Query every node state without using readiness sleeps."""
        operation_deadline = deadline or (time.monotonic() + max(0.1, float(timeout_sec)))
        states: dict[str, tuple[int, str]] = {}
        for node_name in spec.lifecycle_nodes:
            remaining = self.remaining_timeout(operation_deadline)
            if remaining <= 0.0:
                return states, "lifecycle state-query deadline expired"
            state_id, label, error = self.get_node_state(node_name, remaining)
            if error:
                return states, error
            states[node_name] = (state_id, label)
        return states, ""

    @staticmethod
    def remaining_timeout(deadline: float) -> float:
        """Special function: Return seconds left under one absolute operation deadline."""
        return max(0.0, float(deadline) - time.monotonic())

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
            "interface_type": spec.interface_type,
            "implementation": spec.implementation,
            "production_enabled": spec.production_enabled,
            "prepared_skill": self._prepared_skill,
            "integrity_fault_group": spec.conflict_group if spec.conflict_group in self._integrity_faults else "",
            "integrity_fault": self._integrity_faults.get(spec.conflict_group, ""),
            "integrity_faults": dict(self._integrity_faults),
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
        spec = self.registry.get(str(skill_name).strip().upper())
        group = spec.conflict_group if spec is not None else ""
        fault = f"{skill_name}: {message}"
        self._integrity_faults[group] = fault
        self.get_logger().error(
            f"[GPSR_LIFECYCLE] integrity failure: group={group} reason={fault}"
        )
        return False, fault

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
