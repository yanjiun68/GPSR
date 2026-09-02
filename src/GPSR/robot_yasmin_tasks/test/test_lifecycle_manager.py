"""Section: Central GPSR lifecycle-manager transition policy tests."""

import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
from lifecycle_msgs.msg import State, Transition

from robot_yasmin_tasks.gpsr.action_types import (
    ALLOWED_ACTION_TYPES,
    GPSR_CLEANUP_SKILL_SERVICE,
    GPSR_GET_SKILL_STATE_SERVICE,
    GPSR_LIFECYCLE_SERVICE_PREFIX,
    GPSR_MANAGED_LIFECYCLE_NODES,
    GPSR_PREPARE_SKILL_SERVICE,
    GPSR_RECOVER_SKILL_SERVICE,
)
from robot_yasmin_tasks.gpsr.lifecycle_manager import (
    GpsrLifecycleManager,
    SkillLifecycleSpec,
)


def test_registry_covers_every_dispatchable_skill_state_machine() -> None:
    """Special function: Require exact lifecycle policy coverage for every ActionStep."""
    manager = object.__new__(GpsrLifecycleManager)
    registry_path = Path(__file__).parents[1] / "config" / "skill_registry.yaml"
    registry = manager.load_registry(str(registry_path))
    assert set(registry) == ALLOWED_ACTION_TYPES


def test_registry_manages_only_approved_transient_gpsr_nodes() -> None:
    """Special function: Keep hardware, arm, camera, and Nav2 nodes persistently external."""
    # Section: Load the production registry and collect every transition target.
    manager = object.__new__(GpsrLifecycleManager)
    registry_path = Path(__file__).parents[1] / "config" / "skill_registry.yaml"
    registry = manager.load_registry(str(registry_path))
    managed_nodes = {
        node_name
        for specification in registry.values()
        for node_name in specification.lifecycle_nodes
    }

    # Section: The complete configured target set must be inside the code-owned allowlist.
    assert managed_nodes <= GPSR_MANAGED_LIFECYCLE_NODES
    assert not any("my_robot_nav" in node_name for node_name in managed_nodes)
    assert not any("arm" in node_name or "rx200" in node_name for node_name in managed_nodes)


@pytest.mark.parametrize(
    "unapproved_node",
    [
        "/my_robot_nav",
        "/lifecycle_manager_navigation",
        "/controller_server",
        "/smoother_server",
        "/planner_server",
        "/behavior_server",
        "/bt_navigator",
        "/waypoint_follower",
        "/velocity_smoother",
        "/map_server",
        "/amcl",
        "/base_driver",
        "/controller_manager",
        "/rx200_arm_driver",
        "/camera/camera",
    ],
)
def test_registry_rejects_non_gpsr_lifecycle_targets(
    tmp_path: Path,
    unapproved_node: str,
) -> None:
    """Special function: Refuse startup if configuration tries to own persistent drivers."""
    # Section: Copy the complete registry so exact primitive coverage remains valid.
    production_path = Path(__file__).parents[1] / "config" / "skill_registry.yaml"
    payload = json.loads(production_path.read_text(encoding="utf-8"))
    payload["skills"]["NAVIGATE"]["lifecycle_nodes"] = [unapproved_node]
    unsafe_path = tmp_path / "unsafe_skill_registry.yaml"
    unsafe_path.write_text(json.dumps(payload), encoding="utf-8")

    # Section: Registry admission must fail before any lifecycle client is created.
    manager = object.__new__(GpsrLifecycleManager)
    with pytest.raises(RuntimeError, match="non-GPSR lifecycle resources"):
        manager.load_registry(str(unsafe_path))


def test_gpsr_control_services_are_isolated_from_nav2_manager() -> None:
    """Special function: Reserve a GPSR-only lifecycle control namespace."""
    # Section: Every public GPSR manager service must remain under one prefix.
    gpsr_services = {
        GPSR_PREPARE_SKILL_SERVICE,
        GPSR_CLEANUP_SKILL_SERVICE,
        GPSR_RECOVER_SKILL_SERVICE,
        GPSR_GET_SKILL_STATE_SERVICE,
    }
    assert all(
        service_name.startswith(f"{GPSR_LIFECYCLE_SERVICE_PREFIX}/")
        for service_name in gpsr_services
    )

    # Section: GPSR must never expose or consume Nav2 lifecycle-manager APIs.
    assert not any(
        service_name.startswith("/lifecycle_manager_navigation/")
        for service_name in gpsr_services
    )


def test_person_routes_report_truthful_resource_ownership() -> None:
    """Special function: Both person routes reuse one managed tracker without duplicate nodes."""
    # Section: Load production policy rather than duplicating its node lists in the test.
    manager = object.__new__(GpsrLifecycleManager)
    registry_path = Path(__file__).parents[1] / "config" / "skill_registry.yaml"
    registry = manager.load_registry(str(registry_path))
    identity_nodes = set(registry["FIND_PERSON"].lifecycle_nodes)
    attribute_nodes = set(registry["FIND_PERSON_BY_ATTRIBUTE"].lifecycle_nodes)
    assert registry["FIND_PERSON"].conflict_group == "vision_gpu"
    assert identity_nodes == {"/gpsr_human_identity_node"}
    assert "/find_person_by_attribute_skill_server" in attribute_nodes
    assert "/gpsr_tracked_appearance_node" in attribute_nodes
    assert registry["FIND_PERSON_BY_ATTRIBUTE"].conflict_group == "vision_gpu"
    assert identity_nodes <= attribute_nodes


def test_extract_person_info_reuses_tracked_perception_and_owns_its_server() -> None:
    """Special function: Extraction shares perception but dispatches through a dedicated server."""
    manager = object.__new__(GpsrLifecycleManager)
    registry_path = Path(__file__).parents[1] / "config" / "skill_registry.yaml"
    registry = manager.load_registry(str(registry_path))
    find_nodes = set(registry["FIND_PERSON_BY_ATTRIBUTE"].lifecycle_nodes)
    extract_nodes = set(registry["EXTRACT_PERSON_INFO"].lifecycle_nodes)
    assert {"/gpsr_human_identity_node", "/human_skeleton_detection_node", "/gpsr_tracked_appearance_node"} <= find_nodes & extract_nodes
    assert "/extract_person_info_skill_server" in extract_nodes


def test_registry_rejects_a_skill_without_lifecycle_policy(tmp_path: Path) -> None:
    """Special function: Prevent newly dispatched state machines from bypassing the manager."""
    registry_path = tmp_path / "incomplete_registry.yaml"
    registry_path.write_text('{"skills": {"SAY": {"lifecycle_nodes": []}}}', encoding="utf-8")
    manager = object.__new__(GpsrLifecycleManager)
    with pytest.raises(RuntimeError, match="Primitive registry/action mismatch"):
        manager.load_registry(str(registry_path))


def test_missing_physical_primitives_are_disabled_in_production() -> None:
    """Special function: Registry metadata prevents simulation-only production success."""
    manager = object.__new__(GpsrLifecycleManager)
    registry_path = Path(__file__).parents[1] / "config" / "skill_registry.yaml"
    registry = manager.load_registry(str(registry_path))
    for primitive in {"PICK_OBJECT", "PLACE_OBJECT", "HANDOVER_OBJECT", "GUIDE_PERSON"}:
        assert registry[primitive].implementation == "missing"
        assert registry[primitive].production_enabled is False


def test_follow_person_uses_real_lifecycle_adapter() -> None:
    """Special function: FOLLOW_PERSON cannot bypass its master-stack action bridge."""
    manager = object.__new__(GpsrLifecycleManager)
    registry_path = Path(__file__).parents[1] / "config" / "skill_registry.yaml"
    registry = manager.load_registry(str(registry_path))
    follow = registry["FOLLOW_PERSON"]
    assert follow.implementation == "real"
    assert follow.production_enabled is True
    assert follow.lifecycle_nodes == ("/follow_person_skill_server",)
    assert follow.execution_interface == "/person_follow/control"


class FakeLogger:
    """Special function: Accept manager transition logs in pure unit tests."""

    def info(self, _message: str) -> None:
        """Special function: Ignore informational test logs."""
        return None

    def error(self, _message: str) -> None:
        """Special function: Ignore expected failure logs."""
        return None

    def debug(self, _message: str) -> None:
        """Special function: Ignore optional-peer discovery diagnostics."""
        return None


def make_manager(
    fail_transition: tuple[str, int] | None = None,
) -> tuple[GpsrLifecycleManager, dict[str, int]]:
    """Special function: Build a manager policy harness with deterministic lifecycle states."""
    manager = object.__new__(GpsrLifecycleManager)
    manager.registry = {
        "COUNT_OBJECTS": SkillLifecycleSpec(
            "COUNT_OBJECTS",
            "vision_gpu",
            ("/object_detector", "/count_objects"),
            "/gpsr/count_objects",
        ),
        "FIND_OBJECT": SkillLifecycleSpec(
            "FIND_OBJECT",
            "vision_gpu",
            ("/object_detector", "/find_object"),
            "/gpsr/find_object",
        ),
        "COUNT_PEOPLE": SkillLifecycleSpec(
            "COUNT_PEOPLE",
            "vision_gpu",
            ("/human_detector", "/count_people"),
            "/gpsr/count_people",
        ),
        "NAVIGATE": SkillLifecycleSpec(
            "NAVIGATE",
            "navigation_motion",
            (),
            "/nav/navigate_named_pose",
        ),
        "FOLLOW_PERSON": SkillLifecycleSpec(
            "FOLLOW_PERSON",
            "navigation_motion",
            ("/follow_person",),
            "/person_follow/control",
        ),
        "SAY": SkillLifecycleSpec("SAY", "", (), "/voice/tts/say"),
    }
    states = {
        node_name: State.PRIMARY_STATE_UNCONFIGURED
        for spec in manager.registry.values()
        for node_name in spec.lifecycle_nodes
    }
    manager._integrity_faults = {}
    manager._prepared_skill = ""
    manager._warned_unlaunched_peers = set()
    manager._operation_lock = threading.RLock()
    manager.available_nodes = set(states)
    manager.transition_events = []
    manager.get_logger = lambda: FakeLogger()
    manager.log_transition = lambda _skill, _node, _operation: None
    manager.lifecycle_services_available = lambda node_name, _timeout: (
        node_name in manager.available_nodes,
        node_name in manager.available_nodes,
    )
    manager.state_service_is_ready = lambda node_name: node_name in manager.available_nodes

    def get_node_state(node_name: str, _timeout_sec: float):
        """Special function: Return the fake node's current primary state."""
        state_id = states[node_name]
        labels = {
            State.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
            State.PRIMARY_STATE_INACTIVE: "inactive",
            State.PRIMARY_STATE_ACTIVE: "active",
        }
        return state_id, labels[state_id], ""

    def change_state(node_name: str, transition_id: int, _timeout_sec: float):
        """Special function: Apply or reject one deterministic fake transition."""
        manager.transition_events.append((node_name, transition_id))
        if fail_transition == (node_name, transition_id):
            return False, f"forced transition failure: {node_name}:{transition_id}"
        targets = {
            Transition.TRANSITION_CONFIGURE: State.PRIMARY_STATE_INACTIVE,
            Transition.TRANSITION_ACTIVATE: State.PRIMARY_STATE_ACTIVE,
            Transition.TRANSITION_DEACTIVATE: State.PRIMARY_STATE_INACTIVE,
            Transition.TRANSITION_CLEANUP: State.PRIMARY_STATE_UNCONFIGURED,
        }
        states[node_name] = targets[transition_id]
        return True, "complete"

    manager.get_node_state = get_node_state
    manager.change_state = change_state
    return manager, states


def test_single_skill_full_cycle_reaches_unconfigured() -> None:
    """Special function: COUNT_OBJECTS follows configure/activate/deactivate/cleanup."""
    manager, states = make_manager()
    assert manager.prepare_skill("COUNT_OBJECTS", 1.0)[0]
    assert states["/object_detector"] == State.PRIMARY_STATE_ACTIVE
    assert states["/count_objects"] == State.PRIMARY_STATE_ACTIVE
    assert manager.cleanup_skill("COUNT_OBJECTS", 1.0)[0]
    assert all(state == State.PRIMARY_STATE_UNCONFIGURED for state in states.values())


def test_find_object_configures_all_before_activating_and_cleans_in_reverse() -> None:
    """Special function: Enforce two-phase startup and reverse dependency teardown."""
    manager, states = make_manager()
    assert manager.prepare_skill("FIND_OBJECT", 1.0)[0]
    assert manager.transition_events == [
        ("/object_detector", Transition.TRANSITION_CONFIGURE),
        ("/find_object", Transition.TRANSITION_CONFIGURE),
        ("/object_detector", Transition.TRANSITION_ACTIVATE),
        ("/find_object", Transition.TRANSITION_ACTIVATE),
    ]
    assert all(
        states[node_name] == State.PRIMARY_STATE_ACTIVE
        for node_name in ("/object_detector", "/find_object")
    )
    manager.transition_events.clear()
    assert manager.cleanup_skill("FIND_OBJECT", 1.0)[0]
    assert manager.transition_events == [
        ("/find_object", Transition.TRANSITION_DEACTIVATE),
        ("/object_detector", Transition.TRANSITION_DEACTIVATE),
        ("/find_object", Transition.TRANSITION_CLEANUP),
        ("/object_detector", Transition.TRANSITION_CLEANUP),
    ]


def test_conflicting_skill_is_clean_before_next_activation() -> None:
    """Special function: COUNT_PEOPLE activation cleans the active object group first."""
    manager, states = make_manager()
    assert manager.prepare_skill("COUNT_OBJECTS", 1.0)[0]
    assert manager.prepare_skill("COUNT_PEOPLE", 1.0)[0]
    assert states["/object_detector"] == State.PRIMARY_STATE_UNCONFIGURED
    assert states["/count_objects"] == State.PRIMARY_STATE_UNCONFIGURED
    assert states["/human_detector"] == State.PRIMARY_STATE_ACTIVE
    assert states["/count_people"] == State.PRIMARY_STATE_ACTIVE


def test_shared_active_detector_is_cleaned_with_owned_conflict_before_reuse() -> None:
    """Special function: Shared ACTIVE nodes remain safe when ownership is explicit."""
    manager, states = make_manager()
    assert manager.prepare_skill("COUNT_OBJECTS", 1.0)[0]
    assert manager.prepare_skill("FIND_OBJECT", 1.0)[0]
    assert states["/count_objects"] == State.PRIMARY_STATE_UNCONFIGURED
    assert states["/object_detector"] == State.PRIMARY_STATE_ACTIVE
    assert states["/find_object"] == State.PRIMARY_STATE_ACTIVE


def test_repeated_skill_reconfigures_after_cleanup() -> None:
    """Special function: A cleaned COUNT_OBJECTS group can be prepared again."""
    manager, states = make_manager()
    assert manager.prepare_skill("COUNT_OBJECTS", 1.0)[0]
    assert manager.cleanup_skill("COUNT_OBJECTS", 1.0)[0]
    assert manager.prepare_skill("COUNT_OBJECTS", 1.0)[0]
    assert states["/object_detector"] == State.PRIMARY_STATE_ACTIVE


def test_activation_failure_rolls_back_to_safe_unconfigured_state() -> None:
    """Special function: Failed activation cannot leave a partial ACTIVE group."""
    manager, states = make_manager(
        fail_transition=("/count_objects", Transition.TRANSITION_ACTIVATE)
    )
    success, _message = manager.prepare_skill("COUNT_OBJECTS", 1.0)
    assert not success
    assert states["/object_detector"] == State.PRIMARY_STATE_UNCONFIGURED
    assert states["/count_objects"] == State.PRIMARY_STATE_UNCONFIGURED


def test_cleanup_failure_latches_integrity_fault_and_blocks_conflict() -> None:
    """Special function: Uncertain cleanup blocks the next vision skill."""
    manager, _states = make_manager(
        fail_transition=("/count_objects", Transition.TRANSITION_CLEANUP)
    )
    assert manager.prepare_skill("COUNT_OBJECTS", 1.0)[0]
    assert not manager.cleanup_skill("COUNT_OBJECTS", 1.0)[0]
    success, message = manager.prepare_skill("COUNT_PEOPLE", 1.0)
    assert not success
    assert "integrity fault" in message


def test_vision_integrity_fault_does_not_block_navigation() -> None:
    """Special function: A vision teardown fault cannot poison return navigation."""
    manager, _states = make_manager(
        fail_transition=("/count_objects", Transition.TRANSITION_CLEANUP)
    )
    assert manager.prepare_skill("COUNT_OBJECTS", 1.0)[0]
    assert not manager.cleanup_skill("COUNT_OBJECTS", 1.0)[0]
    assert manager.prepare_skill("NAVIGATE", 1.0)[0]


def test_navigation_integrity_fault_blocks_navigation() -> None:
    """Special function: Matching resource-domain faults remain fail-closed."""
    manager, _states = make_manager()
    manager._integrity_faults["navigation_motion"] = "FOLLOW_PERSON teardown uncertain"
    success, message = manager.prepare_skill("NAVIGATE", 1.0)
    assert not success
    assert "navigation_motion" in message


def test_missing_required_node_is_availability_failure_without_fault() -> None:
    """Special function: Requested preflight failure changes no state and latches no fault."""
    manager, states = make_manager()
    manager.available_nodes.remove("/find_object")
    success, message = manager.prepare_skill("FIND_OBJECT", 1.0)
    assert not success
    assert "/find_object/get_state" in message
    assert manager._integrity_faults == {}
    assert states["/object_detector"] == State.PRIMARY_STATE_UNCONFIGURED


def test_absent_optional_vision_peers_do_not_block_find_object() -> None:
    """Special function: Regress unlaunched conflict peers during FIND_OBJECT preparation."""
    manager, states = make_manager()
    manager.available_nodes = {"/object_detector", "/find_object"}
    success, message = manager.prepare_skill("FIND_OBJECT", 1.0)
    assert success, message
    assert states["/object_detector"] == State.PRIMARY_STATE_ACTIVE
    assert states["/find_object"] == State.PRIMARY_STATE_ACTIVE


def test_partially_active_conflict_peer_rejects_find_object() -> None:
    """Special function: Partial non-clean peer ownership blocks conflicting activation."""
    manager, states = make_manager()
    manager.available_nodes = {"/object_detector", "/find_object", "/human_detector"}
    states["/human_detector"] = State.PRIMARY_STATE_ACTIVE
    success, message = manager.prepare_skill("FIND_OBJECT", 1.0)
    assert not success
    assert "partial conflict peer COUNT_PEOPLE" in message
    assert "vision_gpu" in manager._integrity_faults


def test_recovery_ignores_absent_optional_peers_and_prepares_requested_group() -> None:
    """Special function: One recovery restores and prepares only present lifecycle resources."""
    manager, states = make_manager()
    manager.available_nodes = {"/object_detector", "/find_object"}
    manager._integrity_faults["vision_gpu"] = "prior teardown uncertain"
    manager.request_timeout = lambda request: float(request.timeout_sec)
    request = SimpleNamespace(skill_name="FIND_OBJECT", timeout_sec=1.0)
    response = SimpleNamespace(success=False, message="", state_json="")

    result = manager.recover_callback(request, response)

    assert result.success
    assert manager._integrity_faults == {}
    assert states["/object_detector"] == State.PRIMARY_STATE_ACTIVE
    assert states["/find_object"] == State.PRIMARY_STATE_ACTIVE


def test_find_object_cleanup_allows_next_conflicting_vision_skill() -> None:
    """Special function: FIND_OBJECT releases detector and server before COUNT_PEOPLE."""
    manager, states = make_manager()
    assert manager.prepare_skill("FIND_OBJECT", 1.0)[0]
    assert states["/object_detector"] == State.PRIMARY_STATE_ACTIVE
    assert states["/find_object"] == State.PRIMARY_STATE_ACTIVE
    assert manager.cleanup_skill("FIND_OBJECT", 1.0)[0]
    assert states["/object_detector"] == State.PRIMARY_STATE_UNCONFIGURED
    assert states["/find_object"] == State.PRIMARY_STATE_UNCONFIGURED
    assert manager.prepare_skill("COUNT_PEOPLE", 1.0)[0]
    assert states["/human_detector"] == State.PRIMARY_STATE_ACTIVE


def test_prepare_uses_one_absolute_group_deadline() -> None:
    """Special function: Multiple transitions cannot each consume the full request timeout."""
    manager, states = make_manager()
    original_change_state = manager.change_state

    def slow_change_state(node_name: str, transition_id: int, timeout_sec: float):
        """Special function: Consume the tiny deadline during the first configuration."""
        # Section: Exceed the manager's minimum bounded deadline to prove it is shared.
        time.sleep(0.12)
        return original_change_state(node_name, transition_id, timeout_sec)

    manager.change_state = slow_change_state
    started_at = time.monotonic()
    success, message = manager.prepare_skill("FIND_OBJECT", 0.01)

    assert not success
    assert "deadline expired" in message
    assert time.monotonic() - started_at < 0.5
    assert states["/object_detector"] == State.PRIMARY_STATE_UNCONFIGURED
    assert states["/find_object"] == State.PRIMARY_STATE_UNCONFIGURED
