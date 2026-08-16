"""Section: Central GPSR lifecycle-manager transition policy tests."""

from pathlib import Path

import pytest
from lifecycle_msgs.msg import State, Transition

from robot_yasmin_tasks.gpsr.action_types import ALLOWED_ACTION_TYPES
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


def test_person_routes_use_real_disjoint_lifecycle_servers() -> None:
    """Special function: Name and attribute actions cannot prepare through the other server."""
    # Section: Load production policy rather than duplicating its node lists in the test.
    manager = object.__new__(GpsrLifecycleManager)
    registry_path = Path(__file__).parents[1] / "config" / "skill_registry.yaml"
    registry = manager.load_registry(str(registry_path))
    identity_nodes = set(registry["FIND_PERSON"].lifecycle_nodes)
    attribute_nodes = set(registry["FIND_PERSON_BY_ATTRIBUTE"].lifecycle_nodes)
    assert "/find_person_identity_resource" in identity_nodes
    assert "/find_person_by_attribute_skill_server" in attribute_nodes
    assert "/gpsr_tracked_appearance_node" in attribute_nodes
    assert identity_nodes.isdisjoint(attribute_nodes)


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
    for primitive in {"PICK_OBJECT", "PLACE_OBJECT", "HANDOVER_OBJECT", "FOLLOW_PERSON", "GUIDE_PERSON"}:
        assert registry[primitive].implementation == "missing"
        assert registry[primitive].production_enabled is False


class FakeLogger:
    """Special function: Accept manager transition logs in pure unit tests."""

    def info(self, _message: str) -> None:
        """Special function: Ignore informational test logs."""
        return None

    def error(self, _message: str) -> None:
        """Special function: Ignore expected failure logs."""
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
        "SAY": SkillLifecycleSpec("SAY", "", (), "/voice/tts/say"),
    }
    states = {
        node_name: State.PRIMARY_STATE_UNCONFIGURED
        for spec in manager.registry.values()
        for node_name in spec.lifecycle_nodes
    }
    manager._integrity_fault = ""
    manager._prepared_skill = ""
    manager.get_logger = lambda: FakeLogger()
    manager.log_transition = lambda _skill, _node, _operation: None

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


def test_conflicting_skill_is_clean_before_next_activation() -> None:
    """Special function: COUNT_PEOPLE activation cleans the active object group first."""
    manager, states = make_manager()
    assert manager.prepare_skill("COUNT_OBJECTS", 1.0)[0]
    assert manager.prepare_skill("COUNT_PEOPLE", 1.0)[0]
    assert states["/object_detector"] == State.PRIMARY_STATE_UNCONFIGURED
    assert states["/count_objects"] == State.PRIMARY_STATE_UNCONFIGURED
    assert states["/human_detector"] == State.PRIMARY_STATE_ACTIVE
    assert states["/count_people"] == State.PRIMARY_STATE_ACTIVE


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
