"""Section: Hardware-free tests for TF age, overwrite policy, paths, and atomic YAML."""

# Section: Import source helpers directly so tests do not require a running ROS graph.
from copy import deepcopy
from pathlib import Path
import sys

import pytest
from yaml import safe_load

SCRIPT_DIRECTORY = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIRECTORY))

from waypoint_saver_logic import (  # noqa: E402
    WaypointValidationError,
    atomic_write_waypoint_data,
    load_waypoint_data,
    resolve_workspace_root,
    resolve_yaml_path,
    update_waypoint_data,
    validate_transform_age,
)


def build_entry(waypoint_id: int, name: str) -> dict:
    """Special function: Build a compact valid waypoint for overwrite-policy tests."""
    # Section: Use a recognizable pose value so replacement assertions detect updates.
    return {
        "id": waypoint_id,
        "frame_id": "map",
        "name": name,
        "pose": {"position": {"x": 99.0}, "orientation": {"w": 1.0}},
    }


def existing_entry(waypoint_id: int, name: str, x_value: float = 1.0) -> dict:
    """Special function: Build one existing waypoint with a distinguishable old pose."""
    # Section: Keep identity and pose fields compatible with named-navigation YAML.
    return {
        "id": waypoint_id,
        "frame_id": "map",
        "name": name,
        "pose": {"position": {"x": x_value}, "orientation": {"w": 1.0}},
    }


def test_fresh_transform_is_accepted() -> None:
    """Special function: Accept a transform younger than the configured maximum age."""
    # Section: Required fabricated timestamp case: age 0.5s <= 1.0s.
    assert validate_transform_age(100.0, 99.5, 1.0) == pytest.approx(0.5)


def test_transform_at_exact_limit_is_accepted() -> None:
    """Special function: Define the exact freshness boundary as inclusive."""
    # Section: Required deterministic policy: age equal to max age passes.
    assert validate_transform_age(100.0, 99.0, 1.0) == pytest.approx(1.0)


def test_stale_transform_is_rejected() -> None:
    """Special function: Reject a transform older than the configured threshold."""
    # Section: Required fabricated timestamp case: age 3.0s > 1.0s.
    with pytest.raises(WaypointValidationError, match="stale"):
        validate_transform_age(100.0, 97.0, 1.0)


def test_future_transform_is_rejected() -> None:
    """Special function: Reject an obviously future-dated transform deterministically."""
    # Section: Required fabricated timestamp case: timestamp is 2.0s in the future.
    with pytest.raises(WaypointValidationError, match="future"):
        validate_transform_age(100.0, 102.0, 1.0)


def test_zero_transform_timestamp_is_rejected() -> None:
    """Special function: Reject zero because map-to-base TF is expected to be dynamic and stamped."""
    # Section: A zero/uninitialized dynamic TF cannot prove pose freshness.
    with pytest.raises(WaypointValidationError, match="zero or uninitialized"):
        validate_transform_age(100.0, 0.0, 1.0)


def test_new_automatic_id_uses_maximum_plus_one() -> None:
    """Special function: Allocate ID 5 when existing IDs are 0 and 4."""
    # Section: Required overwrite test 1.
    data = {"waypoints": [existing_entry(0, "A"), existing_entry(4, "B")]}
    updated, assigned_id = update_waypoint_data(data, "C", -1, False, build_entry)
    assert assigned_id == 5
    assert updated["waypoints"][-1]["id"] == 5


def test_existing_name_automatic_id_without_overwrite_is_rejected() -> None:
    """Special function: Reject automatic-ID reuse of an existing name without consent."""
    # Section: Required overwrite test 2 verifies the caller document is unchanged.
    data = {"waypoints": [existing_entry(5, "KITCHEN")]}
    original = deepcopy(data)
    with pytest.raises(WaypointValidationError, match="already exists"):
        update_waypoint_data(data, "KITCHEN", -1, False, build_entry)
    assert data == original


def test_existing_name_automatic_id_overwrite_preserves_id() -> None:
    """Special function: Replace one matching name while preserving its existing ID."""
    # Section: Required overwrite test 3 verifies ID 5 and exactly one KITCHEN remain.
    data = {"waypoints": [existing_entry(5, "KITCHEN")]}
    updated, assigned_id = update_waypoint_data(data, "KITCHEN", -1, True, build_entry)
    matches = [entry for entry in updated["waypoints"] if entry["name"] == "KITCHEN"]
    assert assigned_id == 5
    assert len(matches) == 1
    assert matches[0]["id"] == 5
    assert matches[0]["pose"]["position"]["x"] == 99.0


def test_explicit_matching_id_and_name_replaces_exactly_one() -> None:
    """Special function: Replace the single entry selected by the same explicit ID and name."""
    # Section: Required overwrite test 4.
    data = {"waypoints": [existing_entry(5, "KITCHEN")]}
    updated, assigned_id = update_waypoint_data(data, "KITCHEN", 5, True, build_entry)
    assert assigned_id == 5
    assert len(updated["waypoints"]) == 1
    assert updated["waypoints"][0]["pose"]["position"]["x"] == 99.0


def test_ambiguous_explicit_overwrite_is_rejected_unchanged() -> None:
    """Special function: Reject when requested name and ID select two different entries."""
    # Section: Required overwrite test 5 protects both original records.
    data = {
        "waypoints": [
            existing_entry(5, "KITCHEN"),
            existing_entry(6, "LIVING_ROOM"),
        ]
    }
    original = deepcopy(data)
    with pytest.raises(WaypointValidationError, match="Ambiguous overwrite"):
        update_waypoint_data(data, "KITCHEN", 6, True, build_entry)
    assert data == original


def test_existing_name_can_move_to_unused_explicit_id() -> None:
    """Special function: Replace an existing name with requested unused ID 8."""
    # Section: Required overwrite test 6 leaves exactly one KITCHEN entry.
    data = {"waypoints": [existing_entry(5, "KITCHEN")]}
    updated, assigned_id = update_waypoint_data(data, "KITCHEN", 8, True, build_entry)
    assert assigned_id == 8
    assert [(entry["id"], entry["name"]) for entry in updated["waypoints"]] == [
        (8, "KITCHEN")
    ]


def test_existing_id_can_receive_unused_new_name() -> None:
    """Special function: Replace ID 5 with a new unused name."""
    # Section: Required overwrite test 7.
    data = {"waypoints": [existing_entry(5, "KITCHEN")]}
    updated, assigned_id = update_waypoint_data(data, "DINING_ROOM", 5, True, build_entry)
    assert assigned_id == 5
    assert [(entry["id"], entry["name"]) for entry in updated["waypoints"]] == [
        (5, "DINING_ROOM")
    ]


def test_duplicate_existing_names_reject_deterministic_overwrite() -> None:
    """Special function: Reject a corrupted file containing duplicate names."""
    # Section: Required overwrite test 8 prevents silent multi-entry deletion.
    data = {
        "waypoints": [
            existing_entry(5, "KITCHEN"),
            existing_entry(6, "KITCHEN"),
        ]
    }
    with pytest.raises(WaypointValidationError, match="duplicate names"):
        update_waypoint_data(data, "KITCHEN", -1, True, build_entry)


def test_duplicate_existing_ids_reject_deterministic_overwrite() -> None:
    """Special function: Reject a corrupted file containing duplicate IDs."""
    # Section: Required overwrite test 9 prevents ambiguous ID replacement.
    data = {
        "waypoints": [
            existing_entry(5, "KITCHEN"),
            existing_entry(5, "LIVING_ROOM"),
        ]
    }
    with pytest.raises(WaypointValidationError, match="duplicate ids"):
        update_waypoint_data(data, "DINING_ROOM", 5, True, build_entry)


def test_missing_waypoint_directory_and_yaml_are_created(tmp_path: Path) -> None:
    """Special function: Create a missing waypoints directory and parseable initial YAML."""
    # Section: Required filesystem creation cases use only pytest temporary storage.
    yaml_path = tmp_path / "workspace" / "waypoints" / "gpsr_waypoint.yaml"
    data = {"waypoints": [build_entry(0, "A")]}
    atomic_write_waypoint_data(yaml_path, data)
    assert yaml_path.is_file()
    assert safe_load(yaml_path.read_text(encoding="utf-8")) == data


def test_missing_waypoint_yaml_loads_empty_schema(tmp_path: Path) -> None:
    """Special function: Initialize a missing waypoint YAML as an empty waypoint list."""
    # Section: A save request does not require a pre-existing file.
    assert load_waypoint_data(tmp_path / "missing.yaml") == {"waypoints": []}


def test_relative_yaml_path_is_rejected() -> None:
    """Special function: Preserve rejection of relative request paths."""
    # Section: Required invalid-relative-path case.
    with pytest.raises(WaypointValidationError, match="absolute"):
        resolve_yaml_path("waypoints/gpsr_waypoint.yaml")


def test_non_yaml_extension_is_rejected(tmp_path: Path) -> None:
    """Special function: Reject a caller-selected non-YAML extension."""
    # Section: Required invalid-extension case.
    with pytest.raises(WaypointValidationError, match="yaml or .yml"):
        resolve_yaml_path(str(tmp_path / "gpsr_waypoint.txt"))


def test_atomic_replace_failure_preserves_original_yaml(tmp_path: Path) -> None:
    """Special function: Keep the original parseable YAML when replacement fails."""
    # Section: Seed a real temporary file and simulate failure at the atomic replacement boundary.
    yaml_path = tmp_path / "waypoints.yaml"
    original = {"waypoints": [existing_entry(1, "ORIGINAL")]}
    atomic_write_waypoint_data(yaml_path, original)

    # Special function: Simulate an operating-system failure before destination replacement.
    def reject_replace(_source: str | Path, _destination: str | Path) -> None:
        raise OSError("simulated replace failure")

    with pytest.raises(OSError, match="simulated replace failure"):
        atomic_write_waypoint_data(
            yaml_path,
            {"waypoints": [build_entry(2, "NEW")]},
            replace_function=reject_replace,
        )
    assert safe_load(yaml_path.read_text(encoding="utf-8")) == original
    assert list(tmp_path.glob(".*.tmp")) == []


def test_workspace_root_derives_from_colcon_package_share() -> None:
    """Special function: Derive /work/test_ws from a standard installed package share."""
    # Section: Required workspace-root derivation case.
    package_share = "/work/test_ws/install/my_robot_nav/share/my_robot_nav"
    assert resolve_workspace_root(package_share, {}) == Path("/work/test_ws")


def test_workspace_environment_override_has_priority() -> None:
    """Special function: Prefer PUTRABOT_WORKSPACE_ROOT over install-space derivation."""
    # Section: Required environment precedence case.
    assert resolve_workspace_root(
        "/work/test_ws/install/my_robot_nav/share/my_robot_nav",
        {"PUTRABOT_WORKSPACE_ROOT": "/custom/ws"},
    ) == Path("/custom/ws")


def test_workspace_root_failure_requests_environment_override() -> None:
    """Special function: Fail clearly when neither override nor colcon layout can resolve."""
    # Section: Required resolver failure behavior.
    with pytest.raises(WaypointValidationError, match="PUTRABOT_WORKSPACE_ROOT"):
        resolve_workspace_root("/opt/random/share/my_robot_nav", {})
