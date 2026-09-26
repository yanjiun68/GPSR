"""Section: Static launch-default tests that never start Nav2 or physical drivers."""

# Section: Import launch files by path without executing their launch descriptions.
import importlib.util
from pathlib import Path

import pytest

WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
NAV_LAUNCH_DIRECTORY = WORKSPACE_ROOT / "src" / "NAVIGATION" / "my_robot_nav" / "launch"


def load_launch_module(filename: str):
    """Special function: Load one Python launch module for pure helper inspection."""
    # Section: Execute definitions only; generate_launch_description is not called here.
    launch_path = NAV_LAUNCH_DIRECTORY / filename
    module_spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), launch_path)
    assert module_spec is not None and module_spec.loader is not None
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "launch_filename",
    ["lidar_loc_nav.launch.py", "static_map_nav.launch.py"],
)
def test_navigation_defaults_use_workspace_maps_and_waypoints(launch_filename: str) -> None:
    """Special function: Resolve both navigation defaults under the workspace root."""
    # Section: Required static map and waypoint path construction test.
    module = load_launch_module(launch_filename)
    package_share = "/work/test_ws/install/my_robot_nav/share/my_robot_nav"
    map_path, waypoint_path = module.resolve_runtime_data_paths(package_share, {})
    assert map_path == Path("/work/test_ws/maps/gpsr_map.yaml")
    assert waypoint_path == Path("/work/test_ws/waypoints/gpsr_waypoint.yaml")


@pytest.mark.parametrize(
    "launch_filename",
    ["lidar_loc_nav.launch.py", "static_map_nav.launch.py"],
)
def test_navigation_workspace_override_has_priority(launch_filename: str) -> None:
    """Special function: Apply PUTRABOT_WORKSPACE_ROOT before install derivation."""
    # Section: Required environment override path construction test.
    module = load_launch_module(launch_filename)
    map_path, waypoint_path = module.resolve_runtime_data_paths(
        "/ignored/nonstandard/share",
        {"PUTRABOT_WORKSPACE_ROOT": "/custom/ws"},
    )
    assert map_path == Path("/custom/ws/maps/gpsr_map.yaml")
    assert waypoint_path == Path("/custom/ws/waypoints/gpsr_waypoint.yaml")


def test_navigation_workspace_resolution_failure_is_explicit() -> None:
    """Special function: Reject non-colcon layouts without an environment override."""
    # Section: Required resolver failure behavior test.
    module = load_launch_module("lidar_loc_nav.launch.py")
    with pytest.raises(ValueError, match="PUTRABOT_WORKSPACE_ROOT"):
        module.resolve_workspace_root("/not/a/colcon/share/path", {})


def test_navigation_launch_statically_integrates_waypoint_saver() -> None:
    """Special function: Verify lidar owns the node and static-map launch forwards controls."""
    # Section: Read source only; no launch action or hardware process is started.
    lidar_source = (NAV_LAUNCH_DIRECTORY / "lidar_loc_nav.launch.py").read_text(encoding="utf-8")
    static_source = (NAV_LAUNCH_DIRECTORY / "static_map_nav.launch.py").read_text(encoding="utf-8")
    assert "package='waypoint_saver'" in lidar_source
    assert "'max_transform_age_sec': waypoint_saver_max_transform_age_sec" in lidar_source
    assert "'start_waypoint_saver': start_waypoint_saver" in static_source
