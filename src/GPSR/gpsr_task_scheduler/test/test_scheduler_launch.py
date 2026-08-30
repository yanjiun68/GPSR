"""Section: Integrated scheduler launch pipeline-selection tests."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode


def load_scheduler_launch(tmp_path: Path) -> ModuleType:
    """Special function: Load the scheduler launch module without requiring installation."""
    # Section: Keep ROS launch logging inside the writable test directory.
    os.environ["ROS_LOG_DIR"] = str(tmp_path / "ros_logs")
    # Section: Resolve the source launch file relative to this package test.
    launch_path = Path(__file__).parents[1] / "launch" / "gpsr_scheduler.launch.py"
    # Section: Import the launch file through its filesystem path.
    spec = importlib.util.spec_from_file_location("gpsr_scheduler_launch", launch_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_entities(tmp_path: Path) -> tuple[list[object], LaunchContext]:
    """Special function: Build launch entities and apply their declared defaults."""
    # Section: Generate the complete launch description under test.
    module = load_scheduler_launch(tmp_path)
    entities = list(module.generate_launch_description().entities)
    # Section: Execute declarations so conditions can resolve default configurations.
    context = LaunchContext()
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return entities, context


def lifecycle_node(entities: list[object], node_name: str) -> LifecycleNode:
    """Special function: Select one pre-execution lifecycle action by configured node name."""
    # Section: launch_ros exposes the configured name only after execution, so inspect its stored substitution.
    matches = [
        entity
        for entity in entities
        if isinstance(entity, LifecycleNode)
        and str(getattr(entity, "_Node__node_name", "")) == node_name
    ]
    assert len(matches) == 1
    return matches[0]


def test_all_selectable_perception_pipelines_are_enabled_by_default(tmp_path: Path) -> None:
    """Special function: Make the single scheduler launch ready for every perception command."""
    # Section: Resolve launch defaults and verify every selectable transient server is present.
    entities, context = launch_entities(tmp_path)
    for node_name in (
        "find_object_skill_server",
        "count_objects_skill_server",
        "count_people_skill_server",
        "gpsr_human_identity_node",
        "gpsr_tracked_appearance_node",
        "find_person_by_attribute_skill_server",
        "extract_person_info_skill_server",
    ):
        node = lifecycle_node(entities, node_name)
        assert node.condition is not None and node.condition.evaluate(context)


def test_find_object_is_independent_from_count_objects_switch(tmp_path: Path) -> None:
    """Special function: Keep FIND_OBJECT available when object counting is disabled."""
    # Section: Enable FIND_OBJECT alone and disable its sibling COUNT_OBJECTS pipeline.
    entities, context = launch_entities(tmp_path)
    context.launch_configurations["start_find_object_pipeline"] = "true"
    context.launch_configurations["start_count_objects_pipeline"] = "false"
    # Section: The compound server and shared detector must remain available.
    find_server = lifecycle_node(entities, "find_object_skill_server")
    object_detector = lifecycle_node(entities, "gpsr_object_detection_node")
    count_server = lifecycle_node(entities, "count_objects_skill_server")
    assert find_server.condition is not None and find_server.condition.evaluate(context)
    assert object_detector.condition is not None and object_detector.condition.evaluate(context)
    assert count_server.condition is not None and not count_server.condition.evaluate(context)


def test_person_attribute_pipeline_switch_is_independent(tmp_path: Path) -> None:
    """Special function: Attribute lifecycle resources follow only their own launch switch."""
    # Section: Disable the persistent identity include while retaining pose-attribute search.
    entities, context = launch_entities(tmp_path)
    context.launch_configurations["start_find_person_pipeline"] = "false"
    context.launch_configurations["start_find_person_attribute_pipeline"] = "true"
    attribute = lifecycle_node(entities, "find_person_by_attribute_skill_server")
    assert attribute.condition is not None and attribute.condition.evaluate(context)

    # Section: Reverse the switches without coupling the attribute lifecycle server.
    context.launch_configurations["start_find_person_pipeline"] = "true"
    context.launch_configurations["start_find_person_attribute_pipeline"] = "false"
    assert not attribute.condition.evaluate(context)


def test_attribute_and_count_people_share_detector_processes(tmp_path: Path) -> None:
    """Special function: Either consumer starts shared human and pose inference exactly once."""
    # Section: Attribute search alone keeps both shared lifecycle nodes available.
    entities, context = launch_entities(tmp_path)
    context.launch_configurations["start_count_people_pipeline"] = "false"
    context.launch_configurations["start_find_person_attribute_pipeline"] = "true"
    human_detector = lifecycle_node(entities, "gpsr_human_detection_node")
    pose_detector = lifecycle_node(entities, "human_skeleton_detection_node")
    count_server = lifecycle_node(entities, "count_people_skill_server")
    assert human_detector.condition is not None and human_detector.condition.evaluate(context)
    assert pose_detector.condition is not None and pose_detector.condition.evaluate(context)
    assert count_server.condition is not None and not count_server.condition.evaluate(context)

    # Section: Both shared nodes stop only when neither consumer is enabled.
    context.launch_configurations["start_find_person_attribute_pipeline"] = "false"
    assert not human_detector.condition.evaluate(context)
    assert not pose_detector.condition.evaluate(context)


def test_count_objects_is_independent_from_find_object_switch(tmp_path: Path) -> None:
    """Special function: Keep COUNT_OBJECTS available when FIND_OBJECT is disabled."""
    # Section: Enable COUNT_OBJECTS alone and disable its sibling FIND_OBJECT pipeline.
    entities, context = launch_entities(tmp_path)
    context.launch_configurations["start_find_object_pipeline"] = "false"
    context.launch_configurations["start_count_objects_pipeline"] = "true"
    # Section: The count server and shared detector must remain available exactly once.
    find_server = lifecycle_node(entities, "find_object_skill_server")
    object_detector = lifecycle_node(entities, "gpsr_object_detection_node")
    count_server = lifecycle_node(entities, "count_objects_skill_server")
    assert find_server.condition is not None and not find_server.condition.evaluate(context)
    assert object_detector.condition is not None and object_detector.condition.evaluate(context)
    assert count_server.condition is not None and count_server.condition.evaluate(context)


def test_shared_object_detector_stops_only_when_both_consumers_are_disabled(tmp_path: Path) -> None:
    """Special function: Avoid an unused detector process when both object skills are disabled."""
    # Section: Disable both object-skill pipelines.
    entities, context = launch_entities(tmp_path)
    context.launch_configurations["start_find_object_pipeline"] = "false"
    context.launch_configurations["start_count_objects_pipeline"] = "false"
    # Section: The shared detector condition must now evaluate false.
    object_detector = lifecycle_node(entities, "gpsr_object_detection_node")
    assert object_detector.condition is not None
    assert not object_detector.condition.evaluate(context)


def test_command_start_amcl_return_defaults_disable_startup_motion(tmp_path: Path) -> None:
    """Special function: Verify safe default return and navigation action parameters."""
    _entities, context = launch_entities(tmp_path)
    assert context.launch_configurations["return_target_mode"] == "command_start_amcl"
    assert context.launch_configurations["navigate_to_instruction_point_on_start"] == "false"
    assert context.launch_configurations["amcl_pose_topic"] == "/amcl_pose"
    assert context.launch_configurations["amcl_pose_max_age_sec"] == "1.0"
    assert context.launch_configurations["amcl_pose_capture_timeout_sec"] == "3.0"
    assert context.launch_configurations["return_fallback_named_location"] == "instruction_point"
    assert context.launch_configurations["navigate_to_pose_action"] == "/navigate_to_pose"
    assert context.launch_configurations["navigate_to_named_pose_action"] == "/nav/navigate_named_pose"


def test_depth_localization_is_selectable_and_enabled_by_default(tmp_path: Path) -> None:
    """Special function: Preserve 3D FIND_OBJECT behavior while permitting RGB-only launch."""
    # Section: The launch-level switch must be explicit and retain the existing depth default.
    _entities, context = launch_entities(tmp_path)
    assert context.launch_configurations["enable_depth"] == "true"


def test_person_search_policy_has_an_installed_yaml_default(tmp_path: Path) -> None:
    """Special function: Expose one shared policy override for both person find routes."""
    _entities, context = launch_entities(tmp_path)
    configured = context.launch_configurations["find_person_search_config_file"]
    assert str(configured).endswith("/config/find_person_search.yaml")


def test_lifecycle_deadline_defaults_allow_heavy_gpu_configuration(tmp_path: Path) -> None:
    """Special function: Executor response wait must exceed the manager's absolute deadline."""
    _entities, context = launch_entities(tmp_path)
    manager_timeout = float(context.launch_configurations["lifecycle_transition_timeout_sec"])
    client_margin = float(context.launch_configurations["lifecycle_operation_timeout_margin_sec"])
    assert manager_timeout == 60.0
    assert client_margin == 10.0
    assert manager_timeout + client_margin > manager_timeout
