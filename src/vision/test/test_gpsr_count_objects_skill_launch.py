"""Section: Production COUNT_OBJECTS launch ownership tests."""

import importlib.util
from pathlib import Path

from launch_ros.actions import Node


def test_production_launch_contains_only_count_objects_owned_nodes() -> None:
    """Special function: Reject persistent infrastructure from the ephemeral launch."""
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "gpsr_count_objects_skill.launch.py"
    module_spec = importlib.util.spec_from_file_location("gpsr_count_objects_skill_launch", launch_path)
    launch_module = importlib.util.module_from_spec(module_spec)
    assert module_spec.loader is not None
    module_spec.loader.exec_module(launch_module)
    nodes = [entity for entity in launch_module.generate_launch_description().entities if isinstance(entity, Node)]
    actual = {(node.node_package, node.node_executable) for node in nodes}
    assert actual == {
        ("vision", "gpsr_object_detection_node.py"),
        ("vision", "count_objects_perception_server.py"),
        ("robot_yasmin_tasks", "count_objects_skill_server"),
    }
    assert len(nodes) == 3


def test_manual_debug_launch_is_preserved() -> None:
    """Special function: Keep the existing manual test and TTS launch available."""
    debug_launch = Path(__file__).resolve().parents[1] / "launch" / "gpsr_count_objects_test.launch.py"
    assert debug_launch.is_file()
