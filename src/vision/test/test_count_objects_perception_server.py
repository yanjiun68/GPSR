"""Section: Qwen-backed COUNT_OBJECTS perception service regression tests."""

from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path

import pytest
import rclpy

from putrabot_interfaces.msg import ObjectDetection, ObjectDetections
from robot_ai_interfaces.srv import CountObjects


def load_count_objects_module():
    """Special function: Load the count-object script as an isolated testable Python module."""
    # Section: Source loading avoids requiring an installed vision executable during unit tests.
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "count_objects_perception_server.py"
    spec = importlib.util.spec_from_file_location("count_objects_perception_server", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def count_objects_node():
    """Special function: Create and destroy one active Qwen-count perception node per test."""
    # Section: Keep ROS logs isolated so repeated direct lifecycle tests do not collide.
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/roslogs")
    Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    if not rclpy.ok():
        rclpy.init()
    module = load_count_objects_module()
    node = module.CountObjectsPerceptionServer()
    node.trigger_configure()
    node.trigger_activate()
    try:
        yield node
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def make_detection(
    name: str,
    bbox: tuple[int, int, int, int],
    *,
    confidence: float = 0.90,
    attributes: dict[str, str] | None = None,
) -> ObjectDetection:
    """Special function: Build one Qwen-compatible synthetic object ROI message."""
    # Section: Populate the stable legacy fields first so the test also guards message compatibility.
    detection = ObjectDetection()
    detection.name = name
    detection.class_id = 0
    detection.confidence = confidence
    detection.x1, detection.y1, detection.x2, detection.y2 = bbox
    detection.center_u = float((bbox[0] + bbox[2]) / 2.0)
    detection.center_v = float((bbox[1] + bbox[3]) / 2.0)
    # Section: New Qwen metadata is optional until the generated interface is rebuilt in a test environment.
    if hasattr(detection, "attributes_json"):
        detection.attributes_json = json.dumps(attributes or {}, sort_keys=True)
    if hasattr(detection, "image_width"):
        detection.image_width = 640
    if hasattr(detection, "image_height"):
        detection.image_height = 480
    return detection


def publish_frames(node, detections: list[ObjectDetection], frame_count: int = 3) -> None:
    """Special function: Feed enough stable Qwen frames to satisfy temporal count confirmation."""
    # Section: ObjectDetections remains the shared boundary between Qwen and COUNT_OBJECTS.
    for _ in range(frame_count):
        message = ObjectDetections()
        message.stamp = node.get_clock().now().to_msg()
        message.detections = detections
        node.detections_callback(message)


def call_count(
    node,
    category: str,
    filters: list[dict[str, str]] | None = None,
) -> CountObjects.Response:
    """Special function: Invoke COUNT_OBJECTS without a ROS service round trip."""
    # Section: Service JSON mirrors the scheduler/C++ client transport envelope.
    request = CountObjects.Request()
    request.category = category
    request.location = ""
    request.filter_json = json.dumps({"filters": list(filters or [])}, sort_keys=True)
    return node.handle_count_objects(request, CountObjects.Response())


def test_count_exact_object_matches_safe_qwen_container_suffix(count_objects_node) -> None:
    """Special function: Count a configured coke target even when Qwen reports coke_can."""
    # Section: Qwen names may include a visible container word while object knowledge stores coke.
    publish_frames(count_objects_node, [make_detection("coke_can", (40, 80, 120, 300))])

    response = call_count(count_objects_node, "coke")
    result = json.loads(response.result_json)

    assert response.success
    assert response.count == 1
    assert result["matched_objects"][0]["name"] == "coke_can"
    assert result["qwen_query"]["target"] == {"object_name": "coke"}


def test_count_uses_qwen_color_evidence_as_a_local_filter(count_objects_node) -> None:
    """Special function: Count only requested color ROIs rather than every same-name label."""
    # Section: Attribute transport requires the regenerated ObjectDetection interface.
    red_cup = make_detection("cup", (20, 100, 100, 260), attributes={"color": "red"})
    blue_cup = make_detection("cup", (260, 100, 340, 260), attributes={"color": "blue"})
    if not hasattr(red_cup, "attributes_json"):
        pytest.skip("ObjectDetection attributes_json is unavailable until interfaces are rebuilt.")
    publish_frames(count_objects_node, [red_cup, blue_cup])

    response = call_count(
        count_objects_node,
        "cup",
        [{"feature": "object_color", "operator": "equals", "value": "red"}],
    )
    result = json.loads(response.result_json)

    assert response.success
    assert response.count == 1
    assert result["matched_objects"][0]["attributes"]["color"] == "red"
    assert result["filters"] == [
        {"feature": "object_color", "operator": "equals", "value": "red"},
    ]


def test_count_uses_image_relation_with_a_confirmed_reference(count_objects_node) -> None:
    """Special function: Count only bottles locally verified to the left of a cup ROI."""
    # Section: Relation evaluation is deterministic from centers and does not ask Qwen for an action decision.
    left_bottle = make_detection("bottle", (30, 100, 100, 280))
    cup = make_detection("cup", (220, 100, 300, 280))
    right_bottle = make_detection("bottle", (420, 100, 490, 280))
    publish_frames(count_objects_node, [left_bottle, cup, right_bottle])

    response = call_count(
        count_objects_node,
        "bottle",
        [
            {"feature": "spatial_relation", "operator": "equals", "value": "left_of"},
            {"feature": "reference_object", "operator": "equals", "value": "cup"},
        ],
    )

    assert response.success
    assert response.count == 1
    assert json.loads(response.result_json)["qwen_query_dispatched"] is True
