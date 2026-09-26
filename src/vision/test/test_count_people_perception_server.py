"""Section: Count people perception server tests."""

from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path

import pytest
import rclpy

from putrabot_interfaces.msg import HumanSkeleton, HumanSkeletonKeypoint, HumanSkeletons, ObjectDetection, ObjectDetections
from robot_ai_interfaces.srv import CountPeople


def load_count_people_module():
    """Special function: Load the vision script as a testable Python module."""
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "count_people_perception_server.py"
    spec = importlib.util.spec_from_file_location("count_people_perception_server", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def count_people_node():
    """Special function: Create and destroy a fresh count-people ROS node per test."""
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/roslogs")
    Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    if not rclpy.ok():
        rclpy.init()
    module = load_count_people_module()
    node = module.CountPeoplePerceptionServer()
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


def make_detection(name: str = "person", confidence: float = 0.90, bbox: tuple[int, int, int, int] = (10, 10, 60, 120)) -> ObjectDetection:
    """Special function: Build one synthetic object detection message."""
    det = ObjectDetection()
    det.name = name
    det.class_id = 0
    det.confidence = confidence
    det.x1 = bbox[0]
    det.y1 = bbox[1]
    det.x2 = bbox[2]
    det.y2 = bbox[3]
    det.center_u = float((bbox[0] + bbox[2]) / 2.0)
    det.center_v = float((bbox[1] + bbox[3]) / 2.0)
    return det


def publish_frames(node, frames: list[list[ObjectDetection]]) -> None:
    """Special function: Feed synthetic detection frames into the perception server."""
    for detections in frames:
        msg = ObjectDetections()
        msg.stamp = node.get_clock().now().to_msg()
        msg.detections = detections
        node.detections_callback(msg)


def make_keypoint(name: str, index: int, x: float, y: float, confidence: float = 0.90, visible: bool = True) -> HumanSkeletonKeypoint:
    """Special function: Build one synthetic human skeleton keypoint."""
    keypoint = HumanSkeletonKeypoint()
    keypoint.name = name
    keypoint.index = index
    keypoint.x = float(x)
    keypoint.y = float(y)
    keypoint.confidence = float(confidence)
    keypoint.visible = bool(visible)
    return keypoint


def make_skeleton(
    pose: str = "standing",
    confidence: float = 0.90,
    bbox: tuple[int, int, int, int] = (10, 10, 60, 140),
) -> HumanSkeleton:
    """Special function: Build one synthetic human skeleton message."""
    skeleton = HumanSkeleton()
    skeleton.skeleton_id = f"skeleton_{bbox[0]}_{bbox[1]}"
    skeleton.pose = pose
    skeleton.confidence = float(confidence)
    skeleton.x1 = bbox[0]
    skeleton.y1 = bbox[1]
    skeleton.x2 = bbox[2]
    skeleton.y2 = bbox[3]
    skeleton.center_u = float((bbox[0] + bbox[2]) / 2.0)
    skeleton.center_v = float((bbox[1] + bbox[3]) / 2.0)
    skeleton.keypoints = [
        make_keypoint("left_shoulder", 5, bbox[0] + 10, bbox[1] + 20),
        make_keypoint("right_shoulder", 6, bbox[2] - 10, bbox[1] + 20),
        make_keypoint("left_hip", 11, bbox[0] + 14, bbox[1] + 70),
        make_keypoint("right_hip", 12, bbox[2] - 14, bbox[1] + 70),
        make_keypoint("left_knee", 13, bbox[0] + 16, bbox[1] + 100),
        make_keypoint("right_knee", 14, bbox[2] - 16, bbox[1] + 100),
    ]
    return skeleton


def publish_skeleton_frames(node, frames: list[list[HumanSkeleton]]) -> None:
    """Special function: Feed synthetic skeleton frames into the perception server."""
    for skeletons in frames:
        msg = HumanSkeletons()
        msg.stamp = node.get_clock().now().to_msg()
        msg.skeletons = skeletons
        node.skeletons_callback(msg)


def call_count_people(node) -> CountPeople.Response:
    """Special function: Call the count people handler without a ROS service round trip."""
    request = CountPeople.Request()
    response = CountPeople.Response()
    return node.handle_count_people(request, response)


def test_zero_people_returns_zero(count_people_node) -> None:
    """Special function: Empty confirmed frames count as zero visible people."""
    publish_frames(count_people_node, [[], [], []])
    response = call_count_people(count_people_node)
    assert response.success
    assert response.count == 0


def test_one_stable_person_returns_one(count_people_node) -> None:
    """Special function: One stable person detection is counted once."""
    person = make_detection()
    publish_frames(count_people_node, [[person], [person], [person]])
    response = call_count_people(count_people_node)
    assert response.success
    assert response.count == 1


def test_two_separated_people_return_two(count_people_node) -> None:
    """Special function: Two separated person boxes are counted as two humans."""
    left_person = make_detection(bbox=(10, 10, 60, 120))
    right_person = make_detection(bbox=(180, 20, 240, 130))
    publish_frames(count_people_node, [[left_person, right_person], [left_person, right_person], [left_person, right_person]])
    response = call_count_people(count_people_node)
    assert response.success
    assert response.count == 2


def test_low_confidence_person_is_ignored(count_people_node) -> None:
    """Special function: Low-confidence person detections do not pass confirmation."""
    person = make_detection(confidence=0.20)
    publish_frames(count_people_node, [[person], [person], [person]])
    response = call_count_people(count_people_node)
    assert response.success
    assert response.count == 0


def test_stale_scene_returns_failure(count_people_node) -> None:
    """Special function: Stale human detections fail instead of answering from old data."""
    person = make_detection()
    publish_frames(count_people_node, [[person], [person], [person]])
    count_people_node.last_scene_time_sec = count_people_node.now_sec() - 10.0
    response = call_count_people(count_people_node)
    assert not response.success
    assert response.count == 0


def test_skeleton_only_people_count_and_pose_quantities(count_people_node) -> None:
    """Special function: Fresh skeleton-only humans count and report pose quantities."""
    standing = make_skeleton("standing", bbox=(10, 10, 60, 140))
    sitting = make_skeleton("sitting", bbox=(160, 30, 220, 140))
    publish_skeleton_frames(count_people_node, [[standing, sitting], [standing, sitting], [standing, sitting]])
    response = call_count_people(count_people_node)
    result = json.loads(response.result_json)
    assert response.success
    assert response.count == 2
    assert result["pose_quantities"] == {"standing": 1, "sitting": 1}


def test_matching_box_and_skeleton_counts_once(count_people_node) -> None:
    """Special function: Matching box and skeleton detections merge into one counted human."""
    person = make_detection(bbox=(10, 10, 60, 140))
    skeleton = make_skeleton("standing", bbox=(12, 12, 62, 142))
    for _ in range(3):
        publish_frames(count_people_node, [[person]])
        publish_skeleton_frames(count_people_node, [[skeleton]])
    response = call_count_people(count_people_node)
    result = json.loads(response.result_json)
    assert response.success
    assert response.count == 1
    assert result["human_entities"][0]["source"] == "box+skeleton"
    assert result["human_entities"][0]["pose"] == "standing"


def test_mixed_pose_entities_are_sorted_by_pose_then_left_to_right(count_people_node) -> None:
    """Special function: Confirmed people are sorted by pose priority and image x position."""
    left_unknown = make_detection(bbox=(10, 10, 60, 140))
    right_standing = make_skeleton("standing", bbox=(180, 10, 240, 140))
    middle_sitting = make_skeleton("sitting", bbox=(90, 10, 150, 140))
    for _ in range(3):
        publish_frames(count_people_node, [[left_unknown]])
        publish_skeleton_frames(count_people_node, [[middle_sitting, right_standing]])
    response = call_count_people(count_people_node)
    result = json.loads(response.result_json)
    assert response.count == 3
    assert [entity["pose"] for entity in result["human_entities"]] == ["standing", "sitting", "unknown"]


def test_stale_skeletons_fall_back_to_fresh_boxes(count_people_node) -> None:
    """Special function: Stale pose observations do not override fresh person-box counting."""
    skeleton = make_skeleton("lying", bbox=(10, 10, 60, 140))
    publish_skeleton_frames(count_people_node, [[skeleton], [skeleton], [skeleton]])
    count_people_node.last_skeleton_time_sec = count_people_node.now_sec() - 10.0
    person = make_detection(bbox=(10, 10, 60, 140))
    publish_frames(count_people_node, [[person], [person], [person]])
    response = call_count_people(count_people_node)
    result = json.loads(response.result_json)
    assert response.success
    assert response.count == 1
    assert result["pose_quantities"] == {"unknown": 1}
