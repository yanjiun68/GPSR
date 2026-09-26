"""Section: Count people blackboard TTS node tests."""

# Section: Test fixtures, deterministic fakes, and regression scenarios.

from __future__ import annotations

import importlib
import json
import os
from pathlib import Path

import pytest
import rclpy
from std_msgs.msg import String

from robot_yasmin_tasks.gpsr import blackboard_keys as bb


@pytest.fixture()
def count_people_tts_node():
    """Special function: Create and destroy a fresh count-people TTS ROS node per test."""
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/roslogs")
    Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    if not rclpy.ok():
        rclpy.init()
    module = importlib.import_module("robot_yasmin_tasks.ros_servers.count_people_blackboard_tts_node")
    node = module.CountPeopleBlackboardTTSNode()
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def make_payload(count: int, confirmed: bool = True) -> String:
    """Special function: Build one synthetic human quantity message."""
    msg = String()
    msg.data = json.dumps({
        "confirmed": confirmed,
        "human_quantities": {"person": count},
        "total_detected_count": count,
    })
    return msg


def test_confirmed_people_count_updates_blackboard_and_queues_tts(count_people_tts_node) -> None:
    """Special function: Confirmed people quantity payload updates blackboard and speech queue."""
    count_people_tts_node.set_parameters([])
    count_people_tts_node.human_quantities_callback(make_payload(2))
    assert count_people_tts_node.blackboard[bb.COUNT_RESULT] == 2
    assert count_people_tts_node.blackboard[bb.DETECTED_PEOPLE] == [
        {"entity_id": "person_1", "name": "person"},
        {"entity_id": "person_2", "name": "person"},
    ]
    assert count_people_tts_node.blackboard[bb.REPORT_TEXT] == "I can see two people."
    assert list(count_people_tts_node.speech_queue) == ["I can see two people."]


def test_unconfirmed_people_count_is_ignored_by_default(count_people_tts_node) -> None:
    """Special function: Unconfirmed people quantity payload does not update default reporter state."""
    count_people_tts_node.human_quantities_callback(make_payload(3, confirmed=False))
    assert count_people_tts_node.blackboard[bb.COUNT_RESULT] == 0
    assert list(count_people_tts_node.speech_queue) == []
