"""Section: Find object perception server tests."""

from __future__ import annotations

import importlib.util
import json
import sys
import threading
from types import SimpleNamespace
from pathlib import Path

from putrabot_interfaces.msg import ObjectDetection, ObjectDetections


def load_find_object_module():
    """Special function: Load the vision script as a testable Python module."""
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "find_object_perception_server.py"
    spec = importlib.util.spec_from_file_location("find_object_perception_server", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def make_detection(name: str, confidence: float = 0.90, center_u: float = 42.0, center_v: float = 24.0) -> ObjectDetection:
    """Special function: Build one synthetic object detection message."""
    detection = ObjectDetection()
    detection.name = name
    detection.confidence = confidence
    detection.center_u = center_u
    detection.center_v = center_v
    return detection


class RecordingPublisher:
    """Special function: Capture item-found topic messages published by the server."""

    def __init__(self) -> None:
        """Special function: Initialize the in-memory message list."""
        self.messages: list[str] = []

    def publish(self, msg) -> None:
        """Special function: Record one published std_msgs/String payload."""
        self.messages.append(msg.data)


class ReadyRecordingTtsClient:
    """Special function: Capture TTS requests sent by the server."""

    def __init__(self) -> None:
        """Special function: Initialize the in-memory TTS request list."""
        self.requests: list[str] = []

    def service_is_ready(self) -> bool:
        """Special function: Report that the fake TTS service is ready."""
        return True

    def call_async(self, request) -> None:
        """Special function: Record one asynchronous TTS request."""
        self.requests.append(request.text)


class RecordingLogger:
    """Special function: Accept warning logs from server helpers under test."""

    def warn(self, _message: str) -> None:
        """Special function: Ignore warning text during helper tests."""
        return None


def make_server_with_preset(module, **overrides):
    """Special function: Build a lightweight preset-target server test double."""
    server = object.__new__(module.FindObjectPerceptionServer)
    defaults = {
        "target_object_name": "bottle",
        "min_confidence": 0.35,
        "item_found_cooldown_sec": 0.0,
        "speak_on_found": True,
    }
    defaults.update(overrides)
    server.detection_lock = threading.Lock()
    server.detection_event = threading.Event()
    server.latest_detections = []
    server.last_item_found_time_sec = 0.0
    server.last_item_found_object_id = ""
    server.last_item_found_payload = ""
    server.item_found_pub = RecordingPublisher()
    server.tts_client = ReadyRecordingTtsClient()
    server.get_parameter = lambda name: SimpleNamespace(value=defaults[name])
    server.get_logger = lambda: RecordingLogger()
    return server


def publish_detections_to_server(module, server, detections: list[ObjectDetection]) -> None:
    """Special function: Feed one synthetic ObjectDetections frame into the server."""
    msg = ObjectDetections()
    msg.detections = detections
    module.FindObjectPerceptionServer.detections_callback(server, msg)


def parse_item_found(message: str) -> dict:
    """Special function: Parse one item-found topic JSON message."""
    return json.loads(message)


def test_normalize_name_matches_spacing_and_case() -> None:
    """Special function: Object names normalize for exact target/detection comparison."""
    module = load_find_object_module()
    assert module.normalize_name("Red Cup") == "red_cup"
    assert module.normalize_name("red-cup") == "red_cup"


def test_find_matching_detection_uses_exact_name_and_confidence() -> None:
    """Special function: Name-only matching picks the highest-confidence exact target match."""
    module = load_find_object_module()
    server = object.__new__(module.FindObjectPerceptionServer)
    server.detection_lock = module.threading.Lock()
    server.latest_detections = [
        make_detection("bottle", 0.95),
        make_detection("cup", 0.40, 10.0, 20.0),
        make_detection("cup", 0.80, 30.0, 40.0),
    ]
    match = module.FindObjectPerceptionServer.find_matching_detection(server, "cup", 0.35)
    assert match is not None
    assert match.name == "cup"
    assert match.confidence == 0.80


def test_build_result_reports_found_object_id() -> None:
    """Special function: Found action result includes object id from target and detection center."""
    module = load_find_object_module()
    server = object.__new__(module.FindObjectPerceptionServer)
    result = module.FindObjectPerceptionServer.build_result(
        server,
        True,
        "cup",
        make_detection("cup", 0.90, 12.0, 34.0),
        "cup is found.",
    )
    assert result.found
    assert result.object_id == "cup_12_34"
    assert result.message == "cup is found."


def test_preset_target_publishes_item_found_without_tts() -> None:
    """Special function: Preset target detection publishes item-found status without TTS."""
    module = load_find_object_module()
    server = make_server_with_preset(module)
    publish_detections_to_server(module, server, [make_detection("bottle", 0.90)])
    payload = parse_item_found(server.item_found_pub.messages[0])
    assert payload["detected_item"] == "bottle"
    assert payload["itemfound"] is True
    assert payload["target_object_name"] == "bottle"
    assert server.tts_client.requests == []


def test_preset_target_false_status_reports_detected_non_target_item() -> None:
    """Special function: Preset target false status reports detected non-target item."""
    module = load_find_object_module()
    server = make_server_with_preset(module)
    publish_detections_to_server(module, server, [make_detection("red-cup", 0.90)])
    payload = parse_item_found(server.item_found_pub.messages[0])
    assert payload["detected_item"] == "red_cup"
    assert payload["itemfound"] is False
    assert payload["target_object_name"] == "bottle"
    assert server.tts_client.requests == []


def test_preset_target_ignores_low_confidence_detection() -> None:
    """Special function: Preset target detection respects the configured confidence threshold."""
    module = load_find_object_module()
    server = make_server_with_preset(module)
    publish_detections_to_server(module, server, [make_detection("bottle", 0.20)])
    payload = parse_item_found(server.item_found_pub.messages[0])
    assert payload["detected_item"] == ""
    assert payload["itemfound"] is False
    assert server.tts_client.requests == []


def test_empty_preset_target_disables_item_found_output() -> None:
    """Special function: Empty preset target keeps passive item-found output disabled."""
    module = load_find_object_module()
    server = make_server_with_preset(module, target_object_name="")
    publish_detections_to_server(module, server, [make_detection("bottle", 0.90)])
    assert server.item_found_pub.messages == []
    assert server.tts_client.requests == []
