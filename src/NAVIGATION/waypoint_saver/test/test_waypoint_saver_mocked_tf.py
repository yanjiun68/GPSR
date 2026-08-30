"""Section: Mocked-TF waypoint save tests requiring no physical TF tree or robot."""

# Section: Import the installed ROS messages and source server helpers.
from pathlib import Path
import sys

import pytest
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from yaml import safe_load

SCRIPT_DIRECTORY = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIRECTORY))

from waypoint_saver_logic import WaypointValidationError  # noqa: E402
from waypoint_saver_server import save_current_waypoint  # noqa: E402


class MockTransformBuffer:
    """Special class: Return one fabricated transform or raise one configured TF exception."""

    def __init__(self, transform: TransformStamped | None = None, error: Exception | None = None):
        """Special function: Store the deterministic lookup result and call history."""
        # Section: Avoid every live TF subscription and physical frame dependency.
        self.transform = transform
        self.error = error
        self.calls: list[tuple[str, str]] = []

    def lookup_transform(self, target_frame, source_frame, _time, timeout=None):
        """Special function: Emulate Buffer.lookup_transform for map-to-base requests."""
        # Section: Record requested frames so the test verifies the service lookup contract.
        del timeout
        self.calls.append((target_frame, source_frame))
        if self.error is not None:
            raise self.error
        return self.transform


def fabricated_transform(timestamp_sec: float) -> TransformStamped:
    """Special function: Build a complete map-to-base transform with a fabricated timestamp."""
    # Section: Split fractional seconds into ROS integer second and nanosecond fields.
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "base_footprint"
    transform.header.stamp.sec = int(timestamp_sec)
    transform.header.stamp.nanosec = int(round((timestamp_sec - int(timestamp_sec)) * 1_000_000_000))
    transform.transform.translation.x = 1.25
    transform.transform.translation.y = -0.50
    transform.transform.translation.z = 0.0
    transform.transform.rotation.w = 1.0
    return transform


def execute_mocked_save(buffer: MockTransformBuffer, yaml_path: Path) -> tuple[int, object]:
    """Special function: Invoke isolated service logic using fixed frames and fabricated time."""
    # Section: Use the production default 1.0-second freshness threshold without a ROS node.
    return save_current_waypoint(
        buffer,
        yaml_path,
        "KITCHEN",
        -1,
        False,
        "map",
        "base_footprint",
        2.0,
        1.0,
        100.0,
    )


def test_fresh_mocked_tf_succeeds_and_writes_yaml(tmp_path: Path) -> None:
    """Special function: Save a fresh fabricated TF pose to a newly created YAML file."""
    # Section: Required mocked-TF success case.
    yaml_path = tmp_path / "workspace" / "waypoints" / "gpsr_waypoint.yaml"
    buffer = MockTransformBuffer(transform=fabricated_transform(99.5))
    waypoint_id, pose = execute_mocked_save(buffer, yaml_path)
    payload = safe_load(yaml_path.read_text(encoding="utf-8"))
    assert waypoint_id == 0
    assert pose.pose.position.x == pytest.approx(1.25)
    assert buffer.calls == [("map", "base_footprint")]
    assert payload["waypoints"][0]["name"] == "KITCHEN"


def test_stale_mocked_tf_fails_without_writing_yaml(tmp_path: Path) -> None:
    """Special function: Reject stale fabricated TF before creating any waypoint file."""
    # Section: Required mocked-TF stale case.
    yaml_path = tmp_path / "waypoints" / "gpsr_waypoint.yaml"
    buffer = MockTransformBuffer(transform=fabricated_transform(97.0))
    with pytest.raises(WaypointValidationError, match="stale"):
        execute_mocked_save(buffer, yaml_path)
    assert not yaml_path.exists()


def test_mocked_tf_lookup_exception_fails_without_writing_yaml(tmp_path: Path) -> None:
    """Special function: Propagate TF lookup failure without changing the filesystem."""
    # Section: Required mocked-TF exception case.
    yaml_path = tmp_path / "waypoints" / "gpsr_waypoint.yaml"
    buffer = MockTransformBuffer(error=TransformException("fabricated lookup failure"))
    with pytest.raises(TransformException, match="fabricated lookup failure"):
        execute_mocked_save(buffer, yaml_path)
    assert not yaml_path.exists()
