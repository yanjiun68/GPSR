# flake8: noqa: E501
"""Section: Typed ActionQueue construction for Qwen-resolved approach poses."""

# Section: JSON inspection and ROS pose construction support queue assertions.
import json
import math

from geometry_msgs.msg import PoseStamped

from gpsr_task_scheduler.visual_navigation import build_visual_navigation_queue
from gpsr_task_scheduler.visual_navigation_bridge import parse_grounding_payload


def make_pose() -> PoseStamped:
    """Special function: Build one valid map-frame approach pose."""
    # Section: Use a deliberately non-normalized quaternion to verify boundary normalization.
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.25
    pose.pose.position.y = -0.50
    pose.pose.orientation.z = 1.0
    pose.pose.orientation.w = 1.0
    return pose


def test_visual_target_becomes_raw_pose_navigate_step() -> None:
    """Special function: Resolver output enters the existing NAVIGATE primitive without a waypoint name."""
    # Section: Build a queue with stable IDs and inspect the parameters consumed by Nav2Client.
    queue = build_visual_navigation_queue(
        make_pose(),
        "red_cup_7",
        task_id="qwen_nav_1",
        timeout_sec=45.0,
        max_retries=2,
    )
    assert queue.task_id == "qwen_nav_1"
    assert len(queue.steps) == 1
    step = queue.steps[0]
    assert step.action_type == "NAVIGATE"
    assert step.target_location == ""
    assert step.timeout_sec == 45.0
    assert step.max_retries == 2
    parameters = json.loads(step.parameters_json)
    assert parameters["target_reference"] == "red_cup_7"
    assert parameters["target_source"] == "QWEN_SEGMENTED_ROI_DEPTH"
    assert parameters["target_pose"]["frame_id"] == "map"
    assert math.isclose(parameters["target_pose"]["orientation"]["z"], math.sqrt(0.5))
    assert math.isclose(parameters["target_pose"]["orientation"]["w"], math.sqrt(0.5))


def test_non_map_visual_pose_is_rejected() -> None:
    """Special function: Camera-frame coordinates cannot bypass TF and enter the navigation queue."""
    # Section: Preserve the executor's map-only raw-pose safety contract.
    pose = make_pose()
    pose.header.frame_id = "camera_color_optical_frame"
    try:
        build_visual_navigation_queue(pose, "target")
    except ValueError as exc:
        assert "map frame" in str(exc)
        return
    raise AssertionError("Non-map visual navigation pose must be rejected.")


def test_stamped_qwen_grounding_envelope_is_accepted() -> None:
    """Special function: Wrapper metadata combines with model grounding without model-generated depth."""
    # Section: The source image stamp lives outside selected_entity because the ROS wrapper owns it.
    parsed = parse_grounding_payload(
        json.dumps({
            "request_id": "visual_1",
            "image_stamp": {"sec": 12, "nanosec": 300},
            "selected_entity": {
                "target_id": "person_blue_shirt",
                "bbox_2d": [100, 200, 400, 900],
                "coordinate_width": 1000,
                "coordinate_height": 1000,
            },
        }),
        default_stand_off_distance=0.9,
        default_require_mask=True,
    )
    assert parsed.request_id == "visual_1"
    assert parsed.target_id == "person_blue_shirt"
    assert parsed.roi == (100.0, 200.0, 400.0, 900.0)
    assert parsed.image_stamp.sec == 12
    assert parsed.require_mask


def test_unstamped_qwen_grounding_is_rejected() -> None:
    """Special function: Model coordinates cannot be paired with an unrelated latest depth silently."""
    # Section: The wrapper must either supply the true stamp or explicitly request latest-depth behavior.
    payload = {
        "selected_entity": {
            "target_id": "cup",
            "bbox_2d": [100, 100, 200, 200],
            "coordinate_width": 1000,
            "coordinate_height": 1000,
        },
        "image_stamp": {"sec": 0, "nanosec": 0},
    }
    try:
        parse_grounding_payload(
            json.dumps(payload),
            default_stand_off_distance=0.9,
            default_require_mask=False,
        )
    except ValueError as exc:
        assert "use_latest_depth" in str(exc)
        return
    raise AssertionError("An unstamped observation must not bind to arbitrary cached depth.")
