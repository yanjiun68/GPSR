# flake8: noqa: E501
"""Section: Convert trusted visual approach poses into canonical GPSR navigation queues."""

from __future__ import annotations

# Section: JSON and numeric validation protect the executor from malformed model output.
import json
import math
import uuid
from typing import Any

# Section: The helper returns the same typed queue consumed by the existing executor action.
from geometry_msgs.msg import PoseStamped
from robot_ai_interfaces.msg import ActionQueue, ActionStep


def pose_to_navigation_dict(pose: PoseStamped) -> dict[str, Any]:
    """Special function: Validate and serialize a resolver-owned map pose for Nav2Client."""
    # Section: Only map-frame targets are legal at the GPSR navigation boundary.
    frame_id = str(pose.header.frame_id).strip().lstrip("/")
    if frame_id != "map":
        raise ValueError("Visual navigation approach pose must use the map frame.")

    # Section: Copy every position and orientation component into a JSON-safe list for validation.
    values = [
        float(pose.pose.position.x),
        float(pose.pose.position.y),
        float(pose.pose.position.z),
        float(pose.pose.orientation.x),
        float(pose.pose.orientation.y),
        float(pose.pose.orientation.z),
        float(pose.pose.orientation.w),
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Visual navigation approach pose contains non-finite values.")
    quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
    if quaternion_norm <= 1.0e-9:
        raise ValueError("Visual navigation approach pose contains a zero quaternion.")

    # Section: Normalize orientation so downstream Nav2 never receives an invalid quaternion.
    return {
        "frame_id": "map",
        "position": {"x": values[0], "y": values[1], "z": values[2]},
        "orientation": {
            "x": values[3] / quaternion_norm,
            "y": values[4] / quaternion_norm,
            "z": values[5] / quaternion_norm,
            "w": values[6] / quaternion_norm,
        },
    }


def build_visual_navigation_queue(
    approach_pose: PoseStamped,
    target_id: str,
    *,
    task_id: str = "",
    timeout_sec: float = 60.0,
    max_retries: int = 1,
) -> ActionQueue:
    """Special function: Build one validated NAVIGATE queue from a resolved Qwen target."""
    # Section: The resolver pose is authoritative; Qwen's target ID remains audit metadata only.
    target_pose = pose_to_navigation_dict(approach_pose)
    resolved_task_id = str(task_id).strip() or f"visual_nav_{uuid.uuid4().hex}"
    resolved_target_id = str(target_id).strip() or "unnamed_visual_target"
    bounded_timeout = max(1.0, float(timeout_sec))
    bounded_retries = max(0, int(max_retries))

    # Section: Reuse the existing primitive and its raw-pose route instead of adding a second Nav2 executor.
    step = ActionStep()
    step.step_id = f"{resolved_task_id}_approach"
    step.action_type = ActionStep.NAVIGATE
    step.task_id = resolved_task_id
    step.task_family = "visual_approach"
    step.target_object = ""
    step.target_person = ""
    step.target_location = ""
    step.target_category = ""
    step.parameters_json = json.dumps(
        {
            "target_pose": target_pose,
            "target_reference": resolved_target_id,
            "target_source": "QWEN_SEGMENTED_ROI_DEPTH",
            "require_arrival_verification": True,
        },
        separators=(",", ":"),
        sort_keys=True,
    )
    step.timeout_sec = bounded_timeout
    step.max_retries = bounded_retries
    step.requires_success = []
    step.requires_data = []
    step.result_variable = "visual_navigation_result"

    # Section: A single-step typed queue can be submitted directly to ExecuteActionQueue.
    queue = ActionQueue()
    queue.task_id = resolved_task_id
    queue.steps = [step]
    return queue
