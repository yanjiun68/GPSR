# flake8: noqa: E501
"""Section: Bridge stamped Qwen grounding output into a resolved GPSR NAVIGATE queue."""

from __future__ import annotations

# Section: JSON parsing, identifiers, synchronization, and callback binding.
import json
import math
import threading
import uuid
from dataclasses import dataclass
from functools import partial
from typing import Any

# Section: ROS action clients, messages, QoS, and executor support.
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_ai_interfaces.action import ExecuteActionQueue, ResolveVisualTarget
from robot_ai_interfaces.msg import ActionQueue
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Section: Queue construction reuses the executor's established raw-pose NAVIGATE contract.
from gpsr_task_scheduler.visual_navigation import build_visual_navigation_queue


@dataclass(frozen=True)
class GroundingRequest:
    """Special function: Hold one validated, stamped Qwen entity selection."""

    # Section: Correlation and semantic identity remain separate from geometry.
    request_id: str
    target_id: str
    # Section: Bounding coordinates retain Qwen's declared coordinate space.
    roi: tuple[float, float, float, float]
    coordinate_width: int
    coordinate_height: int
    # Section: The wrapper-provided RGB stamp selects corresponding cached aligned depth.
    image_stamp: TimeMsg
    # Section: Execution policy is caller data but remains bounded again by this bridge.
    stand_off_distance: float
    require_mask: bool


def parse_grounding_payload(
    text: str,
    *,
    default_stand_off_distance: float,
    default_require_mask: bool,
) -> GroundingRequest:
    """Special function: Validate the ROS envelope around model-generated grounding JSON."""
    # Section: Model output must be a JSON object with a non-null selected entity.
    payload = json.loads(str(text))
    if not isinstance(payload, dict):
        raise ValueError("Qwen grounding payload must be a JSON object.")
    entity = payload.get("selected_entity")
    if not isinstance(entity, dict):
        raise ValueError("Qwen did not select an entity.")

    # Section: A four-value ordered bounding box is mandatory; prose regions are never accepted.
    bbox = entity.get("bbox_2d")
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise ValueError("selected_entity.bbox_2d must contain four coordinates.")
    roi = tuple(float(value) for value in bbox)
    if not all(math.isfinite(value) for value in roi) or roi[2] <= roi[0] or roi[3] <= roi[1]:
        raise ValueError("selected_entity.bbox_2d is non-finite or unordered.")

    # Section: Coordinate dimensions describe the model output space, normally normalized 1000x1000.
    coordinate_width = int(entity.get("coordinate_width", payload.get("coordinate_width", 0)))
    coordinate_height = int(entity.get("coordinate_height", payload.get("coordinate_height", 0)))
    if coordinate_width <= 0 or coordinate_height <= 0:
        raise ValueError("Qwen grounding coordinate dimensions must be positive.")

    # Section: The ROS wrapper, not Qwen, must copy the source RGB header stamp into this envelope.
    raw_stamp = payload.get("image_stamp")
    if not isinstance(raw_stamp, dict):
        raise ValueError("Qwen grounding envelope is missing image_stamp.")
    image_stamp = TimeMsg()
    image_stamp.sec = int(raw_stamp.get("sec", 0))
    image_stamp.nanosec = int(raw_stamp.get("nanosec", 0))
    if image_stamp.sec == 0 and image_stamp.nanosec == 0 and not bool(payload.get("use_latest_depth", False)):
        raise ValueError("Zero image_stamp requires explicit use_latest_depth=true.")

    # Section: Apply local defaults only to execution policy, never to image geometry.
    stand_off = float(payload.get("stand_off_distance", default_stand_off_distance))
    if not math.isfinite(stand_off) or stand_off < 0.0:
        raise ValueError("stand_off_distance must be finite and non-negative.")
    request_id = str(payload.get("request_id") or f"qwen_visual_{uuid.uuid4().hex}").strip()
    target_id = str(entity.get("target_id") or entity.get("label") or "").strip()
    if not target_id:
        raise ValueError("selected_entity requires target_id or label.")
    return GroundingRequest(
        request_id=request_id,
        target_id=target_id,
        roi=(roi[0], roi[1], roi[2], roi[3]),
        coordinate_width=coordinate_width,
        coordinate_height=coordinate_height,
        image_stamp=image_stamp,
        stand_off_distance=stand_off,
        require_mask=bool(payload.get("require_mask", default_require_mask)),
    )


def stamps_match(first: TimeMsg, second: TimeMsg, maximum_skew_sec: float) -> bool:
    """Special function: Correlate a segmentation mask with the Qwen source observation."""
    # Section: Zero-stamped masks cannot claim association with a historical model image.
    first_sec = float(first.sec) + float(first.nanosec) * 1.0e-9
    second_sec = float(second.sec) + float(second.nanosec) * 1.0e-9
    if first_sec <= 0.0 or second_sec <= 0.0:
        return False
    return abs(first_sec - second_sec) <= max(0.0, float(maximum_skew_sec))


class VisualNavigationBridge(Node):
    """Special function: Resolve Qwen grounding and optionally dispatch its typed navigation queue."""

    def __init__(self) -> None:
        """Special function: Configure the grounding, mask, resolver, queue, and status boundaries."""
        # Section: Initialize the bridge independently of both the VLM and motion executor processes.
        super().__init__("qwen_visual_navigation_bridge")

        # Section: Topic and action names remain deployment-configurable.
        self.declare_parameter("grounding_topic", "/ai/qwen/grounding_json")
        self.declare_parameter("mask_topic", "/ai/qwen/selected_mask")
        self.declare_parameter("resolved_queue_topic", "/gpsr/visual_navigation_queue")
        self.declare_parameter("status_topic", "/gpsr/visual_navigation_status")
        self.declare_parameter("resolver_action", "/vision/resolve_qwen_roi")
        self.declare_parameter("executor_action", "/execute_action_queue")

        # Section: Automatic physical motion is disabled unless the launch configuration opts in.
        self.declare_parameter("auto_execute", False)
        self.declare_parameter("default_require_mask", True)
        self.declare_parameter("default_stand_off_distance", 0.90)
        self.declare_parameter("navigation_timeout_sec", 60.0)
        self.declare_parameter("navigation_max_retries", 1)
        self.declare_parameter("mask_timestamp_skew_sec", 0.20)

        # Section: Reentrant callbacks permit action results while subscriptions remain active.
        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._busy = False
        self._latest_mask: Image | None = None

        # Section: Sensor QoS prevents the bridge from forcing reliable delivery on a best-effort mask producer.
        self._mask_sub = self.create_subscription(
            Image,
            str(self.get_parameter("mask_topic").value),
            self._mask_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        # Section: Grounding JSON is an event and therefore uses ordinary reliable topic QoS.
        self._grounding_sub = self.create_subscription(
            String,
            str(self.get_parameter("grounding_topic").value),
            self._grounding_callback,
            10,
            callback_group=self._callback_group,
        )

        # Section: Publish the exact typed queue before any optional action submission.
        self._queue_pub = self.create_publisher(
            ActionQueue,
            str(self.get_parameter("resolved_queue_topic").value),
            10,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )

        # Section: Persistent clients avoid repeated ROS graph discovery for each VLM inference.
        self._resolver_client = ActionClient(
            self,
            ResolveVisualTarget,
            str(self.get_parameter("resolver_action").value),
            callback_group=self._callback_group,
        )
        self._executor_client = ActionClient(
            self,
            ExecuteActionQueue,
            str(self.get_parameter("executor_action").value),
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "Qwen visual navigation bridge ready; auto_execute=%s"
            % bool(self.get_parameter("auto_execute").value)
        )

    def _publish_status(self, phase: str, request_id: str, message: str, **extra: Any) -> None:
        """Special function: Publish one machine-readable bridge decision for operators and Qwen."""
        # Section: Status never contains image data, masks, or arbitrary model prose.
        payload = {"phase": str(phase), "request_id": str(request_id), "message": str(message)}
        payload.update(extra)
        status = String()
        status.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self._status_pub.publish(status)

    def _release(self) -> None:
        """Special function: Reopen single-admission grounding intake after a terminal outcome."""
        # Section: One visual target at a time prevents mask and task correlation races.
        with self._state_lock:
            self._busy = False

    def _mask_callback(self, message: Image) -> None:
        """Special function: Retain the latest segmentation mask for timestamp correlation."""
        # Section: The complete message retains encoding, dimensions, and acquisition stamp.
        with self._state_lock:
            self._latest_mask = message

    def _grounding_callback(self, message: String) -> None:
        """Special function: Validate one Qwen event and start asynchronous 3D resolution."""
        # Section: Reject concurrent grounding rather than binding a mask to the wrong target.
        with self._state_lock:
            if self._busy:
                self._publish_status("REJECTED", "", "visual_navigation_busy")
                return
            self._busy = True
            latest_mask = self._latest_mask
        try:
            request = parse_grounding_payload(
                message.data,
                default_stand_off_distance=float(self.get_parameter("default_stand_off_distance").value),
                default_require_mask=bool(self.get_parameter("default_require_mask").value),
            )
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._publish_status("REJECTED", "", str(exc))
            self._release()
            return

        # Section: A requested mask is attached only when its header belongs to the source RGB observation.
        selected_mask = Image()
        if latest_mask is not None and stamps_match(
            request.image_stamp,
            latest_mask.header.stamp,
            float(self.get_parameter("mask_timestamp_skew_sec").value),
        ):
            selected_mask = latest_mask
        if request.require_mask and int(selected_mask.height) <= 0:
            self._publish_status("REJECTED", request.request_id, "matching_segmentation_mask_unavailable")
            self._release()
            return
        if not self._resolver_client.wait_for_server(timeout_sec=0.25):
            self._publish_status("FAILED", request.request_id, "visual_target_resolver_unavailable")
            self._release()
            return

        # Section: Transfer only structured grounding data into the typed resolver action.
        goal = ResolveVisualTarget.Goal()
        goal.target_id = request.target_id
        goal.roi_x_min, goal.roi_y_min, goal.roi_x_max, goal.roi_y_max = request.roi
        goal.coordinate_width = request.coordinate_width
        goal.coordinate_height = request.coordinate_height
        goal.image_stamp = request.image_stamp
        goal.segmentation_mask = selected_mask
        goal.require_mask = request.require_mask
        goal.stand_off_distance = request.stand_off_distance
        self._publish_status("RESOLVING", request.request_id, "grounding_accepted", target_id=request.target_id)
        future = self._resolver_client.send_goal_async(goal)
        future.add_done_callback(partial(self._resolver_goal_response, request=request))

    def _resolver_goal_response(self, future: Any, *, request: GroundingRequest) -> None:
        """Special function: Continue only when the ROI resolver accepts the typed goal."""
        # Section: Transport errors and rejected goals terminate before any ActionQueue exists.
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status("FAILED", request.request_id, f"resolver_goal_error: {exc}")
            self._release()
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status("FAILED", request.request_id, "resolver_goal_rejected")
            self._release()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self._resolver_result, request=request))

    def _resolver_result(self, future: Any, *, request: GroundingRequest) -> None:
        """Special function: Convert a successful stand-off pose into the canonical typed queue."""
        # Section: Resolution failure remains perception data and never produces a navigation command.
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self._publish_status("FAILED", request.request_id, f"resolver_result_error: {exc}")
            self._release()
            return
        if result is None or not bool(result.success):
            self._publish_status(
                "FAILED",
                request.request_id,
                str(getattr(result, "message", "visual_target_resolution_failed")),
                result_code=int(getattr(result, "result_code", -1)),
            )
            self._release()
            return
        try:
            queue = build_visual_navigation_queue(
                result.approach_pose,
                request.target_id,
                task_id=request.request_id,
                timeout_sec=float(self.get_parameter("navigation_timeout_sec").value),
                max_retries=int(self.get_parameter("navigation_max_retries").value),
            )
        except ValueError as exc:
            self._publish_status("FAILED", request.request_id, str(exc))
            self._release()
            return

        # Section: Publishing always precedes optional execution so operators can inspect the exact goal.
        self._queue_pub.publish(queue)
        self._publish_status(
            "QUEUE_READY",
            request.request_id,
            "resolved_visual_navigation_queue",
            target_id=request.target_id,
            depth_m=float(result.depth_m),
            valid_depth_ratio=float(result.valid_depth_ratio),
            auto_execute=bool(self.get_parameter("auto_execute").value),
        )
        if not bool(self.get_parameter("auto_execute").value):
            self._release()
            return
        if not self._executor_client.wait_for_server(timeout_sec=0.25):
            self._publish_status("FAILED", request.request_id, "action_queue_executor_unavailable")
            self._release()
            return

        # Section: The executor, not this bridge, owns navigation timeout, cancellation, retry, and completion.
        execute_goal = ExecuteActionQueue.Goal()
        execute_goal.queue = queue
        execute_future = self._executor_client.send_goal_async(execute_goal)
        execute_future.add_done_callback(partial(self._executor_goal_response, request=request))

    def _executor_goal_response(self, future: Any, *, request: GroundingRequest) -> None:
        """Special function: Track executor admission before waiting for navigation completion."""
        # Section: A queue rejection is a terminal scheduling failure, not perception failure.
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status("FAILED", request.request_id, f"executor_goal_error: {exc}")
            self._release()
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status("FAILED", request.request_id, "action_queue_rejected")
            self._release()
            return
        self._publish_status("EXECUTING", request.request_id, "navigation_queue_accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self._executor_result, request=request))

    def _executor_result(self, future: Any, *, request: GroundingRequest) -> None:
        """Special function: Publish the authoritative queue outcome and reopen intake."""
        # Section: Mirror only the executor's terminal result; the bridge never infers arrival from time.
        try:
            wrapped = future.result()
            result = wrapped.result
            succeeded = bool(result.success)
            self._publish_status(
                "SUCCEEDED" if succeeded else "FAILED",
                request.request_id,
                str(result.outcome),
                completed_results=len(result.results),
            )
        except Exception as exc:
            self._publish_status("FAILED", request.request_id, f"executor_result_error: {exc}")
        finally:
            self._release()


def main(args: list[str] | None = None) -> None:
    """Special function: Run subscriptions and both action-client chains concurrently."""
    # Section: Multiple executor threads prevent action callbacks from blocking grounding intake state updates.
    rclpy.init(args=args)
    node = VisualNavigationBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
