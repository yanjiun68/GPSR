#!/usr/bin/env python3
# flake8: noqa: E501
"""Section: Resolve a Qwen-grounded ROI into map-frame target and approach poses."""

from __future__ import annotations

# Section: Standard-library support for bounded observation storage and thread safety.
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

# Section: Numeric image processing used for mask and depth sampling.
import cv2
import numpy as np

# Section: ROS messages, actions, QoS, and TF integration.
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from robot_ai_interfaces.action import ResolveVisualTarget
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

# Section: Importing tf2_geometry_msgs registers PointStamped transformations with tf2_ros.
import tf2_geometry_msgs  # noqa: F401


@dataclass(frozen=True)
class DepthObservation:
    """Special function: Keep one aligned depth frame with ROS and local receipt times."""

    # Section: The ROS stamp correlates Qwen's source image with the measured depth frame.
    stamp_sec: float
    # Section: The monotonic receipt time supports freshness checks when stamps are unavailable.
    received_monotonic: float
    # Section: The original message is retained until one ROI is known and can be sampled.
    message: Image


@dataclass(frozen=True)
class DepthSample:
    """Special function: Carry one robust metric sample and its source pixel diagnostics."""

    # Section: Metric depth along the camera optical Z axis.
    depth_m: float
    # Section: Representative image pixel selected only from valid target-supported depth.
    pixel_u: float
    pixel_v: float
    # Section: Fraction of eligible mask or ROI pixels containing acceptable depth.
    valid_ratio: float
    valid_count: int


def stamp_to_seconds(stamp: Any) -> float:
    """Special function: Convert a ROS stamp-like object to floating-point seconds."""
    # Section: Explicit conversion avoids depending on a live ROS clock in geometry tests.
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1.0e-9


def stamp_is_zero(stamp: Any) -> bool:
    """Special function: Detect a caller that intentionally requests the latest observation."""
    # Section: A zero stamp is the documented opt-in to latest-depth behavior.
    return int(getattr(stamp, "sec", 0)) == 0 and int(getattr(stamp, "nanosec", 0)) == 0


def scale_and_clip_roi(
    roi: tuple[float, float, float, float],
    coordinate_size: tuple[int, int],
    image_size: tuple[int, int],
) -> tuple[int, int, int, int]:
    """Special function: Map Qwen coordinates into the aligned-depth image and clip bounds."""
    # Section: Reject undefined coordinate systems before applying any image-space motion data.
    coordinate_width, coordinate_height = coordinate_size
    image_width, image_height = image_size
    if coordinate_width <= 0 or coordinate_height <= 0 or image_width <= 0 or image_height <= 0:
        raise ValueError("ROI coordinate and image dimensions must be positive.")

    # Section: Require finite, ordered coordinates so invalid model JSON cannot select arbitrary pixels.
    x_min, y_min, x_max, y_max = [float(value) for value in roi]
    if not all(math.isfinite(value) for value in (x_min, y_min, x_max, y_max)):
        raise ValueError("ROI coordinates must be finite.")
    if x_max <= x_min or y_max <= y_min:
        raise ValueError("ROI maximum coordinates must exceed minimum coordinates.")

    # Section: Apply proportional scaling for Qwen's configured coordinate space, commonly 0..1000.
    left = int(math.floor(x_min * image_width / coordinate_width))
    top = int(math.floor(y_min * image_height / coordinate_height))
    right = int(math.ceil(x_max * image_width / coordinate_width))
    bottom = int(math.ceil(y_max * image_height / coordinate_height))

    # Section: Clamp every edge and retain at least one pixel in each dimension.
    left = max(0, min(image_width - 1, left))
    top = max(0, min(image_height - 1, top))
    right = max(left + 1, min(image_width, right))
    bottom = max(top + 1, min(image_height, bottom))
    return left, top, right, bottom


def depth_to_meters(depth_image: np.ndarray, encoding: str) -> np.ndarray:
    """Special function: Normalize standard ROS depth encodings into meters."""
    # Section: Preserve floating-point depth while avoiding mutation of the cached ROS image.
    converted = np.asarray(depth_image).astype(np.float32, copy=True)
    normalized_encoding = str(encoding).strip().upper()
    # Section: RealSense and Astra commonly publish unsigned 16-bit millimetres.
    if normalized_encoding in {"16UC1", "MONO16"} or np.asarray(depth_image).dtype == np.uint16:
        converted *= 0.001
    # Section: 32FC1 is already expressed in metres by the ROS depth-image convention.
    elif normalized_encoding != "32FC1" and not np.issubdtype(np.asarray(depth_image).dtype, np.floating):
        raise ValueError(f"Unsupported depth encoding: {encoding}.")
    return converted


def prepare_mask(mask_image: np.ndarray | None, image_size: tuple[int, int]) -> np.ndarray | None:
    """Special function: Convert an optional segmentation image into a depth-sized boolean mask."""
    # Section: Absence is valid for navigation when the caller permits box-only depth sampling.
    if mask_image is None or np.asarray(mask_image).size == 0:
        return None
    mask = np.asarray(mask_image)
    # Section: Collapse colour masks without assuming any particular foreground colour channel.
    if mask.ndim == 3:
        mask = np.any(mask > 0, axis=2).astype(np.uint8)
    elif mask.ndim != 2:
        raise ValueError("Segmentation mask must be a mono or colour image.")
    image_width, image_height = image_size
    # Section: Nearest-neighbour resizing preserves binary instance boundaries.
    if mask.shape[1] != image_width or mask.shape[0] != image_height:
        mask = cv2.resize(mask.astype(np.uint8), (image_width, image_height), interpolation=cv2.INTER_NEAREST)
    return mask > 0


def sample_roi_depth(
    depth_m: np.ndarray,
    roi: tuple[int, int, int, int],
    mask: np.ndarray | None,
    min_depth_m: float,
    max_depth_m: float,
    min_valid_ratio: float,
    min_valid_pixels: int,
    bbox_inset_ratio: float,
    outlier_distance_m: float,
) -> DepthSample:
    """Special function: Estimate target depth from segmented pixels or a conservative inner ROI."""
    # Section: Validate depth shape before indexing model-controlled coordinates.
    if depth_m.ndim != 2 or depth_m.size == 0:
        raise ValueError("Aligned depth image must contain one non-empty channel.")
    left, top, right, bottom = roi
    if left < 0 or top < 0 or right > depth_m.shape[1] or bottom > depth_m.shape[0]:
        raise ValueError("Scaled ROI lies outside the depth image.")

    # Section: A segmentation mask owns the eligible pixels; otherwise inset the box to reduce background.
    eligible = np.zeros(depth_m.shape, dtype=bool)
    if mask is not None:
        if mask.shape != depth_m.shape:
            raise ValueError("Prepared segmentation mask does not match the depth image.")
        eligible[top:bottom, left:right] = mask[top:bottom, left:right]
    else:
        width = right - left
        height = bottom - top
        inset_x = min(max(0, int(round(width * max(0.0, bbox_inset_ratio)))), max(0, (width - 1) // 2))
        inset_y = min(max(0, int(round(height * max(0.0, bbox_inset_ratio)))), max(0, (height - 1) // 2))
        eligible[top + inset_y:bottom - inset_y, left + inset_x:right - inset_x] = True

    # Section: Reject an empty segmentation rather than silently falling back to unrelated background.
    eligible_count = int(np.count_nonzero(eligible))
    if eligible_count <= 0:
        raise ValueError("ROI or segmentation mask contains no eligible pixels.")

    # Section: Keep only finite sensor readings within the configured navigation range.
    valid = eligible & np.isfinite(depth_m) & (depth_m >= min_depth_m) & (depth_m <= max_depth_m)
    raw_valid_count = int(np.count_nonzero(valid))
    valid_ratio = float(raw_valid_count) / float(eligible_count)
    if raw_valid_count < max(1, int(min_valid_pixels)) or valid_ratio < float(min_valid_ratio):
        raise ValueError(
            f"Insufficient target depth support: pixels={raw_valid_count}, ratio={valid_ratio:.3f}."
        )

    # Section: Median-centred rejection removes isolated foreground and flying depth pixels.
    values = depth_m[valid]
    initial_median = float(np.median(values))
    absolute_deviation = np.abs(values - initial_median)
    median_deviation = float(np.median(absolute_deviation))
    robust_limit = max(float(outlier_distance_m), 3.0 * 1.4826 * median_deviation)
    robust_valid = valid & (np.abs(depth_m - initial_median) <= robust_limit)
    robust_count = int(np.count_nonzero(robust_valid))
    if robust_count < max(1, int(min_valid_pixels)):
        raise ValueError("Depth outlier rejection left too few target pixels.")

    # Section: Use robust medians for both metric depth and the representative target pixel.
    rows, columns = np.nonzero(robust_valid)
    final_depth = float(np.median(depth_m[robust_valid]))
    return DepthSample(
        depth_m=final_depth,
        pixel_u=float(np.median(columns)),
        pixel_v=float(np.median(rows)),
        valid_ratio=valid_ratio,
        valid_count=robust_count,
    )


def project_pixel_to_camera(
    pixel_u: float,
    pixel_v: float,
    depth_m: float,
    camera_matrix: list[float] | tuple[float, ...],
) -> tuple[float, float, float]:
    """Special function: Back-project an aligned colour pixel through calibrated pinhole intrinsics."""
    # Section: Extract the four pinhole values required for a metric camera-frame point.
    if len(camera_matrix) != 9:
        raise ValueError("CameraInfo.k must contain nine values.")
    fx, fy = float(camera_matrix[0]), float(camera_matrix[4])
    principal_x, principal_y = float(camera_matrix[2]), float(camera_matrix[5])
    if fx <= 0.0 or fy <= 0.0 or not all(
        math.isfinite(value) for value in (fx, fy, principal_x, principal_y, depth_m)
    ):
        raise ValueError("Camera calibration or target depth is invalid.")
    # Section: Return ROS optical-frame coordinates: X right, Y down, Z forward.
    return (
        (float(pixel_u) - principal_x) * float(depth_m) / fx,
        (float(pixel_v) - principal_y) * float(depth_m) / fy,
        float(depth_m),
    )


def scale_camera_matrix(
    camera_matrix: list[float] | tuple[float, ...],
    calibration_size: tuple[int, int],
    image_size: tuple[int, int],
) -> list[float]:
    """Special function: Scale calibrated intrinsics when aligned depth uses another resolution."""
    # Section: Calibration and image dimensions must describe valid rectangular images.
    if len(camera_matrix) != 9:
        raise ValueError("CameraInfo.k must contain nine values.")
    calibration_width, calibration_height = calibration_size
    image_width, image_height = image_size
    if min(calibration_width, calibration_height, image_width, image_height) <= 0:
        raise ValueError("Camera calibration and image dimensions must be positive.")
    scale_x = float(image_width) / float(calibration_width)
    scale_y = float(image_height) / float(calibration_height)
    scaled = [float(value) for value in camera_matrix]
    # Section: Focal lengths and principal points follow their corresponding image axes.
    scaled[0] *= scale_x
    scaled[2] *= scale_x
    scaled[4] *= scale_y
    scaled[5] *= scale_y
    return scaled


def compute_standoff_geometry(
    robot_xy: tuple[float, float],
    target_xy: tuple[float, float],
    stand_off_distance: float,
) -> tuple[float, float, float]:
    """Special function: Place the base between its current position and the target, facing the target."""
    # Section: A non-negative finite stand-off is mandatory for predictable approach behavior.
    stand_off = float(stand_off_distance)
    if not math.isfinite(stand_off) or stand_off < 0.0:
        raise ValueError("Stand-off distance must be finite and non-negative.")
    robot_x, robot_y = [float(value) for value in robot_xy]
    target_x, target_y = [float(value) for value in target_xy]
    delta_x = target_x - robot_x
    delta_y = target_y - robot_y
    distance = math.hypot(delta_x, delta_y)
    if distance <= 1.0e-6:
        raise ValueError("Robot and target positions are indistinguishable in the map frame.")

    # Section: Stay in place when already inside the requested radius; only update final heading.
    if distance <= stand_off:
        approach_x, approach_y = robot_x, robot_y
    else:
        approach_x = target_x - delta_x * stand_off / distance
        approach_y = target_y - delta_y * stand_off / distance
    heading = math.atan2(target_y - approach_y, target_x - approach_x)
    return approach_x, approach_y, heading


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Special function: Convert a planar heading to a normalized ROS quaternion tuple."""
    # Section: Nav2 target poses use roll and pitch zero with yaw split across Z and W.
    return 0.0, 0.0, math.sin(float(yaw) / 2.0), math.cos(float(yaw) / 2.0)


class QwenRoiDepthResolver(Node):
    """Special function: Serve bounded ROI-to-depth and map stand-off resolution requests."""

    def __init__(self) -> None:
        """Special function: Configure subscriptions, TF, diagnostics publishers, and the action server."""
        # Section: Initialize the ROS node before declaring resolver policy.
        super().__init__("qwen_roi_depth_resolver")

        # Section: Sensor and coordinate-frame parameters must match the aligned RGB-D pipeline.
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("action_name", "/vision/resolve_qwen_roi")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # Section: Depth-cache policy accommodates bounded local Qwen inference latency.
        self.declare_parameter("depth_cache_frames", 120)
        self.declare_parameter("depth_cache_sample_period_sec", 0.10)
        self.declare_parameter("max_observation_age_sec", 20.0)
        self.declare_parameter("max_image_depth_skew_sec", 0.20)
        self.declare_parameter("max_latest_depth_age_sec", 0.75)

        # Section: Geometric validation rejects weak depth before it reaches Nav2.
        self.declare_parameter("min_depth_m", 0.20)
        self.declare_parameter("max_depth_m", 8.0)
        self.declare_parameter("min_valid_depth_ratio", 0.25)
        self.declare_parameter("min_valid_depth_pixels", 20)
        self.declare_parameter("bbox_inset_ratio", 0.15)
        self.declare_parameter("depth_outlier_distance_m", 0.08)
        self.declare_parameter("default_stand_off_distance", 0.90)
        self.declare_parameter("tf_timeout_sec", 0.50)

        # Section: Cache only sampled depth frames to bound memory while retaining inference history.
        cache_frames = max(2, int(self.get_parameter("depth_cache_frames").value))
        self._depth_cache: deque[DepthObservation] = deque(maxlen=cache_frames)
        self._latest_camera_info: CameraInfo | None = None
        self._last_cached_depth_stamp = -math.inf
        self._cache_lock = threading.Lock()
        self._bridge = CvBridge()

        # Section: Sensor and action callbacks may overlap while TF uses the node's default group.
        self._callback_group = ReentrantCallbackGroup()

        # Section: TF owns all sensor-to-map and base-to-map transforms.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Section: Sensor-data QoS matches camera drivers that publish best-effort streams.
        self._depth_sub = self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._depth_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )

        # Section: Published poses let RViz and operators inspect exactly what will enter the queue.
        self._target_pose_pub = self.create_publisher(PoseStamped, "/vision/qwen_target_pose", 10)
        self._approach_pose_pub = self.create_publisher(PoseStamped, "/vision/qwen_approach_pose", 10)

        # Section: The action boundary provides cancellation and explicit perception failure codes.
        self._action_server = ActionServer(
            self,
            ResolveVisualTarget,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "Qwen ROI resolver ready: action=%s depth=%s camera_info=%s"
            % (
                str(self.get_parameter("action_name").value),
                str(self.get_parameter("depth_topic").value),
                str(self.get_parameter("camera_info_topic").value),
            )
        )

    def _depth_callback(self, message: Image) -> None:
        """Special function: Retain a rate-limited history of aligned depth observations."""
        # Section: A bounded sampling period controls memory without changing the camera publisher.
        stamp_sec = stamp_to_seconds(message.header.stamp)
        sample_period = max(0.0, float(self.get_parameter("depth_cache_sample_period_sec").value))
        with self._cache_lock:
            if stamp_sec > 0.0 and stamp_sec - self._last_cached_depth_stamp < sample_period:
                return
            self._depth_cache.append(DepthObservation(stamp_sec, time.monotonic(), message))
            if stamp_sec > 0.0:
                self._last_cached_depth_stamp = stamp_sec

    def _camera_info_callback(self, message: CameraInfo) -> None:
        """Special function: Cache the latest static colour calibration used by aligned depth."""
        # Section: Camera calibration is small and normally constant for the complete runtime.
        with self._cache_lock:
            self._latest_camera_info = message

    def _goal_callback(self, goal: ResolveVisualTarget.Goal) -> GoalResponse:
        """Special function: Reject structurally unsafe ROI requests before allocating execution work."""
        # Section: Validate only message-local fields here; sensor availability is reported in the result.
        try:
            scale_and_clip_roi(
                (goal.roi_x_min, goal.roi_y_min, goal.roi_x_max, goal.roi_y_max),
                (int(goal.coordinate_width), int(goal.coordinate_height)),
                (int(goal.coordinate_width), int(goal.coordinate_height)),
            )
            stand_off = float(goal.stand_off_distance)
            if stand_off < 0.0 or not math.isfinite(stand_off):
                raise ValueError("Invalid stand-off distance.")
        except (TypeError, ValueError):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle: Any) -> CancelResponse:
        """Special function: Permit the caller to cancel before navigation receives a target."""
        # Section: Resolution has no irreversible actuation and is always safe to cancel.
        return CancelResponse.ACCEPT

    def _feedback(self, goal_handle: Any, phase: str, valid_ratio: float = 0.0) -> None:
        """Special function: Publish compact phase feedback during depth and TF processing."""
        # Section: Feedback intentionally excludes raw images and model-generated free text.
        feedback = ResolveVisualTarget.Feedback()
        feedback.phase = str(phase)
        feedback.valid_depth_ratio = float(valid_ratio)
        goal_handle.publish_feedback(feedback)

    def _result(
        self,
        result_code: int,
        message: str,
        *,
        target_pose: PoseStamped | None = None,
        approach_pose: PoseStamped | None = None,
        depth_m: float = 0.0,
        valid_ratio: float = 0.0,
    ) -> ResolveVisualTarget.Result:
        """Special function: Build one complete action result for every terminal path."""
        # Section: The success flag is derived from the stable result code rather than caller text.
        result = ResolveVisualTarget.Result()
        result.success = int(result_code) == int(ResolveVisualTarget.Result.SUCCESS)
        result.result_code = int(result_code)
        result.target_pose = target_pose or PoseStamped()
        result.approach_pose = approach_pose or PoseStamped()
        result.depth_m = float(depth_m)
        result.valid_depth_ratio = float(valid_ratio)
        result.message = str(message)
        return result

    def _select_depth(self, requested_stamp: TimeMsg) -> tuple[DepthObservation | None, str]:
        """Special function: Select the cached depth frame corresponding to Qwen's source image."""
        # Section: Copy references under lock so camera callbacks can continue during conversion.
        with self._cache_lock:
            observations = list(self._depth_cache)
        if not observations:
            return None, "No aligned depth image has been received."

        # Section: Zero-stamped callers explicitly use the newest observation with a strict receipt-age check.
        if stamp_is_zero(requested_stamp):
            newest = observations[-1]
            age = time.monotonic() - newest.received_monotonic
            maximum_age = float(self.get_parameter("max_latest_depth_age_sec").value)
            if age > maximum_age:
                return None, f"Latest aligned depth image is stale ({age:.3f}s)."
            return newest, ""

        # Section: Timestamped grounding selects the nearest retained depth frame rather than the latest frame.
        requested_sec = stamp_to_seconds(requested_stamp)
        candidates = [observation for observation in observations if observation.stamp_sec > 0.0]
        if not candidates:
            return None, "Cached aligned depth images do not contain ROS timestamps."
        selected = min(candidates, key=lambda observation: abs(observation.stamp_sec - requested_sec))
        skew = abs(selected.stamp_sec - requested_sec)
        maximum_skew = float(self.get_parameter("max_image_depth_skew_sec").value)
        if skew > maximum_skew:
            return None, f"Nearest depth frame differs from the Qwen image by {skew:.3f}s."

        # Section: Bound semantic staleness even when an exact historical frame remains cached.
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        age = max(0.0, now_sec - selected.stamp_sec)
        maximum_age = float(self.get_parameter("max_observation_age_sec").value)
        if age > maximum_age:
            return None, f"Qwen observation is stale ({age:.3f}s)."
        return selected, ""

    def _mask_from_goal(self, goal: ResolveVisualTarget.Goal) -> tuple[np.ndarray | None, str]:
        """Special function: Decode the optional segmentation mask carried with the grounding result."""
        # Section: An empty Image means the caller supplied a bounding box without segmentation.
        mask_message = goal.segmentation_mask
        if int(mask_message.height) <= 0 or int(mask_message.width) <= 0 or not mask_message.data:
            if bool(goal.require_mask):
                return None, "A segmentation mask is required but was not supplied."
            return None, ""
        # Section: Reject masks associated with a different observation timestamp.
        if not stamp_is_zero(goal.image_stamp) and not stamp_is_zero(mask_message.header.stamp):
            mask_skew = abs(stamp_to_seconds(goal.image_stamp) - stamp_to_seconds(mask_message.header.stamp))
            if mask_skew > float(self.get_parameter("max_image_depth_skew_sec").value):
                return None, f"Segmentation mask differs from the Qwen image by {mask_skew:.3f}s."
        try:
            return self._bridge.imgmsg_to_cv2(mask_message, desired_encoding="passthrough"), ""
        except Exception as exc:
            return None, f"Segmentation mask conversion failed: {exc}"

    def _execute(self, goal_handle: Any) -> ResolveVisualTarget.Result:
        """Special function: Resolve one grounded ROI without directly commanding robot motion."""
        # Section: Cancellation before sensor access produces a distinct non-motion outcome.
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(ResolveVisualTarget.Result.CANCELED, "Visual target resolution canceled.")
        self._feedback(goal_handle, "SELECT_DEPTH")

        # Section: Select the depth frame tied to the exact RGB observation seen by Qwen.
        depth_observation, depth_error = self._select_depth(goal_handle.request.image_stamp)
        if depth_observation is None:
            goal_handle.abort()
            return self._result(ResolveVisualTarget.Result.DEPTH_UNAVAILABLE, depth_error)
        with self._cache_lock:
            camera_info = self._latest_camera_info
        if camera_info is None or len(camera_info.k) != 9 or float(camera_info.k[0]) <= 0.0:
            goal_handle.abort()
            return self._result(
                ResolveVisualTarget.Result.CALIBRATION_UNAVAILABLE,
                "Calibrated CameraInfo is unavailable; uncalibrated external-camera data is rejected.",
            )

        # Section: Decode the depth snapshot and map the Qwen coordinate space into depth pixels.
        try:
            depth_native = self._bridge.imgmsg_to_cv2(depth_observation.message, desired_encoding="passthrough")
            depth_m = depth_to_meters(depth_native, depth_observation.message.encoding)
            image_height, image_width = depth_m.shape[:2]
            scaled_roi = scale_and_clip_roi(
                (
                    goal_handle.request.roi_x_min,
                    goal_handle.request.roi_y_min,
                    goal_handle.request.roi_x_max,
                    goal_handle.request.roi_y_max,
                ),
                (int(goal_handle.request.coordinate_width), int(goal_handle.request.coordinate_height)),
                (int(image_width), int(image_height)),
            )
        except Exception as exc:
            goal_handle.abort()
            return self._result(ResolveVisualTarget.Result.INVALID_ROI, f"Depth or ROI conversion failed: {exc}")

        # Section: Decode and align Qwen/SAM segmentation before sampling any target depth.
        mask_native, mask_error = self._mask_from_goal(goal_handle.request)
        if mask_error:
            goal_handle.abort()
            return self._result(ResolveVisualTarget.Result.MASK_UNAVAILABLE, mask_error)
        try:
            mask = prepare_mask(mask_native, (image_width, image_height))
        except ValueError as exc:
            goal_handle.abort()
            return self._result(ResolveVisualTarget.Result.MASK_UNAVAILABLE, str(exc))

        # Section: Robust depth quality gates prevent Qwen's ROI from becoming an unchecked map target.
        self._feedback(goal_handle, "SAMPLE_DEPTH")
        try:
            sample = sample_roi_depth(
                depth_m,
                scaled_roi,
                mask,
                float(self.get_parameter("min_depth_m").value),
                float(self.get_parameter("max_depth_m").value),
                float(self.get_parameter("min_valid_depth_ratio").value),
                int(self.get_parameter("min_valid_depth_pixels").value),
                float(self.get_parameter("bbox_inset_ratio").value),
                float(self.get_parameter("depth_outlier_distance_m").value),
            )
            scaled_camera_matrix = scale_camera_matrix(
                list(camera_info.k),
                (int(camera_info.width), int(camera_info.height)),
                (int(image_width), int(image_height)),
            )
            camera_xyz = project_pixel_to_camera(
                sample.pixel_u,
                sample.pixel_v,
                sample.depth_m,
                scaled_camera_matrix,
            )
        except ValueError as exc:
            goal_handle.abort()
            return self._result(ResolveVisualTarget.Result.DEPTH_QUALITY_FAILED, str(exc))
        self._feedback(goal_handle, "TRANSFORM_TARGET", sample.valid_ratio)

        # Section: The aligned depth header owns the observation timestamp and optical frame.
        source_frame = str(depth_observation.message.header.frame_id or camera_info.header.frame_id).strip()
        if not source_frame:
            goal_handle.abort()
            return self._result(ResolveVisualTarget.Result.TF_FAILED, "Depth and CameraInfo frame IDs are empty.")
        camera_point = PointStamped()
        camera_point.header.stamp = depth_observation.message.header.stamp
        camera_point.header.frame_id = source_frame
        camera_point.point.x, camera_point.point.y, camera_point.point.z = camera_xyz

        # Section: Transform the measured target and current base origin independently into map coordinates.
        map_frame = str(self.get_parameter("map_frame").value).strip().lstrip("/")
        base_frame = str(self.get_parameter("base_frame").value).strip().lstrip("/")
        tf_timeout = Duration(seconds=max(0.01, float(self.get_parameter("tf_timeout_sec").value)))
        try:
            map_point = self._tf_buffer.transform(camera_point, map_frame, timeout=tf_timeout)
            base_transform = self._tf_buffer.lookup_transform(map_frame, base_frame, Time(), timeout=tf_timeout)
        except TransformException as exc:
            goal_handle.abort()
            return self._result(ResolveVisualTarget.Result.TF_FAILED, f"Target/base TF resolution failed: {exc}")

        # Section: Compute a planar stand-off from the current base position and face the selected entity.
        stand_off = float(goal_handle.request.stand_off_distance)
        if stand_off <= 0.0:
            stand_off = float(self.get_parameter("default_stand_off_distance").value)
        try:
            approach_x, approach_y, approach_yaw = compute_standoff_geometry(
                (
                    float(base_transform.transform.translation.x),
                    float(base_transform.transform.translation.y),
                ),
                (float(map_point.point.x), float(map_point.point.y)),
                stand_off,
            )
        except ValueError as exc:
            goal_handle.abort()
            return self._result(ResolveVisualTarget.Result.TF_FAILED, str(exc))

        # Section: Preserve the observed 3D entity pose separately from the planar base goal.
        now_stamp = self.get_clock().now().to_msg()
        target_pose = PoseStamped()
        target_pose.header.frame_id = map_frame
        target_pose.header.stamp = map_point.header.stamp
        target_pose.pose.position = map_point.point
        target_pose.pose.orientation.w = 1.0
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = map_frame
        approach_pose.header.stamp = now_stamp
        approach_pose.pose.position.x = float(approach_x)
        approach_pose.pose.position.y = float(approach_y)
        approach_pose.pose.position.z = 0.0
        quaternion = yaw_to_quaternion(approach_yaw)
        approach_pose.pose.orientation.x = quaternion[0]
        approach_pose.pose.orientation.y = quaternion[1]
        approach_pose.pose.orientation.z = quaternion[2]
        approach_pose.pose.orientation.w = quaternion[3]

        # Section: Publish inspectable poses before reporting success to the queue-producing caller.
        self._target_pose_pub.publish(target_pose)
        self._approach_pose_pub.publish(approach_pose)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(ResolveVisualTarget.Result.CANCELED, "Visual target resolution canceled.")
        self._feedback(goal_handle, "RESOLVED", sample.valid_ratio)
        goal_handle.succeed()
        return self._result(
            ResolveVisualTarget.Result.SUCCESS,
            f"Resolved target '{goal_handle.request.target_id}' to a map-frame stand-off pose.",
            target_pose=target_pose,
            approach_pose=approach_pose,
            depth_m=sample.depth_m,
            valid_ratio=sample.valid_ratio,
        )


def main(args: list[str] | None = None) -> None:
    """Special function: Run camera subscriptions and action work on separate executor threads."""
    # Section: Multiple threads allow depth callbacks to continue while an action resolves TF.
    rclpy.init(args=args)
    node = QwenRoiDepthResolver()
    executor = MultiThreadedExecutor(num_threads=3)
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
