#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
# flake8: noqa: E501
"""ROS 2 semantic target, segmentation, RGB-D mesh, and grasp-geometry pipeline."""

from __future__ import annotations

# Section: Standard-library primitives provide explicit state, immutable observations, and one worker.
from dataclasses import dataclass
from enum import Enum
import json
import math
from pathlib import Path
import threading
import time
from typing import Any

# Section: Image and numeric dependencies support mask, overlay, and geometry conversion.
import cv2
import numpy as np

# Section: ROS interfaces expose perception products only; no motion client is imported.
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray
from vision.msg import RoiResult

# Section: Importing tf2_geometry_msgs registers PoseStamped transformations with tf2_ros.
import tf2_geometry_msgs  # noqa: F401

# Section: Typed pure helpers enforce stage boundaries from semantic grounding to visualization.
from target_geometry.core import (
    OrganizedSurface,
    TargetMesh,
    build_organized_mesh,
    compute_oriented_bounding_box,
    depth_to_meters,
    project_masked_depth,
    reject_local_depth_outliers,
    reject_support_plane,
    resize_mask_nearest,
    scale_camera_matrix,
    select_mask_component,
    select_surface_grasp,
    validate_binary_mask,
    voxel_downsample,
)
from target_geometry.grounding import GroundedTarget, GroundingError, parse_qwen_grounding
from target_geometry.markers import make_delete_all_markers, make_grasp_pose, make_target_markers


class TargetState(str, Enum):
    """Enumerate externally visible perception pipeline states."""

    NO_TARGET = "NO_TARGET"
    TARGET_REQUESTED = "TARGET_REQUESTED"
    GROUNDING = "GROUNDING"
    SEGMENTING = "SEGMENTING"
    DEPTH_PROJECTING = "DEPTH_PROJECTING"
    MESH_RECONSTRUCTION = "MESH_RECONSTRUCTION"
    GEOMETRY_ANALYSIS = "GEOMETRY_ANALYSIS"
    TARGET_READY = "TARGET_READY"
    TARGET_LOST = "TARGET_LOST"
    FAILED = "FAILED"


@dataclass(frozen=True)
class ReceivedMessage:
    """Pair one ROS sensor message with a monotonic receipt time."""

    message: Any
    received_monotonic: float


@dataclass(frozen=True)
class RgbdBundle:
    """Carry one validated RGB, aligned-depth, and calibration combination."""

    rgb_message: Image
    depth_message: Image
    camera_info: CameraInfo


@dataclass(frozen=True)
class SegmentationResult:
    """Return one genuine model mask with updated tracking hints."""

    mask: np.ndarray
    bbox_xyxy: tuple[int, int, int, int]
    point_uv: tuple[float, float]
    confidence: float
    label: str


def stamp_seconds(stamp: Any) -> float:
    """Convert a ROS stamp-like value into floating-point seconds."""
    # Section: Tests and callbacks use the same explicit conversion without consulting a ROS clock.
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1.0e-9


def bbox_iou(
    first: tuple[int, int, int, int] | None,
    second: tuple[int, int, int, int],
) -> float:
    """Score overlap between an optional grounded box and a segmentation box."""
    # Section: A missing grounding box contributes no overlap but does not invalidate segmentation.
    if first is None:
        return 0.0
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    first_area = max(0, first[2] - first[0]) * max(0, first[3] - first[1])
    second_area = max(0, second[2] - second[0]) * max(0, second[3] - second[1])
    union = first_area + second_area - intersection
    return float(intersection) / float(union) if union > 0 else 0.0


class TargetSegmenter:
    """Define the replaceable pixel-segmentation boundary."""

    def segment(
        self,
        image: np.ndarray,
        *,
        bbox: tuple[int, int, int, int] | None = None,
        point: tuple[float, float] | None = None,
        label: str | None = None,
    ) -> SegmentationResult:
        """Return a real binary mask for one semantic target."""
        raise NotImplementedError


class YoloETargetSegmenter(TargetSegmenter):
    """Produce target pixels using the existing open-vocabulary YOLOE-seg weights."""

    def __init__(
        self,
        *,
        model_path: str,
        device: str,
        confidence_threshold: float,
        image_size: int,
        logger: Any,
    ) -> None:
        """Store segmentation settings and defer GPU allocation until requested."""
        # Section: Lazy loading avoids consuming segmentation resources before any target exists.
        self.model_path = str(Path(model_path).expanduser().resolve())
        self.device = str(device)
        self.confidence_threshold = float(confidence_threshold)
        self.image_size = int(image_size)
        self.logger = logger
        self.model: Any = None
        self.configured_label = ""

    def load(self) -> None:
        """Load the installed YOLOE segmentation model once."""
        # Section: The model must genuinely expose segmentation masks for this pipeline.
        if self.model is not None:
            return
        from ultralytics import YOLO

        if not Path(self.model_path).is_file():
            raise FileNotFoundError(f"YOLOE segmentation weights do not exist: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.logger.info(f"Loaded target segmenter: {self.model_path}")

    def _configure_label(self, label: str) -> None:
        """Configure one semantic class for open-vocabulary YOLOE segmentation."""
        # Section: Recompute text embeddings only when the requested semantic class changes.
        normalized = str(label or "").strip().casefold()
        if not normalized:
            raise ValueError("segmentation label is empty")
        if normalized == self.configured_label:
            return
        set_classes = getattr(self.model, "set_classes", None)
        if not callable(set_classes):
            raise RuntimeError("configured YOLO model does not support open-vocabulary classes")
        set_classes([normalized])
        self.configured_label = normalized

    def segment(
        self,
        image: np.ndarray,
        *,
        bbox: tuple[int, int, int, int] | None = None,
        point: tuple[float, float] | None = None,
        label: str | None = None,
    ) -> SegmentationResult:
        """Select the best actual YOLOE mask using Qwen box and point hints."""
        # Section: Qwen identifies meaning while YOLOE alone determines target pixel membership.
        self.load()
        self._configure_label(str(label or ""))
        # Section: Ultralytics accepts OpenCV arrays in BGR order while Qwen receives RGB.
        model_image = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
        results = self.model.predict(
            source=model_image,
            imgsz=self.image_size,
            conf=self.confidence_threshold,
            device=self.device,
            verbose=False,
        )
        if not results or results[0].boxes is None or results[0].masks is None:
            raise ValueError(f"segmentation backend found no mask for '{label}'")
        boxes = results[0].boxes
        mask_tensor = results[0].masks.data
        if len(boxes) == 0 or len(mask_tensor) == 0:
            raise ValueError(f"segmentation backend found no mask for '{label}'")

        # Section: Score each genuine instance mask by detector confidence and grounding consistency.
        height, width = image.shape[:2]
        candidates: list[tuple[float, int, tuple[int, int, int, int], float]] = []
        for index in range(min(len(boxes), len(mask_tensor))):
            box = boxes[index]
            coordinates = box.xyxy[0].detach().cpu().numpy().tolist()
            left = max(0, min(width - 1, int(math.floor(coordinates[0]))))
            top = max(0, min(height - 1, int(math.floor(coordinates[1]))))
            right = max(left + 1, min(width, int(math.ceil(coordinates[2]))))
            bottom = max(top + 1, min(height, int(math.ceil(coordinates[3]))))
            candidate_box = (left, top, right, bottom)
            confidence = float(box.conf[0].item())
            point_bonus = 0.0
            if point is not None:
                point_bonus = 1.0 if (
                    candidate_box[0] <= point[0] < candidate_box[2]
                    and candidate_box[1] <= point[1] < candidate_box[3]
                ) else 0.0
            score = 0.50 * confidence + 0.35 * bbox_iou(bbox, candidate_box) + 0.15 * point_bonus
            candidates.append((score, index, candidate_box, confidence))
        if not candidates:
            raise ValueError(f"segmentation backend returned no usable instance for '{label}'")
        _score, selected_index, selected_box, selected_confidence = max(candidates, key=lambda item: item[0])

        # Section: Resize network mask logits with nearest-neighbour interpolation and threshold once.
        mask = mask_tensor[selected_index].detach().cpu().numpy()
        if mask.shape != (height, width):
            mask = cv2.resize(mask.astype(np.float32), (width, height), interpolation=cv2.INTER_NEAREST)
        binary = mask > 0.5
        if not np.any(binary):
            raise ValueError("selected segmentation mask is empty")
        moments = cv2.moments(binary.astype(np.uint8), binaryImage=True)
        if moments["m00"] > 0.0:
            center = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        else:
            center = (0.5 * (selected_box[0] + selected_box[2]), 0.5 * (selected_box[1] + selected_box[3]))
        return SegmentationResult(
            mask=binary,
            bbox_xyxy=selected_box,
            point_uv=(float(center[0]), float(center[1])),
            confidence=selected_confidence,
            label=str(label),
        )


class QwenImageGrounder:
    """Invoke Qwen only for semantic target acquisition or reacquisition."""

    def __init__(
        self,
        *,
        model_path: str,
        prompt_path: str,
        max_new_tokens: int,
        resized_image_size: int,
        minimum_confidence: float,
        minimum_bbox_fraction: float,
        maximum_bbox_fraction: float,
        minimum_instance_confidence_margin: float,
        logger: Any,
    ) -> None:
        """Store local model policy without blocking camera callbacks."""
        # Section: The node's single worker owns every heavyweight model operation.
        self.model_path = str(Path(model_path).expanduser().resolve())
        self.prompt_path = str(Path(prompt_path).expanduser().resolve())
        self.max_new_tokens = max(32, int(max_new_tokens))
        self.resized_image_size = max(224, int(resized_image_size))
        self.minimum_confidence = float(minimum_confidence)
        self.minimum_bbox_fraction = float(minimum_bbox_fraction)
        self.maximum_bbox_fraction = float(maximum_bbox_fraction)
        self.minimum_instance_confidence_margin = float(minimum_instance_confidence_margin)
        self.logger = logger
        self.processor: Any = None
        self.model: Any = None
        self.torch: Any = None
        self.prompt = ""

    def load(self) -> None:
        """Load the local Qwen processor, prompt, and 4-bit weights once."""
        # Section: Validate local assets before allocating CUDA memory.
        if self.model is not None:
            return
        model_path = Path(self.model_path)
        prompt_path = Path(self.prompt_path)
        if not model_path.exists():
            raise FileNotFoundError(f"Qwen model path does not exist: {model_path}")
        if not prompt_path.is_file():
            raise FileNotFoundError(f"Qwen grounding prompt does not exist: {prompt_path}")
        self.prompt = prompt_path.read_text(encoding="utf-8").strip()
        if not self.prompt:
            raise ValueError("Qwen grounding prompt is empty")

        # Section: Imports remain lazy so hardware-independent geometry tests need no ML runtime.
        import torch
        from transformers import (
            AutoProcessor,
            BitsAndBytesConfig,
            Qwen3VLForConditionalGeneration,
        )

        quantization = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16,
            bnb_4bit_use_double_quant=True,
        )
        self.processor = AutoProcessor.from_pretrained(self.model_path, local_files_only=True)
        self.model = Qwen3VLForConditionalGeneration.from_pretrained(
            self.model_path,
            quantization_config=quantization,
            device_map="auto",
            low_cpu_mem_usage=True,
            local_files_only=True,
        )
        # Section: Disable training behavior because grounding is inference-only.
        self.model.eval()
        self.torch = torch
        self.logger.info("Loaded UnifoLM-ER-1 in NF4 for on-demand target grounding.")

    def ground(self, image_rgb: np.ndarray, requested_target: str) -> GroundedTarget:
        """Ground one semantic target in exactly one RGB observation."""
        # Section: The request text explicitly binds Qwen to semantic selection, not pixel segmentation.
        self.load()
        from PIL import Image as PilImage

        height, width = image_rgb.shape[:2]
        request_prompt = (
            f"{self.prompt}\n\nRequested target: {requested_target}\n"
            "Select only that requested physical object. Return JSON only."
        )
        messages = [{
            "role": "user",
            "content": [
                {
                    "type": "image",
                    "image": PilImage.fromarray(image_rgb),
                    "resized_height": self.resized_image_size,
                    "resized_width": self.resized_image_size,
                },
                {"type": "text", "text": request_prompt},
            ],
        }]
        inputs = self.processor.apply_chat_template(
            messages,
            tokenize=True,
            add_generation_prompt=True,
            return_dict=True,
            return_tensors="pt",
        )
        inputs = inputs.to(self.model.device)
        with self.torch.inference_mode():
            output_ids = self.model.generate(
                **inputs,
                max_new_tokens=self.max_new_tokens,
                do_sample=False,
            )
        generated_ids = output_ids[:, inputs["input_ids"].shape[1] :]
        raw_response = self.processor.batch_decode(
            generated_ids,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )[0].strip()
        return parse_qwen_grounding(
            raw_response,
            requested_target=requested_target,
            image_width=width,
            image_height=height,
            minimum_confidence=self.minimum_confidence,
            minimum_bbox_fraction=self.minimum_bbox_fraction,
            maximum_bbox_fraction=self.maximum_bbox_fraction,
            minimum_instance_confidence_margin=self.minimum_instance_confidence_margin,
        )


class TargetGraspGeometryNode(Node):
    """Produce target masks, polygon meshes, and candidate grasp geometry only."""

    def __init__(self) -> None:
        """Configure topics, parameters, latest-frame storage, TF, and worker."""
        super().__init__("target_grasp_geometry")

        # Section: Reuse the managed RealSense aligned RGB-D pipeline and calibrated optical frame.
        self.declare_parameter("target_topic", "/vision/target_object")
        # Section: A shared ER-Flow node supplies the semantic ROI without a second Qwen allocation.
        self.declare_parameter("roi_topic", "/vision/roi/result")
        self.declare_parameter("external_roi_timeout_sec", 1.5)
        # Section: Require the shared ROI and current RGB frame to describe the same acquisition time.
        self.declare_parameter("max_external_roi_rgb_time_difference", 0.05)
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("mask_topic", "/vision/target_mask")
        self.declare_parameter("overlay_topic", "/vision/target_overlay")
        self.declare_parameter("grasp_pose_topic", "/vision/grasp_pose")
        self.declare_parameter("markers_topic", "/vision/target_markers")
        self.declare_parameter("status_topic", "/vision/target_status")
        self.declare_parameter("debug_cloud_topic", "/vision/debug_target_cloud")

        # Section: Qwen performs semantic acquisition only on new or explicitly reacquired targets.
        default_prompt = str(Path(get_package_share_directory("vision")) / "config" / "qwen_grounding_prompt.txt")
        self.declare_parameter("qwen_enabled", True)
        self.declare_parameter("qwen_model_path", "/home/yanjiun68/ai/models/UnifoLM-ER-1")
        self.declare_parameter("qwen_prompt_path", default_prompt)
        self.declare_parameter("qwen_max_new_tokens", 192)
        self.declare_parameter("qwen_image_size", 448)
        self.declare_parameter("qwen_minimum_confidence", 0.35)
        self.declare_parameter("qwen_instance_confidence_margin", 0.05)
        self.declare_parameter("minimum_bbox_fraction", 0.0005)
        self.declare_parameter("maximum_bbox_fraction", 0.70)

        # Section: YOLOE supplies genuine instance pixels and remains replaceable behind TargetSegmenter.
        self.declare_parameter("segmentation_backend", "yoloe")
        self.declare_parameter("segmentation_model_path", "/home/yanjiun68/workspace/WS1/PutrabotUPM-project/yoloe-26s-seg.pt")
        self.declare_parameter("segmentation_device", "cuda")
        self.declare_parameter("segmentation_confidence", 0.25)
        self.declare_parameter("segmentation_image_size", 640)
        self.declare_parameter("minimum_mask_pixels", 100)
        self.declare_parameter("maximum_mask_fraction", 0.70)

        # Section: Timestamp and metric-depth policy prevents unrelated RGB-D combinations.
        self.declare_parameter("max_rgb_depth_time_difference", 0.08)
        self.declare_parameter("max_frame_age", 0.50)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 2.0)
        self.declare_parameter("minimum_valid_depth_ratio", 0.40)

        # Section: Conservative cleanup retains object boundaries while rejecting isolated measurements.
        self.declare_parameter("voxel_leaf_size", 0.005)
        self.declare_parameter("enable_outlier_filter", True)
        self.declare_parameter("outlier_kernel_size", 5)
        self.declare_parameter("outlier_maximum_deviation_m", 0.025)
        self.declare_parameter("enable_plane_removal", False)
        self.declare_parameter("plane_distance_threshold_m", 0.008)
        self.declare_parameter("plane_minimum_samples", 30)
        self.declare_parameter("plane_minimum_inlier_fraction", 0.05)
        self.declare_parameter("plane_maximum_inlier_fraction", 0.45)
        self.declare_parameter("plane_ransac_iterations", 80)
        self.declare_parameter("enable_clustering", True)

        # Section: Organized triangulation parameters control discontinuities and transport complexity.
        self.declare_parameter("mesh_update_rate_hz", 5.0)
        self.declare_parameter("mesh_pixel_stride", 2)
        self.declare_parameter("mesh_max_edge_length", 0.03)
        self.declare_parameter("mesh_max_depth_jump", 0.02)
        self.declare_parameter("mesh_min_triangle_area", 1.0e-7)
        self.declare_parameter("mesh_max_triangle_area", 5.0e-4)
        self.declare_parameter("mesh_max_triangles", 12000)
        self.declare_parameter("normal_search_radius", 0.02)

        # Section: Grasp and visualization controls remain independent of any arm controller.
        self.declare_parameter("grasp_boundary_margin_pixels", 4.0)
        self.declare_parameter("grasp_lower_boundary_rejection_fraction", 0.12)
        self.declare_parameter("grasp_minimum_axis_projection", 0.25)
        self.declare_parameter("grasp_marker_size", 0.03)
        self.declare_parameter("approach_marker_length", 0.12)
        self.declare_parameter("closing_axis_length", 0.10)
        self.declare_parameter("output_grasp_frame", "")
        self.declare_parameter("tf_timeout_sec", 0.25)
        self.declare_parameter("publish_debug_overlay", True)
        self.declare_parameter("publish_debug_cloud", False)
        self.declare_parameter("target_lost_frame_count", 3)

        # Section: Sensor callbacks only replace latest bounded observations under a short lock.
        self._bridge = CvBridge()
        self._sensor_lock = threading.Lock()
        self._latest_rgb: ReceivedMessage | None = None
        self._latest_depth: ReceivedMessage | None = None
        self._latest_camera_info: ReceivedMessage | None = None
        self._latest_roi: ReceivedMessage | None = None

        # Section: Target generations cancel obsolete worker sessions without unbounded queues.
        self._target_lock = threading.Lock()
        self._target = ""
        self._target_generation = 0
        self._target_event = threading.Event()
        self._stop_event = threading.Event()
        self._last_frame_id = "camera_color_optical_frame"
        self._last_stamp: Any = None
        self._last_rgb_shape: tuple[int, int] | None = None

        # Section: TF is optional and only transforms the final grasp pose when configured.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Section: Camera subscriptions use sensor-data QoS while commands and results remain reliable.
        self._target_sub = self.create_subscription(
            String,
            str(self.get_parameter("target_topic").value),
            self._target_callback,
            10,
        )
        self._rgb_sub = self.create_subscription(
            Image,
            str(self.get_parameter("rgb_topic").value),
            self._rgb_callback,
            qos_profile_sensor_data,
        )
        self._depth_sub = self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self._roi_sub = self.create_subscription(
            RoiResult,
            str(self.get_parameter("roi_topic").value),
            self._roi_callback,
            10,
        )

        # Section: Required outputs always exist; optional debug publishers remain inexpensive when unused.
        self._mask_pub = self.create_publisher(Image, str(self.get_parameter("mask_topic").value), 10)
        self._overlay_pub = self.create_publisher(Image, str(self.get_parameter("overlay_topic").value), 10)
        self._grasp_pose_pub = self.create_publisher(PoseStamped, str(self.get_parameter("grasp_pose_topic").value), 10)
        self._markers_pub = self.create_publisher(MarkerArray, str(self.get_parameter("markers_topic").value), 10)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self._debug_cloud_pub = self.create_publisher(PointCloud2, str(self.get_parameter("debug_cloud_topic").value), 10)

        # Section: Construct lazy model backends before starting the single processing worker.
        self._grounder = QwenImageGrounder(
            model_path=str(self.get_parameter("qwen_model_path").value),
            prompt_path=str(self.get_parameter("qwen_prompt_path").value),
            max_new_tokens=int(self.get_parameter("qwen_max_new_tokens").value),
            resized_image_size=int(self.get_parameter("qwen_image_size").value),
            minimum_confidence=float(self.get_parameter("qwen_minimum_confidence").value),
            minimum_bbox_fraction=float(self.get_parameter("minimum_bbox_fraction").value),
            maximum_bbox_fraction=float(self.get_parameter("maximum_bbox_fraction").value),
            minimum_instance_confidence_margin=float(
                self.get_parameter("qwen_instance_confidence_margin").value
            ),
            logger=self.get_logger(),
        )
        backend = str(self.get_parameter("segmentation_backend").value).strip().casefold()
        if backend != "yoloe":
            raise ValueError(f"unsupported segmentation backend: {backend}")
        self._segmenter: TargetSegmenter = YoloETargetSegmenter(
            model_path=str(self.get_parameter("segmentation_model_path").value),
            device=str(self.get_parameter("segmentation_device").value),
            confidence_threshold=float(self.get_parameter("segmentation_confidence").value),
            image_size=int(self.get_parameter("segmentation_image_size").value),
            logger=self.get_logger(),
        )
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._publish_status(TargetState.NO_TARGET, "waiting for /vision/target_object")

    def _rgb_callback(self, message: Image) -> None:
        """Retain only the newest RGB observation."""
        # Section: Model inference never runs inside this camera callback.
        with self._sensor_lock:
            self._latest_rgb = ReceivedMessage(message, time.monotonic())

    def _depth_callback(self, message: Image) -> None:
        """Retain only the newest aligned-depth observation."""
        # Section: A latest-frame policy avoids processing obsolete sensor queues.
        with self._sensor_lock:
            self._latest_depth = ReceivedMessage(message, time.monotonic())

    def _camera_info_callback(self, message: CameraInfo) -> None:
        """Retain the latest calibrated color intrinsics."""
        # Section: CameraInfo is small and normally constant for the complete sensor runtime.
        with self._sensor_lock:
            self._latest_camera_info = ReceivedMessage(message, time.monotonic())

    def _roi_callback(self, message: RoiResult) -> None:
        """Special function: Retain only the newest typed ROI from the shared ER-Flow tracker."""
        # Section: The worker validates status, identity, dimensions, and freshness before using the box.
        with self._sensor_lock:
            self._latest_roi = ReceivedMessage(message, time.monotonic())

    def _external_grounding(
        self,
        target: str,
        image_width: int,
        image_height: int,
        rgb_stamp: Any,
    ) -> GroundedTarget | None:
        """Special function: Convert one fresh valid shared ROI into the grasp pipeline's grounding type."""
        # Section: Copy the small message reference under the same bounded sensor-state lock.
        with self._sensor_lock:
            received = self._latest_roi
        if received is None:
            return None
        timeout_sec = max(0.05, float(self.get_parameter("external_roi_timeout_sec").value))
        if time.monotonic() - received.received_monotonic > timeout_sec:
            return None
        roi_message = received.message
        normalized_target = " ".join(str(target).casefold().split())
        normalized_result_target = " ".join(str(roi_message.target_id).casefold().split())
        if normalized_result_target != normalized_target:
            return None
        if str(roi_message.status).casefold() != "ok" or not bool(roi_message.tracking_valid):
            return None
        # Section: Receipt freshness is insufficient; reject a valid but different-scene ROI before pixel scaling.
        roi_stamp_sec = stamp_seconds(roi_message.header.stamp)
        rgb_stamp_sec = stamp_seconds(rgb_stamp)
        maximum_roi_rgb_skew = float(
            self.get_parameter("max_external_roi_rgb_time_difference").value
        )
        if (
            roi_stamp_sec <= 0.0
            or rgb_stamp_sec <= 0.0
            or abs(roi_stamp_sec - rgb_stamp_sec) > maximum_roi_rgb_skew
        ):
            return None
        source_width = int(roi_message.image_width)
        source_height = int(roi_message.image_height)
        if min(source_width, source_height, image_width, image_height) <= 0:
            return None
        left = int(math.floor(float(roi_message.roi.x_offset) * image_width / source_width))
        top = int(math.floor(float(roi_message.roi.y_offset) * image_height / source_height))
        right = int(math.ceil(float(roi_message.roi.x_offset + roi_message.roi.width) * image_width / source_width))
        bottom = int(math.ceil(float(roi_message.roi.y_offset + roi_message.roi.height) * image_height / source_height))
        left = max(0, min(image_width - 1, left))
        top = max(0, min(image_height - 1, top))
        right = max(left + 1, min(image_width, right))
        bottom = max(top + 1, min(image_height, bottom))
        confidence = float(roi_message.confidence)
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            return None
        return GroundedTarget(
            label=str(target),
            confidence=confidence,
            bbox_xyxy=(left, top, right, bottom),
            point_uv=(0.5 * float(left + right), 0.5 * float(top + bottom)),
        )

    def _target_callback(self, message: String) -> None:
        """Replace the active target and invalidate all stale visualization."""
        # Section: An empty command explicitly clears the active target.
        target = str(message.data or "").strip()
        with self._target_lock:
            self._target_generation += 1
            self._target = target
            generation = self._target_generation
        self._clear_outputs("target changed")
        if target:
            self._publish_status(TargetState.TARGET_REQUESTED, "target accepted", target=target, generation=generation)
        else:
            self._publish_status(TargetState.NO_TARGET, "target cleared", generation=generation)
        self._target_event.set()

    def _current_target(self) -> tuple[str, int]:
        """Read the active immutable target-generation pair."""
        # Section: The short lock does not overlap image conversion or inference.
        with self._target_lock:
            return self._target, self._target_generation

    def _snapshot_rgbd(self) -> RgbdBundle:
        """Select one fresh, timestamp-correlated RGB-D-calibration bundle."""
        # Section: Copy message references atomically, then perform validation without blocking callbacks.
        with self._sensor_lock:
            rgb = self._latest_rgb
            depth = self._latest_depth
            camera_info = self._latest_camera_info
        if rgb is None or depth is None or camera_info is None:
            raise RuntimeError("RGB, aligned depth, or CameraInfo has not been received")
        now = time.monotonic()
        max_age = max(0.0, float(self.get_parameter("max_frame_age").value))
        if now - rgb.received_monotonic > max_age:
            raise RuntimeError("latest RGB frame is stale")
        if now - depth.received_monotonic > max_age:
            raise RuntimeError("latest aligned depth frame is stale")
        if now - camera_info.received_monotonic > max_age:
            raise RuntimeError("latest CameraInfo is stale")
        rgb_stamp = stamp_seconds(rgb.message.header.stamp)
        depth_stamp = stamp_seconds(depth.message.header.stamp)
        info_stamp = stamp_seconds(camera_info.message.header.stamp)
        if rgb_stamp <= 0.0 or depth_stamp <= 0.0 or info_stamp <= 0.0:
            raise RuntimeError("RGB, aligned depth, and CameraInfo require non-zero timestamps")
        skew = abs(rgb_stamp - depth_stamp)
        maximum_skew = float(self.get_parameter("max_rgb_depth_time_difference").value)
        if skew > maximum_skew:
            raise RuntimeError(f"RGB/depth timestamp difference is too large: {skew:.3f}s")
        info_skew = abs(rgb_stamp - info_stamp)
        if info_skew > maximum_skew:
            raise RuntimeError(
                f"RGB/CameraInfo timestamp difference is too large: {info_skew:.3f}s"
            )
        if len(camera_info.message.k) != 9 or float(camera_info.message.k[0]) <= 0.0:
            raise RuntimeError("calibrated CameraInfo is unavailable")
        # Section: Aligned depth must use color-camera calibration in the same optical frame.
        rgb_frame = str(rgb.message.header.frame_id).strip().lstrip("/")
        depth_frame = str(depth.message.header.frame_id).strip().lstrip("/")
        info_frame = str(camera_info.message.header.frame_id).strip().lstrip("/")
        if not rgb_frame or not depth_frame or not info_frame:
            raise RuntimeError("RGB, aligned depth, and CameraInfo require frame IDs")
        if rgb_frame != info_frame or depth_frame != info_frame:
            raise RuntimeError(
                "RGB, aligned depth, and CameraInfo are not expressed in the same optical frame"
            )
        return RgbdBundle(rgb.message, depth.message, camera_info.message)

    def _worker_loop(self) -> None:
        """Process one active target session without blocking sensor callbacks."""
        # Section: The event wakes only for a new target; each session then updates at a bounded rate.
        while not self._stop_event.is_set():
            self._target_event.wait(timeout=0.25)
            if self._stop_event.is_set():
                break
            if not self._target_event.is_set():
                continue
            self._target_event.clear()
            target, generation = self._current_target()
            if not target:
                continue
            self._run_target_session(target, generation)

    def _run_target_session(self, target: str, generation: int) -> None:
        """Ground once, then update segmentation, mesh, and grasp at a bounded rate."""
        # Section: Qwen may run once more only after confirmed loss during the same target generation.
        grounding: GroundedTarget | None = None
        reacquisition_used = False
        consecutive_failures = 0
        update_rate = max(0.1, float(self.get_parameter("mesh_update_rate_hz").value))
        period = 1.0 / update_rate
        while not self._stop_event.is_set():
            active_target, active_generation = self._current_target()
            if active_generation != generation or active_target != target:
                return
            cycle_started = time.monotonic()
            try:
                bundle = self._snapshot_rgbd()
                rgb = self._bridge.imgmsg_to_cv2(bundle.rgb_message, desired_encoding="rgb8")
                if grounding is None and bool(self.get_parameter("qwen_enabled").value):
                    self._publish_status(TargetState.GROUNDING, "running on-demand semantic grounding", target=target)
                    grounding = self._grounder.ground(rgb, target)
                if not bool(self.get_parameter("qwen_enabled").value):
                    # Section: Refresh the 5 Hz geometry input from a timestamp-matched shared 15 Hz tracker result.
                    grounding = self._external_grounding(
                        target,
                        int(rgb.shape[1]),
                        int(rgb.shape[0]),
                        bundle.rgb_message.header.stamp,
                    )
                    if grounding is None:
                        self._publish_status(
                            TargetState.GROUNDING,
                            "waiting for a fresh valid shared ER-Flow ROI",
                            target=target,
                        )
                        remaining = max(0.0, period - (time.monotonic() - cycle_started))
                        if self._target_event.wait(timeout=remaining):
                            self._target_event.clear()
                            return
                        continue
                grounding = self._process_bundle(target, bundle, rgb, grounding)
                consecutive_failures = 0
            except GroundingError as exc:
                self._clear_outputs(str(exc))
                self._publish_status(TargetState.TARGET_LOST, str(exc), target=target)
                return
            except Exception as exc:
                consecutive_failures += 1
                self._clear_outputs(str(exc))
                self._publish_status(
                    TargetState.FAILED,
                    str(exc),
                    target=target,
                    consecutive_failures=consecutive_failures,
                )
                lost_threshold = max(1, int(self.get_parameter("target_lost_frame_count").value))
                if consecutive_failures >= lost_threshold:
                    if bool(self.get_parameter("qwen_enabled").value) and not reacquisition_used:
                        reacquisition_used = True
                        grounding = None
                        consecutive_failures = 0
                        self._publish_status(TargetState.TARGET_LOST, "target lost; Qwen reacquisition scheduled", target=target)
                    else:
                        self._publish_status(TargetState.TARGET_LOST, "target geometry unavailable", target=target)
                        return
            remaining = max(0.0, period - (time.monotonic() - cycle_started))
            if self._target_event.wait(timeout=remaining):
                self._target_event.clear()
                return

    def _process_bundle(
        self,
        target: str,
        bundle: RgbdBundle,
        rgb: np.ndarray,
        grounding: GroundedTarget | None,
    ) -> GroundedTarget | None:
        """Execute segmentation through grasp visualization for one RGB-D bundle."""
        # Section: Segmentation uses Qwen hints but alone determines target pixel membership.
        self._publish_status(TargetState.SEGMENTING, "segmenting grounded target", target=target)
        segmentation_label = grounding.label if grounding is not None else target
        segmentation = self._segmenter.segment(
            rgb,
            bbox=grounding.bbox_xyxy if grounding is not None else None,
            point=grounding.point_uv if grounding is not None else None,
            label=segmentation_label,
        )
        rgb_height, rgb_width = rgb.shape[:2]
        mask = validate_binary_mask(
            segmentation.mask,
            (rgb_height, rgb_width),
            minimum_pixels=int(self.get_parameter("minimum_mask_pixels").value),
            maximum_fraction=float(self.get_parameter("maximum_mask_fraction").value),
        )
        if bool(self.get_parameter("enable_clustering").value):
            mask = select_mask_component(
                mask,
                seed_point=segmentation.point_uv,
                minimum_pixels=int(self.get_parameter("minimum_mask_pixels").value),
            )

        # Section: Mask publication is mono8 and preserves the exact RGB observation header.
        mask_message = self._bridge.cv2_to_imgmsg((mask.astype(np.uint8) * 255), encoding="mono8")
        mask_message.header = bundle.rgb_message.header
        self._mask_pub.publish(mask_message)
        # Section: Record mask metadata immediately so a later geometry failure can clear it.
        self._last_rgb_shape = (rgb_height, rgb_width)
        self._last_frame_id = str(bundle.rgb_message.header.frame_id)
        self._last_stamp = bundle.rgb_message.header.stamp

        # Section: Aligned depth is converted to metres and projected only where the true mask is non-zero.
        self._publish_status(TargetState.DEPTH_PROJECTING, "extracting segmented metric depth", target=target)
        depth_native = self._bridge.imgmsg_to_cv2(bundle.depth_message, desired_encoding="passthrough")
        depth_m = depth_to_meters(depth_native, bundle.depth_message.encoding)
        depth_height, depth_width = depth_m.shape[:2]
        depth_mask = resize_mask_nearest(mask, (depth_height, depth_width))
        intrinsics = scale_camera_matrix(
            list(bundle.camera_info.k),
            (int(bundle.camera_info.width), int(bundle.camera_info.height)),
            (depth_width, depth_height),
        )
        surface = project_masked_depth(
            depth_m,
            depth_mask,
            intrinsics,
            min_depth_m=float(self.get_parameter("min_depth_m").value),
            max_depth_m=float(self.get_parameter("max_depth_m").value),
            minimum_valid_depth_ratio=float(self.get_parameter("minimum_valid_depth_ratio").value),
        )
        if bool(self.get_parameter("enable_outlier_filter").value):
            surface = reject_local_depth_outliers(
                surface,
                kernel_size=int(self.get_parameter("outlier_kernel_size").value),
                maximum_deviation_m=float(self.get_parameter("outlier_maximum_deviation_m").value),
            )
        if bool(self.get_parameter("enable_plane_removal").value):
            surface = reject_support_plane(
                surface,
                distance_threshold_m=float(self.get_parameter("plane_distance_threshold_m").value),
                minimum_samples=int(self.get_parameter("plane_minimum_samples").value),
                minimum_inlier_fraction=float(
                    self.get_parameter("plane_minimum_inlier_fraction").value
                ),
                maximum_inlier_fraction=float(
                    self.get_parameter("plane_maximum_inlier_fraction").value
                ),
                iterations=int(self.get_parameter("plane_ransac_iterations").value),
            )
        if surface.valid_depth_ratio < float(self.get_parameter("minimum_valid_depth_ratio").value):
            raise ValueError(
                f"cleanup left insufficient valid target depth: {surface.valid_depth_ratio:.3f}"
            )

        # Section: Organized neighboring pixels form faces only after edge, jump, and area validation.
        self._publish_status(TargetState.MESH_RECONSTRUCTION, "reconstructing organized polygon mesh", target=target)
        source_frame = str(bundle.depth_message.header.frame_id or bundle.camera_info.header.frame_id).strip()
        if not source_frame:
            raise ValueError("aligned depth and CameraInfo frame IDs are empty")
        mesh = build_organized_mesh(
            surface,
            frame_id=source_frame,
            stamp=bundle.depth_message.header.stamp,
            pixel_stride=int(self.get_parameter("mesh_pixel_stride").value),
            max_edge_length=float(self.get_parameter("mesh_max_edge_length").value),
            max_depth_jump=float(self.get_parameter("mesh_max_depth_jump").value),
            min_triangle_area=float(self.get_parameter("mesh_min_triangle_area").value),
            max_triangle_area=float(self.get_parameter("mesh_max_triangle_area").value),
            max_triangles=int(self.get_parameter("mesh_max_triangles").value),
        )

        # Section: PCA and grasp analysis use a reduced copy while retaining full mesh accuracy.
        self._publish_status(TargetState.GEOMETRY_ANALYSIS, "calculating OBB and surface grasp", target=target)
        analysis_points = voxel_downsample(
            mesh.vertices,
            float(self.get_parameter("voxel_leaf_size").value),
        )
        obb = compute_oriented_bounding_box(analysis_points)
        grasp = select_surface_grasp(
            mesh,
            obb,
            surface.valid_mask,
            boundary_margin_pixels=float(self.get_parameter("grasp_boundary_margin_pixels").value),
            normal_search_radius=float(self.get_parameter("normal_search_radius").value),
            lower_boundary_rejection_fraction=float(
                self.get_parameter("grasp_lower_boundary_rejection_fraction").value
            ),
            minimum_axis_projection=float(
                self.get_parameter("grasp_minimum_axis_projection").value
            ),
        )

        # Section: Publish the candidate pose and polygon-first marker array in the measured frame.
        grasp_pose = make_grasp_pose(mesh, grasp)
        grasp_pose = self._transform_grasp_pose_if_requested(grasp_pose)
        markers = make_target_markers(
            mesh,
            obb,
            grasp,
            label=target,
            grasp_marker_size=float(self.get_parameter("grasp_marker_size").value),
            approach_marker_length=float(self.get_parameter("approach_marker_length").value),
            closing_axis_length=float(self.get_parameter("closing_axis_length").value),
        )
        self._grasp_pose_pub.publish(grasp_pose)
        self._markers_pub.publish(markers)
        if bool(self.get_parameter("publish_debug_overlay").value):
            self._publish_overlay(bundle.rgb_message, rgb, mask, segmentation, mesh, grasp.vertex_index, (depth_width, depth_height), target)
        if bool(self.get_parameter("publish_debug_cloud").value):
            header = bundle.depth_message.header
            cloud = point_cloud2.create_cloud_xyz32(header, mesh.vertices.astype(np.float32).tolist())
            self._debug_cloud_pub.publish(cloud)

        # Section: Record source metadata only after every required output is valid and published.
        self._last_frame_id = mesh.frame_id
        self._last_stamp = mesh.stamp
        self._last_rgb_shape = (rgb_height, rgb_width)
        self._publish_status(
            TargetState.TARGET_READY,
            "validated mesh and grasp candidate published",
            target=target,
            frame_id=mesh.frame_id,
            mask_pixels=int(np.count_nonzero(mask)),
            valid_depth_ratio=surface.valid_depth_ratio,
            vertices=len(mesh.vertices),
            triangles=len(mesh.triangles),
            grasp_quality=grasp.quality,
        )

        # Section: Updated mask geometry supplies lightweight tracking hints without rerunning Qwen.
        return GroundedTarget(
            label=segmentation.label,
            confidence=segmentation.confidence,
            bbox_xyxy=segmentation.bbox_xyxy,
            point_uv=segmentation.point_uv,
        ) if grounding is not None else None

    def _transform_grasp_pose_if_requested(self, pose: PoseStamped) -> PoseStamped:
        """Transform grasp output through TF while retaining camera fallback."""
        # Section: Empty output frame intentionally keeps geometry in the source camera optical frame.
        target_frame = str(self.get_parameter("output_grasp_frame").value).strip().lstrip("/")
        if not target_frame or target_frame == pose.header.frame_id.strip().lstrip("/"):
            return pose
        try:
            return self._tf_buffer.transform(
                pose,
                target_frame,
                timeout=Duration(seconds=max(0.01, float(self.get_parameter("tf_timeout_sec").value))),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Grasp TF to '{target_frame}' failed; retaining camera-frame pose: {exc}"
            )
            return pose

    def _publish_overlay(
        self,
        source_message: Image,
        rgb: np.ndarray,
        mask: np.ndarray,
        segmentation: SegmentationResult,
        mesh: TargetMesh,
        grasp_vertex_index: int,
        depth_size: tuple[int, int],
        target: str,
    ) -> None:
        """Publish an RGB diagnostic with grounding, contour, label, and grasp pixel."""
        # Section: Overlay never substitutes for the binary mask or polygon mesh outputs.
        overlay = cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR)
        contours, _hierarchy = cv2.findContours(
            (mask.astype(np.uint8) * 255), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(overlay, contours, -1, (0, 255, 0), 2)
        left, top, right, bottom = segmentation.bbox_xyxy
        cv2.rectangle(overlay, (left, top), (right, bottom), (0, 180, 255), 2)
        depth_width, depth_height = depth_size
        pixel_u, pixel_v = mesh.pixel_coordinates[int(grasp_vertex_index)]
        rgb_u = int(round(float(pixel_u) * overlay.shape[1] / max(1, depth_width)))
        rgb_v = int(round(float(pixel_v) * overlay.shape[0] / max(1, depth_height)))
        cv2.circle(overlay, (rgb_u, rgb_v), 6, (0, 0, 255), -1)
        cv2.putText(
            overlay,
            f"{target} mask/grasp",
            (max(0, left), max(20, top - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        message = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        message.header = source_message.header
        self._overlay_pub.publish(message)

    def _publish_status(self, state: TargetState, message: str, **details: Any) -> None:
        """Publish one machine-readable target pipeline state transition."""
        # Section: Stable JSON supports rosbag analysis and hardware acceptance checks.
        payload = {"state": state.value, "message": str(message)}
        payload.update(details)
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _clear_outputs(self, reason: str) -> None:
        """Delete stale markers and publish an empty mask after invalidation."""
        # Section: DELETEALL ensures no old mesh or grasp remains visible as current evidence.
        if self._last_stamp is not None:
            self._markers_pub.publish(make_delete_all_markers(self._last_frame_id, self._last_stamp))
        if self._last_rgb_shape is not None:
            empty = np.zeros(self._last_rgb_shape, dtype=np.uint8)
            message = self._bridge.cv2_to_imgmsg(empty, encoding="mono8")
            message.header.frame_id = self._last_frame_id
            if self._last_stamp is not None:
                message.header.stamp = self._last_stamp
            self._mask_pub.publish(message)
        self.get_logger().debug(f"Cleared target outputs: {reason}")

    def destroy_node(self) -> bool:
        """Stop the model worker before releasing ROS resources."""
        # Section: Bounded joining avoids indefinite shutdown during external launch termination.
        self._stop_event.set()
        self._target_event.set()
        self._worker.join(timeout=3.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run camera callbacks concurrently with the dedicated inference worker."""
    # Section: Multiple ROS executor threads keep sensor intake and TF responsive during publishing.
    rclpy.init(args=args)
    node = TargetGraspGeometryNode()
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
    # Section: Start the perception-only node when invoked through ros2 run or ros2 launch.
    main()
