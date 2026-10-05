#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
"""Section: Command-triggered Qwen ROI grounding for the vision Astra camera adapter."""

from __future__ import annotations

# Section: Standard-library support for JSON contracts, immutable frame records, and worker synchronization.
import copy
import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any

# Section: OpenCV draws an optional operator-visible ROI overlay on the Astra RGB stream.
import cv2
import numpy as np

# Section: ROS receives Astra-adapter images and publishes ROI results plus annotated images.
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String
from vision.msg import RoiResult as RoiResultMessage

# Section: Keep model imports optional so validation helpers remain testable without the Qwen environment.
QWEN_IMPORT_ERROR: Exception | None = None
try:
    from PIL import Image as PilImage
    import torch
    from transformers import (
        AutoProcessor,
        BitsAndBytesConfig,
        Qwen3VLForConditionalGeneration,
    )
except ImportError as exc:
    QWEN_IMPORT_ERROR = exc


# Section: Bound external target text before it becomes part of the fixed model prompt.
MAX_TARGET_TEXT_LENGTH = 128

# Section: Qwen returns boxes in one stable normalized coordinate system for resolver compatibility.
DEFAULT_COORDINATE_WIDTH = 1000
DEFAULT_COORDINATE_HEIGHT = 1000


@dataclass(frozen=True)
class TargetRequest:
    """Special function: Store one normalized target object request from the Astra Qwen query topic."""

    # Section: The canonical target ID is program-owned and returned with every result.
    target_id: str


@dataclass(frozen=True)
class LatestFrame:
    """Special function: Hold a copied Astra RGB frame and its original ROS header for one inference request."""

    # Section: Qwen consumes RGB pixels while the header stamp correlates the resulting ROI with depth data.
    rgb_image: np.ndarray
    header: Any
    # Section: A local sequence prevents repeating tracker work on the same camera observation.
    sequence: int


class OpenCvTemplateTracker:
    """Special function: Track a validated ER-Flow box using only standard OpenCV operations."""

    def __init__(self, search_margin: float = 0.50, minimum_score: float = 0.20) -> None:
        """Special function: Configure bounded local template matching without another learned model."""
        # Section: Search remains local so similar objects elsewhere do not silently replace the target.
        self.search_margin = max(0.10, float(search_margin))
        self.minimum_score = min(1.0, max(0.0, float(minimum_score)))
        self._template: np.ndarray | None = None
        self._bbox: tuple[int, int, int, int] | None = None

    def reset(self) -> None:
        """Special function: Remove stale visual evidence after a command or tracking failure."""
        # Section: Both template and geometry must be cleared atomically by the caller's state lock.
        self._template = None
        self._bbox = None

    def initialize(self, rgb_image: np.ndarray, bbox: tuple[int, int, int, int]) -> None:
        """Special function: Seed tracking from one strictly validated model-generated pixel ROI."""
        # Section: Clamp the model box before slicing to prevent empty or wrapped NumPy regions.
        height, width = rgb_image.shape[:2]
        left = max(0, min(width - 1, int(bbox[0])))
        top = max(0, min(height - 1, int(bbox[1])))
        right = max(left + 1, min(width, int(bbox[2])))
        bottom = max(top + 1, min(height, int(bbox[3])))
        gray = cv2.cvtColor(np.asarray(rgb_image), cv2.COLOR_RGB2GRAY)
        template = gray[top:bottom, left:right].copy()
        if template.size < 16:
            raise ValueError("Tracked ROI is too small to initialize an OpenCV template.")
        self._template = template
        self._bbox = (left, top, right, bottom)

    def update(self, rgb_image: np.ndarray) -> tuple[tuple[int, int, int, int], float] | None:
        """Special function: Find the previous target locally and return its updated box and score."""
        # Section: An absent acquisition is an ordinary waiting state, not a tracker exception.
        if self._template is None or self._bbox is None:
            return None
        gray = cv2.cvtColor(np.asarray(rgb_image), cv2.COLOR_RGB2GRAY)
        image_height, image_width = gray.shape[:2]
        left, top, right, bottom = self._bbox
        box_width = right - left
        box_height = bottom - top
        margin_x = max(4, int(round(box_width * self.search_margin)))
        margin_y = max(4, int(round(box_height * self.search_margin)))
        search_left = max(0, left - margin_x)
        search_top = max(0, top - margin_y)
        search_right = min(image_width, right + margin_x)
        search_bottom = min(image_height, bottom + margin_y)
        search = gray[search_top:search_bottom, search_left:search_right]
        if search.shape[0] < box_height or search.shape[1] < box_width:
            self.reset()
            return None
        # Section: SQDIFF_NORMED is stable for low-texture templates; one minus its minimum is confidence-like.
        scores = cv2.matchTemplate(search, self._template, cv2.TM_SQDIFF_NORMED)
        minimum_value, _maximum_value, minimum_location, _maximum_location = cv2.minMaxLoc(scores)
        score = max(0.0, min(1.0, 1.0 - float(minimum_value)))
        if not math.isfinite(score) or score < self.minimum_score:
            self.reset()
            return None
        updated_left = search_left + int(minimum_location[0])
        updated_top = search_top + int(minimum_location[1])
        updated_bbox = (
            updated_left,
            updated_top,
            min(image_width, updated_left + box_width),
            min(image_height, updated_top + box_height),
        )
        self._bbox = updated_bbox
        return updated_bbox, score


def normalize_target_text(value: Any) -> str:
    """Special function: Validate a short target-object description before model prompting."""
    # Section: Require plain text instead of converting arbitrary JSON values into prompts.
    if not isinstance(value, str):
        raise ValueError("Target object must be a string.")
    # Section: Normalize whitespace while preserving useful multi-word visual descriptions.
    target_id = " ".join(value.split())
    if not target_id:
        raise ValueError("Target object must not be empty.")
    if len(target_id) > MAX_TARGET_TEXT_LENGTH:
        raise ValueError(f"Target object exceeds {MAX_TARGET_TEXT_LENGTH} characters.")
    return target_id


def parse_target_request(raw_text: str) -> TargetRequest:
    """Special function: Accept a plain target name or a compact JSON target envelope."""
    # Section: Bound the transport payload before parsing so a topic message cannot become an unbounded prompt.
    if not isinstance(raw_text, str):
        raise ValueError("Qwen query message must be text.")
    text = raw_text.strip()
    if not text:
        raise ValueError("Qwen query message is empty.")
    if len(text) > MAX_TARGET_TEXT_LENGTH * 4:
        raise ValueError("Qwen query message is too long.")
    # Section: A plain string is the convenient operator form: ros2 topic pub ... data:='cup'.
    if not text.startswith("{"):
        return TargetRequest(target_id=normalize_target_text(text))
    # Section: JSON callers may use target_id, object, target, or query without supplying model instructions.
    payload = json.loads(text)
    if not isinstance(payload, dict):
        raise ValueError("Qwen JSON query must be an object.")
    target_value = payload.get("target_id", payload.get("object", payload.get("target", payload.get("query"))))
    return TargetRequest(target_id=normalize_target_text(target_value))


def extract_json_object(model_text: str) -> dict[str, Any]:
    """Special function: Extract exactly one JSON object from a Qwen response with optional Markdown fencing."""
    # Section: Trim common code fences before strict JSON parsing because models occasionally add them.
    text = str(model_text or "").strip()
    if text.startswith("```"):
        text = text.strip("`").removeprefix("json").strip()
    # Section: Recover one outer object only; prose without a JSON object remains invalid.
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end <= start:
        raise ValueError("Qwen response does not contain a complete JSON object.")
    payload = json.loads(text[start : end + 1])
    if not isinstance(payload, dict):
        raise ValueError("Qwen response must decode to a JSON object.")
    return payload


def scale_bbox_to_pixels(
    bbox_2d: tuple[float, float, float, float],
    coordinate_width: int,
    coordinate_height: int,
    image_width: int,
    image_height: int,
) -> list[int]:
    """Special function: Convert normalized Qwen ROI coordinates into bounded Astra RGB pixels."""
    # Section: Reject impossible dimensions before applying model-controlled coordinates.
    if min(coordinate_width, coordinate_height, image_width, image_height) <= 0:
        raise ValueError("ROI coordinate and image dimensions must be positive.")
    left, top, right, bottom = bbox_2d
    # Section: Preserve a non-empty clipped rectangle suitable for the annotated-image publisher.
    pixel_left = max(0, min(image_width - 1, int(math.floor(left * image_width / coordinate_width))))
    pixel_top = max(0, min(image_height - 1, int(math.floor(top * image_height / coordinate_height))))
    pixel_right = max(pixel_left + 1, min(image_width, int(math.ceil(right * image_width / coordinate_width))))
    pixel_bottom = max(pixel_top + 1, min(image_height, int(math.ceil(bottom * image_height / coordinate_height))))
    return [pixel_left, pixel_top, pixel_right, pixel_bottom]


def normalize_qwen_roi(
    payload: dict[str, Any],
    request: TargetRequest,
    image_width: int,
    image_height: int,
    minimum_confidence: float,
) -> dict[str, Any]:
    """Special function: Validate Qwen grounding JSON and emit a resolver-ready ROI result."""
    # Section: Restrict model status to an explicit public vocabulary rather than guessing missing results.
    status = str(payload.get("status", "")).strip().casefold()
    # Section: Accept the prompt's compatibility alias but expose only the four public ROI statuses.
    status = "ok" if status == "found" else status
    if status not in {"ok", "not_found", "ambiguous", "uncertain"}:
        raise ValueError("Qwen ROI status must be ok, not_found, ambiguous, or uncertain.")
    # Section: Program-owned request identity prevents the model from retargeting a caller's object.
    result: dict[str, Any] = {
        "status": status,
        "target_id": request.target_id,
        "bbox_2d": None,
        "coordinate_width": None,
        "coordinate_height": None,
        "bbox_pixels": None,
        "confidence": None,
    }
    if status != "ok":
        return result
    # Section: A found target requires four finite, ordered coordinates in a declared coordinate system.
    bbox_value = payload.get("bbox_2d", payload.get("bbox"))
    if not isinstance(bbox_value, list) or len(bbox_value) != 4:
        raise ValueError("Found Qwen ROI requires bbox_2d with four values.")
    bbox = tuple(float(value) for value in bbox_value)
    if not all(math.isfinite(value) for value in bbox) or bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
        raise ValueError("Qwen ROI coordinates must be finite and ordered.")
    coordinate_width = int(payload.get("coordinate_width", DEFAULT_COORDINATE_WIDTH))
    coordinate_height = int(payload.get("coordinate_height", DEFAULT_COORDINATE_HEIGHT))
    if coordinate_width <= 0 or coordinate_height <= 0:
        raise ValueError("Qwen ROI coordinate dimensions must be positive.")
    if bbox[0] < 0.0 or bbox[1] < 0.0 or bbox[2] > coordinate_width or bbox[3] > coordinate_height:
        raise ValueError("Qwen ROI lies outside its declared coordinate system.")
    # Section: Confidence gates suppress weak model boxes before they reach a depth resolver.
    confidence = float(payload.get("confidence", 0.0))
    if not math.isfinite(confidence) or confidence < 0.0 or confidence > 1.0:
        raise ValueError("Qwen ROI confidence must be between zero and one.")
    if confidence < float(minimum_confidence):
        result["status"] = "uncertain"
        result["confidence"] = confidence
        return result
    result.update(
        {
            "bbox_2d": [float(value) for value in bbox],
            "coordinate_width": coordinate_width,
            "coordinate_height": coordinate_height,
            "bbox_pixels": scale_bbox_to_pixels(
                bbox,
                coordinate_width,
                coordinate_height,
                int(image_width),
                int(image_height),
            ),
            "confidence": confidence,
        }
    )
    return result


class AstraCamQwenNode(Node):
    """Special function: Ground one requested object ROI against the latest vision Astra RGB frame."""

    def __init__(self) -> None:
        """Special function: Configure Astra topics, output contracts, and one reusable local Qwen model."""
        # Section: The graph name is distinct from the generic GPSR detector and mirrors the Astra adapter purpose.
        super().__init__("astra_cam_qw")
        # Section: Camera and query interfaces default to the stable outputs of astra_camera.launch.py.
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("query_topic", "/astra_cam_qw/query")
        self.declare_parameter("result_topic", "/astra_cam_qw/result")
        self.declare_parameter("typed_result_topic", "/vision/roi/result")
        self.declare_parameter("annotated_image_topic", "/astra_cam_qw/annotated_image")
        # Section: Model controls use the installed Qwen3-VL-compatible UnifoLM ER-Flow checkpoint.
        self.declare_parameter("model_path", "/home/yanjiun68/ai/models/UnifoLM-ER-Flow")
        self.declare_parameter("image_size", 448)
        self.declare_parameter("max_new_tokens", 192)
        self.declare_parameter("minimum_confidence", 0.35)
        # Section: Heavy semantic inference is capped at 1 Hz while standard OpenCV tracking publishes at 20 Hz.
        self.declare_parameter("inference_rate_hz", 1.0)
        self.declare_parameter("tracking_rate_hz", 20.0)
        self.declare_parameter("tracker_search_margin", 0.50)
        self.declare_parameter("tracker_minimum_score", 0.20)
        # Section: Optional visual debugging remains off by default for headless robot deployments.
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("show_gui", False)
        self.declare_parameter("gui_window_name", "Astra Qwen ROI")

        # Section: Retain only one copied latest RGB frame and prevent concurrent GPU generation requests.
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_frame: LatestFrame | None = None
        self._frame_sequence = 0
        self._inference_lock = threading.Lock()
        self._inference_running = False
        self._last_inference_started = -math.inf
        self._state_lock = threading.RLock()
        self._active_request: TargetRequest | None = None
        self._command_revision = 0
        self._tracker_revision = 0
        self._last_tracked_sequence = -1
        self._tracker = OpenCvTemplateTracker(
            search_margin=float(self.get_parameter("tracker_search_margin").value),
            minimum_score=float(self.get_parameter("tracker_minimum_score").value),
        )

        # Section: Load model resources once at startup, following the 4-bit Qwen setup used by test_camera.py.
        self.processor, self.model, self.torch = self._load_qwen_model()

        # Section: Sensor-data QoS matches Astra's best-effort image publisher and avoids stale frame queues.
        camera_topic = str(self.get_parameter("camera_topic").value)
        self._camera_subscription = self.create_subscription(
            RosImage,
            camera_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        # Section: Queries are ordinary reliable text commands because they trigger one expensive model operation.
        self._query_subscription = self.create_subscription(
            String,
            str(self.get_parameter("query_topic").value),
            self._query_callback,
            10,
        )
        self._result_publisher = self.create_publisher(
            String,
            str(self.get_parameter("result_topic").value),
            10,
        )
        self._typed_result_publisher = self.create_publisher(
            RoiResultMessage,
            str(self.get_parameter("typed_result_topic").value),
            10,
        )
        self._annotated_image_publisher = self.create_publisher(
            RosImage,
            str(self.get_parameter("annotated_image_topic").value),
            10,
        )
        tracking_rate_hz = max(0.1, float(self.get_parameter("tracking_rate_hz").value))
        # Section: The timer drops stale frames rather than creating a callback backlog.
        self._tracking_timer = self.create_timer(1.0 / tracking_rate_hz, self._tracking_tick)
        self.get_logger().info(
            "Astra Qwen ROI node ready: camera=%s query=%s result=%s"
            % (
                camera_topic,
                str(self.get_parameter("query_topic").value),
                str(self.get_parameter("result_topic").value),
            )
        )

    def _load_qwen_model(self) -> tuple[Any, Any, Any]:
        """Special function: Load quantized UnifoLM ER-Flow using the reference strategy."""
        # Section: Raise a clear startup error when ROS used Python without the configured Qwen dependencies.
        if QWEN_IMPORT_ERROR is not None:
            raise RuntimeError(
                "Qwen dependencies are unavailable; set VISION_PYTHON to the qwen3v1 virtual-environment Python."
            ) from QWEN_IMPORT_ERROR
        model_path = str(self.get_parameter("model_path").value)
        self.get_logger().info("Loading UnifoLM-ER-Flow processor for ROI grounding...")
        processor = AutoProcessor.from_pretrained(
            model_path,
            local_files_only=True,
        )
        # Section: Quantize the compatible checkpoint to NF4 for constrained robot GPU memory.
        quantization = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16,
            bnb_4bit_use_double_quant=True,
        )
        self.get_logger().info("Loading UnifoLM-ER-Flow in NF4 for ROI grounding...")
        model = Qwen3VLForConditionalGeneration.from_pretrained(
            model_path,
            quantization_config=quantization,
            device_map="auto",
            low_cpu_mem_usage=True,
            local_files_only=True,
        )
        # Section: Disable training behavior because this node performs inference only.
        model.eval()
        self.get_logger().info("UnifoLM-ER-Flow loaded in NF4 for ROI grounding.")
        return processor, model, torch

    def _image_callback(self, message: RosImage) -> None:
        """Special function: Cache one RGB frame from the Astra adapter for the next command-triggered Qwen request."""
        try:
            # Section: RGB conversion creates the exact channel order expected by PIL and the Qwen processor.
            rgb_image = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
            # Section: Copy pixels and header because ROS may recycle the incoming message after this callback returns.
            with self._frame_lock:
                self._frame_sequence += 1
                latest_frame = LatestFrame(
                    np.asarray(rgb_image).copy(),
                    copy.deepcopy(message.header),
                    self._frame_sequence,
                )
                self._latest_frame = latest_frame
        except Exception as exc:
            # Section: Preserve the previous valid frame after one malformed image conversion failure.
            self.get_logger().error(f"Astra RGB conversion failed: {exc}")

    def _query_callback(self, message: String) -> None:
        """Special function: Replace the active target and let the rate-limited timer schedule inference."""
        try:
            # Section: Input accepts only one target description, never arbitrary model instruction text.
            request = parse_target_request(message.data)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._publish_result(
                {"status": "uncertain", "processing_status": "invalid_query", "error": str(exc)}
            )
            self.get_logger().warn(f"Rejected Astra Qwen query: {exc}")
            return
        # Section: A revision switch invalidates any late model result from the previous command.
        with self._state_lock:
            self._command_revision += 1
            self._active_request = request
            self._tracker_revision = 0
            self._last_tracked_sequence = -1
            self._tracker.reset()
            revision = self._command_revision
        with self._frame_lock:
            latest_frame = self._latest_frame
        if latest_frame is None:
            self._publish_result(
                {
                    "status": "uncertain",
                    "processing_status": "no_frame",
                    "target_id": request.target_id,
                    "command_revision": revision,
                    "error": "No RGB frame received yet.",
                }
            )

    def _tracking_tick(self) -> None:
        """Special function: Schedule at most 1 Hz inference and publish fresh OpenCV tracking at 20 Hz."""
        # Section: Snapshot only the newest camera frame; no timer invocation queues historical images.
        with self._frame_lock:
            latest_frame = self._latest_frame
        with self._state_lock:
            request = self._active_request
            revision = self._command_revision
        if latest_frame is None or request is None:
            return
        self._schedule_inference_if_due(latest_frame, request, revision)
        # Section: Update at most once per source frame even when the timer is faster than the camera.
        with self._state_lock:
            if self._tracker_revision != revision or self._last_tracked_sequence == latest_frame.sequence:
                return
            tracked = self._tracker.update(latest_frame.rgb_image)
            self._last_tracked_sequence = latest_frame.sequence
        if tracked is None:
            self._publish_result(
                {
                    "status": "uncertain",
                    "processing_status": "tracking_lost",
                    "target_id": request.target_id,
                    "command_revision": revision,
                    "image_width": int(latest_frame.rgb_image.shape[1]),
                    "image_height": int(latest_frame.rgb_image.shape[0]),
                },
                header=latest_frame.header,
                tracking_valid=False,
            )
            return
        bbox, tracker_score = tracked
        result = {
            "status": "ok",
            "processing_status": "tracking_ok",
            "target_id": request.target_id,
            "command_revision": revision,
            "bbox_pixels": list(bbox),
            "bbox_2d": list(bbox),
            "coordinate_width": int(latest_frame.rgb_image.shape[1]),
            "coordinate_height": int(latest_frame.rgb_image.shape[0]),
            "confidence": float(tracker_score),
            "image_width": int(latest_frame.rgb_image.shape[1]),
            "image_height": int(latest_frame.rgb_image.shape[0]),
        }
        self._publish_result(result, header=latest_frame.header, tracking_valid=True)
        self._publish_annotation(latest_frame, result)

    def _schedule_inference_if_due(
        self,
        latest_frame: LatestFrame,
        request: TargetRequest,
        revision: int,
    ) -> None:
        """Special function: Start one GPU request only when the global 1 Hz interval has elapsed."""
        # Section: The lock prevents overlapping generation and enforces the rate across command changes.
        now = time.monotonic()
        inference_rate_hz = max(0.01, float(self.get_parameter("inference_rate_hz").value))
        with self._inference_lock:
            if self._inference_running or now - self._last_inference_started < 1.0 / inference_rate_hz:
                return
            self._inference_running = True
            self._last_inference_started = now
        try:
            # Section: The worker receives immutable request/frame/revision snapshots and cannot block tracking callbacks.
            threading.Thread(
                target=self._run_inference,
                args=(latest_frame, request, revision),
                daemon=True,
            ).start()
        except Exception as exc:
            with self._inference_lock:
                self._inference_running = False
            self._publish_result(
                {
                    "status": "uncertain",
                    "processing_status": "model_error",
                    "target_id": request.target_id,
                    "command_revision": revision,
                    "error": str(exc),
                },
                header=latest_frame.header,
                model_generated=True,
            )

    @staticmethod
    def _build_roi_prompt(request: TargetRequest) -> str:
        """Special function: Build a fixed Qwen grounding contract for exactly one requested visual target."""
        # Section: JSON serialization makes target text data rather than free-form instruction content inside the prompt.
        target_json = json.dumps(request.target_id, ensure_ascii=False)
        return (
            "You are a ROS 2 Astra camera ROI grounding system. Analyze only the supplied current RGB image.\n"
            f"Find the visible target object described by this data: {target_json}.\n"
            "Return exactly one JSON object and no Markdown or prose.\n"
            "Use this schema exactly:\n"
            '{"status":"ok|not_found|ambiguous|uncertain","bbox_2d":[x_min,y_min,x_max,y_max],'
            '"coordinate_width":1000,"coordinate_height":1000,"confidence":0.0}\n'
            "When status is ok, bbox_2d must be four ordered integer image coordinates in 0..1000.\n"
            "When status is not_found, ambiguous, or uncertain, set bbox_2d to null and confidence to 0.0.\n"
            "Do not estimate distance, map position, robot motion, or objects not visible in this image."
        )

    def _run_inference(self, latest_frame: LatestFrame, request: TargetRequest, revision: int) -> None:
        """Special function: Ground one requested object ROI with Qwen and publish a stamped Astra-compatible result."""
        try:
            # Section: Convert the copied RGB ndarray to the in-memory PIL image expected by Qwen3-VL.
            pil_image = PilImage.fromarray(latest_frame.rgb_image)
            image_size = max(64, int(self.get_parameter("image_size").value))
            messages = [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image",
                            "image": pil_image,
                            "resized_height": image_size,
                            "resized_width": image_size,
                        },
                        {"type": "text", "text": self._build_roi_prompt(request)},
                    ],
                }
            ]
            # Section: Apply Qwen's chat template and place encoded image/text tensors on the selected model device.
            inputs = self.processor.apply_chat_template(
                messages,
                tokenize=True,
                add_generation_prompt=True,
                return_dict=True,
                return_tensors="pt",
            ).to(self.model.device)
            # Section: Deterministic decoding matches test_camera.py and makes repeated operator requests reproducible.
            with self.torch.inference_mode():
                output_ids = self.model.generate(
                    **inputs,
                    max_new_tokens=max(32, int(self.get_parameter("max_new_tokens").value)),
                    do_sample=False,
                )
            # Section: Decode only completion tokens so prompt text cannot be mistaken for model JSON.
            generated_ids = output_ids[:, inputs["input_ids"].shape[1] :]
            model_text = self.processor.batch_decode(
                generated_ids,
                skip_special_tokens=True,
                clean_up_tokenization_spaces=False,
            )[0].strip()
            raw_payload = extract_json_object(model_text)
            result = normalize_qwen_roi(
                raw_payload,
                request,
                int(latest_frame.rgb_image.shape[1]),
                int(latest_frame.rgb_image.shape[0]),
                float(self.get_parameter("minimum_confidence").value),
            )
            # Section: Preserve the source RGB stamp so this result can be correlated with a depth observation.
            result["image_stamp"] = {
                "sec": int(latest_frame.header.stamp.sec),
                "nanosec": int(latest_frame.header.stamp.nanosec),
            }
            result["camera_topic"] = str(self.get_parameter("camera_topic").value)
            result["image_width"] = int(latest_frame.rgb_image.shape[1])
            result["image_height"] = int(latest_frame.rgb_image.shape[0])
            result["processing_status"] = "model_ok" if result["status"] == "ok" else "model_complete"
            result["command_revision"] = revision
            # Section: A completed worker must not replace state selected by a newer command.
            with self._state_lock:
                if revision != self._command_revision or request != self._active_request:
                    return
                if result["status"] == "ok":
                    bbox = tuple(int(value) for value in result["bbox_pixels"])
                    self._tracker.initialize(latest_frame.rgb_image, bbox)
                    self._tracker_revision = revision
                    self._last_tracked_sequence = latest_frame.sequence
                else:
                    self._tracker.reset()
                    self._tracker_revision = 0
            self._publish_result(
                result,
                header=latest_frame.header,
                model_generated=True,
                tracking_valid=result["status"] == "ok",
            )
            self._publish_annotation(latest_frame, result)
        except Exception as exc:
            # Section: Report parse and model failures as structured topic output rather than silently dropping the query.
            self.get_logger().error(f"Astra Qwen ROI inference failed: {exc}")
            self._publish_result(
                {
                    "status": "uncertain",
                    "processing_status": "model_error",
                    "target_id": request.target_id,
                    "command_revision": revision,
                    "image_width": int(latest_frame.rgb_image.shape[1]),
                    "image_height": int(latest_frame.rgb_image.shape[0]),
                    "error": str(exc),
                },
                header=latest_frame.header,
                model_generated=True,
            )
        finally:
            # Section: Always release the single-inference reservation regardless of model or output failure.
            with self._inference_lock:
                self._inference_running = False

    def _publish_annotation(self, latest_frame: LatestFrame, result: dict[str, Any]) -> None:
        """Special function: Publish and optionally display the Astra RGB frame with its accepted Qwen ROI."""
        # Section: Skip image work entirely when the operator did not request annotation publishing.
        if not bool(self.get_parameter("publish_annotated_image").value):
            return
        annotated = latest_frame.rgb_image.copy()
        bbox_pixels = result.get("bbox_pixels")
        if result.get("status") == "ok" and isinstance(bbox_pixels, list) and len(bbox_pixels) == 4:
            # Section: Draw the accepted pixel ROI and its confidence without altering the source camera frame.
            left, top, right, bottom = [int(value) for value in bbox_pixels]
            cv2.rectangle(annotated, (left, top), (right - 1, bottom - 1), (0, 255, 0), 2)
            label = f"{result['target_id']} {float(result['confidence']):.2f}"
            cv2.putText(annotated, label, (left, max(22, top - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # Section: Preserve the original image header so visual tools align the annotation with the source frame.
        annotated_message = self.bridge.cv2_to_imgmsg(annotated, encoding="rgb8")
        annotated_message.header = latest_frame.header
        self._annotated_image_publisher.publish(annotated_message)
        # Section: Desktop GUI remains explicitly opt-in so headless robot runs do not call OpenCV HighGUI.
        if bool(self.get_parameter("show_gui").value):
            try:
                window_name = str(self.get_parameter("gui_window_name").value)
                cv2.imshow(window_name, cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)
            except cv2.error as exc:
                self.get_logger().error(f"Astra Qwen ROI GUI failed: {exc}")

    def _publish_result(
        self,
        result: dict[str, Any],
        *,
        header: Any | None = None,
        model_generated: bool = False,
        tracking_valid: bool = False,
    ) -> None:
        """Special function: Publish matching typed and compatibility JSON ROI result envelopes."""
        # Section: Operational errors never escape through the semantic status field.
        payload = dict(result)
        status = str(payload.get("status", "uncertain")).strip().casefold()
        if status not in {"ok", "not_found", "ambiguous", "uncertain"}:
            payload.setdefault("processing_status", status or "invalid_status")
            status = "uncertain"
        payload["status"] = status
        payload.setdefault("processing_status", "unspecified")
        payload.setdefault("command_revision", self._command_revision)
        # Section: Compact JSON preserves the existing CLI/debug transport during typed-message migration.
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._result_publisher.publish(message)
        # Section: The typed contract carries a valid status on every output, including failures without a box.
        typed = RoiResultMessage()
        if header is not None:
            typed.header = copy.deepcopy(header)
        typed.command_revision = int(payload.get("command_revision", 0))
        typed.target_id = str(payload.get("target_id", ""))
        typed.status = status
        typed.processing_status = str(payload.get("processing_status", "unspecified"))
        typed.confidence = float(payload.get("confidence") or 0.0)
        typed.image_width = max(0, int(payload.get("image_width") or 0))
        typed.image_height = max(0, int(payload.get("image_height") or 0))
        typed.model_generated = bool(model_generated)
        typed.tracking_valid = bool(tracking_valid and status == "ok")
        bbox_pixels = payload.get("bbox_pixels")
        if status == "ok" and isinstance(bbox_pixels, list) and len(bbox_pixels) == 4:
            left, top, right, bottom = [int(value) for value in bbox_pixels]
            typed.roi.x_offset = max(0, left)
            typed.roi.y_offset = max(0, top)
            typed.roi.width = max(0, right - left)
            typed.roi.height = max(0, bottom - top)
            typed.roi.do_rectify = False
        self._typed_result_publisher.publish(typed)

    def destroy_node(self) -> bool:
        """Special function: Close the optional OpenCV GUI before releasing ROS resources."""
        # Section: HighGUI cleanup is conditional because headless deployments never create the window.
        if bool(self.get_parameter("show_gui").value):
            try:
                cv2.destroyWindow(str(self.get_parameter("gui_window_name").value))
            except cv2.error:
                pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Special function: Start the Astra Qwen ROI node and release ROS resources on shutdown."""
    # Section: Let rclpy consume standard ROS arguments before constructing the model-backed node.
    rclpy.init(args=args)
    node: AstraCamQwenNode | None = None
    try:
        # Section: Construct the node before spinning so model failures are visible as startup errors.
        node = AstraCamQwenNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Section: Ctrl+C is the expected interactive shutdown path for this command-driven node.
        pass
    finally:
        # Section: Destroy only a successfully constructed node and keep rclpy shutdown idempotent.
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# Section: Execute only when ROS starts this installed script directly.
if __name__ == "__main__":
    main()
