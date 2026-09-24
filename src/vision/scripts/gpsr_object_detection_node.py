#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
"""Section: Lifecycle-managed GPSR object and human YOLO detector."""

from __future__ import annotations

import os
import copy
import json
import re
import threading
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from putrabot_interfaces.msg import ObjectDetection, ObjectDetections
from robot_yasmin_tasks.object_knowledge import ObjectKnowledge


# Section: Object prompts are loaded from object_knowledge.yaml during CONFIGURE.
DEFAULT_OBJECT_CLASS_PROMPTS = ["__none__"]


# Section: Optional YOLO import provides a deterministic CONFIGURE failure.
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# Section: Optional torch import enforces the configured CUDA-only policy.
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class GpsrObjectDetectionNode(LifecycleNode):
    """Special function: Run the configured Qwen or YOLO detector only while lifecycle ACTIVE."""

    def __init__(self) -> None:
        """Special function: Declare detector configuration without loading the model."""
        super().__init__("gpsr_object_detection_node")

        # Section: Declare camera, detector, and output parameters.
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/perception/raw_object_detections")
        self.declare_parameter("annotated_image_topic", "/perception/object_detections_image")
        # Section: Qwen is the preferred semantic world-sensing backend for GPSR object skills.
        self.declare_parameter("inference_backend", "qwen")
        # Section: Default standalone GPSR object detection to the requested YOLOE weights.
        self.declare_parameter(
            "yolo_model_path",
            "/home/yanjiun68/workspace/WS1/PutrabotUPM-project/yoloe-26s-seg.pt",
        )
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("inference_stride", 3)
        self.declare_parameter("yolo_device", "cuda")
        self.declare_parameter("include_classes", ["__none__"])
        self.declare_parameter("exclude_classes", ["person"])
        self.declare_parameter("yolo_class_prompts", DEFAULT_OBJECT_CLASS_PROMPTS)
        self.declare_parameter(
            "object_knowledge_file", str(ObjectKnowledge.default_path())
        )
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("show_gui", False)
        self.declare_parameter("gui_window_name", "GPSR Object Detection")
        self.declare_parameter("ready_topic", "/perception/object_detector_ready")
        self.declare_parameter("enable_model_warmup", True)
        # Section: Qwen settings mirror test_camera.py while adding a ROS-safe structured output contract.
        self.declare_parameter("qwen_model_path", "/home/yanjiun68/ai/models/UnifoLM-ER-1")
        self.declare_parameter(
            "qwen_prompt_path",
            str(Path(get_package_share_directory("vision")) / "config" / "qwen_world_prompt.txt"),
        )
        self.declare_parameter("qwen_max_new_tokens", 256)
        self.declare_parameter("qwen_image_size", 448)
        self.declare_parameter("qwen_minimum_confidence", 0.35)
        self.declare_parameter("qwen_inference_period_sec", 1.0)
        # Section: FIND_OBJECT and COUNT_OBJECTS publish structured queries here to focus shared Qwen sensing.
        self.declare_parameter("qwen_query_topic", "/perception/object_detector_query")

        # Section: Resource handles stay empty until CONFIGURE.
        self.bridge = None
        self.model = None
        self.qwen_processor = None
        self.qwen_model = None
        self.qwen_torch = None
        self.qwen_model_lock = threading.RLock()
        self.qwen_query_lock = threading.RLock()
        self.qwen_query: dict[str, Any] = {}
        self.qwen_inference_thread = None
        self.qwen_inference_running = False
        self.qwen_last_inference_time = 0.0
        self.qwen_force_inference = False
        self.image_subscription = None
        self.qwen_query_subscription = None
        self.detections_pub = None
        self.annotated_pub = None
        self.ready_pub = None
        self.skill_active = False
        self.detector_ready = False
        self.frame_count = 0
        self.gui_available = False
        self.get_logger().info("GPSR object detector is UNCONFIGURED.")

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Load the model and allocate camera-facing ROS resources."""
        try:
            # Section: Read runtime parameters for the new clean configuration.
            self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
            self.image_size = int(self.get_parameter("image_size").value)
            self.inference_stride = max(1, int(self.get_parameter("inference_stride").value))
            # Section: Normalize the backend name before selecting exactly one model owner.
            self.inference_backend = str(self.get_parameter("inference_backend").value).strip().casefold()
            if self.inference_backend not in {"qwen", "yolo", "yoloe"}:
                raise ValueError("inference_backend must be qwen or yolo")
            self.include_classes = {
                str(name).strip().lower()
                for name in self.get_parameter("include_classes").value
                if str(name).strip() and str(name).strip().lower() != "__none__"
            }
            self.exclude_classes = {
                str(name).strip().lower()
                for name in self.get_parameter("exclude_classes").value
                if str(name).strip() and str(name).strip().lower() != "__none__"
            }
            # Section: Read open-vocabulary labels before loading YOLOE embeddings.
            self.yolo_class_prompts = [
                str(name).strip().lower()
                for name in self.get_parameter("yolo_class_prompts").value
                if str(name).strip() and str(name).strip().lower() != "__none__"
            ]
            knowledge_file = str(self.get_parameter("object_knowledge_file").value).strip()
            if self.include_classes:
                # Section: A specialized detector such as the person tracker owns its prompts.
                self.yolo_class_prompts = sorted(self.include_classes)
            elif knowledge_file:
                # Section: GPSR detector prompts derive from authoritative supported objects.
                self.yolo_class_prompts = ObjectKnowledge.from_file(
                    knowledge_file
                ).detector_prompts()
            self.publish_annotated_image = bool(
                self.get_parameter("publish_annotated_image").value
            )
            self.show_gui = bool(self.get_parameter("show_gui").value)
            self.gui_window_name = str(self.get_parameter("gui_window_name").value)
            # Section: Resolve CUDA only for the YOLO backend; Qwen uses its own device map.
            self.yolo_device = self.resolve_yolo_device(
                str(self.get_parameter("yolo_device").value)
            ) if self.inference_backend in {"yolo", "yoloe"} else ""
            self.gui_available = self.show_gui and bool(os.environ.get("DISPLAY", "").strip())
            if self.show_gui and not self.gui_available:
                self.get_logger().warn(
                    "show_gui is true, but DISPLAY is empty; GUI is disabled."
                )

            # Section: Heavy model ownership begins in CONFIGURE.
            self.bridge = CvBridge()
            if self.inference_backend in {"yolo", "yoloe"}:
                # Section: Preserve the existing deterministic YOLOE loading path as an explicit backend.
                self.model = self.load_model(str(self.get_parameter("yolo_model_path").value))
                if self.model is None:
                    raise RuntimeError("YOLO model could not be loaded")
                self.configure_open_vocabulary_classes()
                if bool(self.get_parameter("enable_model_warmup").value):
                    self.warm_up_model()
            else:
                # Section: Load the Qwen model using the same local 4-bit strategy as test_camera.py.
                if not TORCH_AVAILABLE:
                    raise RuntimeError("Qwen backend requires torch in the configured vision environment")
                self.load_qwen_model()
            self.frame_count = 0
            self.detector_ready = False
            self.qwen_last_inference_time = 0.0

            # Section: Allocate publishers and subscription without processing while INACTIVE.
            sensor_qos = QoSProfile(depth=10)
            sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.image_subscription = self.create_subscription(
                Image,
                str(self.get_parameter("rgb_topic").value),
                self.image_callback,
                sensor_qos,
            )
            # Section: Cache the latest goal-specific constraints while keeping the detector lifecycle-owned.
            self.qwen_query_subscription = self.create_subscription(
                String,
                str(self.get_parameter("qwen_query_topic").value),
                self.qwen_query_callback,
                10,
            )
            self.detections_pub = self.create_lifecycle_publisher(
                ObjectDetections,
                str(self.get_parameter("detections_topic").value),
                10,
            )
            self.annotated_pub = self.create_lifecycle_publisher(
                Image,
                str(self.get_parameter("annotated_image_topic").value),
                10,
            )
            # Section: Transient readiness lets a later-configured skill server see current state.
            ready_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.ready_pub = self.create_lifecycle_publisher(
                Bool,
                str(self.get_parameter("ready_topic").value),
                ready_qos,
            )
            self.get_logger().info(
                "[GPSR_LIFECYCLE] Object detector configured: "
                f"backend={self.inference_backend}, "
                f"model={self.get_parameter('qwen_model_path').value if self.inference_backend == 'qwen' else self.get_parameter('yolo_model_path').value}"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"Object detector configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Activate lifecycle publishers and permit inference."""
        result = super().on_activate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result
        self.skill_active = True
        self.detector_ready = False
        if self.ready_pub is not None:
            self.ready_pub.publish(Bool(data=False))
        self.get_logger().info(f"[GPSR_LIFECYCLE] {self.inference_backend.upper()} detector: ACTIVE")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Stop inference before deactivating lifecycle publishers."""
        self.skill_active = False
        self.detector_ready = False
        if self.ready_pub is not None:
            self.ready_pub.publish(Bool(data=False))
        if self.gui_available:
            cv2.destroyWindow(self.gui_window_name)
        return super().on_deactivate(state)

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Unload the active backend and destroy configured ROS resources."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Release detector resources during lifecycle shutdown."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Fail closed and reset detector resource ownership."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> None:
        """Special function: Destroy subscription, publishers, model, bridge, and GUI."""
        if self.image_subscription is not None:
            self.destroy_subscription(self.image_subscription)
            self.image_subscription = None
        if self.qwen_query_subscription is not None:
            self.destroy_subscription(self.qwen_query_subscription)
            self.qwen_query_subscription = None
        if self.detections_pub is not None:
            self.destroy_publisher(self.detections_pub)
            self.detections_pub = None
        if self.annotated_pub is not None:
            self.destroy_publisher(self.annotated_pub)
            self.annotated_pub = None
        if self.ready_pub is not None:
            self.destroy_publisher(self.ready_pub)
            self.ready_pub = None
        self.model = None
        # Section: Serialize Qwen cleanup against an in-flight inference worker.
        with self.qwen_model_lock:
            self.qwen_processor = None
            self.qwen_model = None
            self.qwen_torch = None
        # Section: Discard one finished mission's target constraints before a future lifecycle activation.
        with self.qwen_query_lock:
            self.qwen_query = {}
            self.qwen_force_inference = False
        self.bridge = None
        self.frame_count = 0
        self.detector_ready = False
        if self.gui_available:
            cv2.destroyAllWindows()
        self.gui_available = False

    @staticmethod
    def normalize_qwen_symbol(value: Any) -> str:
        """Special function: Convert one query value into a bounded identifier safe for a model prompt."""
        # Section: Drop free-form punctuation so ROS query transport cannot inject prompt instructions.
        normalized = re.sub(r"[^a-z0-9]+", "_", str(value or "").casefold()).strip("_")
        return normalized[:64]

    @classmethod
    def normalize_qwen_query(cls, payload: Any) -> dict[str, Any]:
        """Special function: Validate a GPSR object query before it influences Qwen inference."""
        # Section: Only a fixed schema crosses from GPSR action data into the VLM prompt.
        if not isinstance(payload, dict):
            raise ValueError("Qwen object query must be a JSON object.")
        target_source = payload.get("target", payload)
        target_source = target_source if isinstance(target_source, dict) else {}
        target: dict[str, str] = {}
        for field in ("object_name", "category"):
            value = cls.normalize_qwen_symbol(target_source.get(field, payload.get(field, "")))
            if value:
                target[field] = value

        # Section: Permit semantic characteristics and image-space relations that local code can validate.
        feature_aliases = {
            "color": "object_color",
            "approximate_shape": "shape",
            "reference": "reference_object",
        }
        allowed_features = {
            "object_color",
            "shape",
            "orientation",
            "size_relation",
            "spatial_relation",
            "reference_object",
            "reference_category",
        }
        allowed_size_relations = {"smallest", "largest"}
        allowed_spatial_relations = {"left_of", "right_of", "above", "below", "near", "far_from"}
        filters: list[dict[str, str]] = []
        raw_filters = payload.get("filters", [])
        if raw_filters is None:
            raw_filters = []
        if not isinstance(raw_filters, list):
            raise ValueError("Qwen object query filters must be a list.")
        for item in raw_filters:
            if not isinstance(item, dict):
                continue
            feature = feature_aliases.get(
                cls.normalize_qwen_symbol(item.get("feature", "")),
                cls.normalize_qwen_symbol(item.get("feature", "")),
            )
            value = cls.normalize_qwen_symbol(item.get("value", ""))
            if feature not in allowed_features or not value:
                continue
            if feature == "size_relation" and value == "biggest":
                value = "largest"
            if feature == "size_relation" and value not in allowed_size_relations:
                continue
            if feature == "spatial_relation" and value not in allowed_spatial_relations:
                continue
            candidate = {"feature": feature, "operator": "equals", "value": value}
            if candidate not in filters:
                filters.append(candidate)

        # Section: Result fields are advisory; perception still publishes one stable detector contract.
        allowed_fields = {"image_position", "color", "shape", "orientation", "description"}
        raw_requested_fields = payload.get("requested_fields", [])
        raw_requested_fields = raw_requested_fields if isinstance(raw_requested_fields, list) else []
        requested_fields = [
            value
            for value in (cls.normalize_qwen_symbol(item) for item in raw_requested_fields)
            if value in allowed_fields
        ]
        return {
            "target": target,
            "filters": filters,
            "requested_fields": list(dict.fromkeys(requested_fields)),
        }

    @classmethod
    def qwen_query_instruction(cls, query: dict[str, Any]) -> str:
        """Special function: Render a validated query as a concise Qwen prompt supplement."""
        # Section: The base prompt remains authoritative; this section only prioritizes current GPSR evidence.
        normalized = cls.normalize_qwen_query(query)
        target = normalized["target"]
        filters = normalized["filters"]
        if not target and not filters:
            return ""
        lines = ["Active GPSR search query:"]
        if target.get("object_name"):
            lines.append(f"- target object: {target['object_name']}")
        if target.get("category"):
            lines.append(f"- target category: {target['category']}")
        for item in filters:
            lines.append(f"- {item['feature']}: {item['value']}")
        lines.append("Return all visible candidates needed to verify this query, including any reference object.")
        return "\n\n" + "\n".join(lines)

    @classmethod
    def qwen_object_attributes(cls, item: dict[str, Any]) -> dict[str, str]:
        """Special function: Preserve safe Qwen object attributes beside the stable detection fields."""
        # Section: Keep model-provided spatial wording separate from locally derived image position.
        nested = item.get("attributes", {})
        nested = nested if isinstance(nested, dict) else {}
        fields = {
            "color": "color",
            "shape": "shape",
            "approximate_shape": "shape",
            "orientation": "orientation",
            "relative_position": "model_relative_position",
        }
        attributes: dict[str, str] = {}
        for source, destination in fields.items():
            raw_value = item.get(source, nested.get(source, ""))
            value = cls.normalize_qwen_symbol(raw_value)
            if value:
                attributes[destination] = value
        return attributes

    def qwen_query_callback(self, message: String) -> None:
        """Special function: Cache one validated GPSR object query and request a fresh Qwen frame."""
        # Section: Invalid transport is ignored rather than becoming unconstrained model prompt text.
        try:
            payload = json.loads(str(message.data or ""))
            query = self.normalize_qwen_query(payload)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Ignoring invalid Qwen object query: {exc}")
            return
        with self.qwen_query_lock:
            self.qwen_query = query
            self.qwen_force_inference = True
        self.get_logger().info("Received a structured GPSR object query for Qwen world sensing.")

    def load_model(self, model_path: str) -> Any | None:
        """Special function: Load the configured YOLO model."""
        if not YOLO_AVAILABLE:
            self.get_logger().error("ultralytics is not installed.")
            return None
        try:
            return YOLO(model_path)
        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLO model '{model_path}': {exc}")
            return None

    def configure_open_vocabulary_classes(self) -> None:
        """Special function: Give YOLOE explicit text prompts before its first prediction."""
        if self.model is None:
            raise RuntimeError("Cannot configure class prompts without a loaded YOLO model.")
        set_classes = getattr(self.model, "set_classes", None)
        if not callable(set_classes):
            return

        # Section: An explicit include list narrows inference; otherwise retain broad object coverage.
        requested_prompts = (
            sorted(self.include_classes)
            if self.include_classes
            else list(self.yolo_class_prompts)
        )
        effective_prompts = [
            name for name in requested_prompts
            if name and name not in self.exclude_classes
        ]
        if not effective_prompts:
            raise RuntimeError("Open-vocabulary YOLO requires at least one class prompt.")

        # Section: YOLOE generates text embeddings and replaces numeric placeholder names.
        set_classes(effective_prompts)
        configured_names = {
            str(name).strip().lower()
            for name in getattr(self.model, "names", {}).values()
        }
        if not set(effective_prompts).issubset(configured_names):
            raise RuntimeError("YOLOE did not retain the configured class prompts.")
        self.get_logger().info(
            f"Configured open-vocabulary YOLO with {len(effective_prompts)} object prompts."
        )

    def warm_up_model(self) -> None:
        """Special function: Initialize the configured inference backend before activation."""
        if self.model is None:
            raise RuntimeError("Cannot warm up an unloaded YOLO model.")
        # Section: A small black frame initializes CUDA kernels without publishing fake evidence.
        warmup_frame = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        self.model.predict(
            source=warmup_frame,
            imgsz=self.image_size,
            conf=self.confidence_threshold,
            device=self.yolo_device,
            verbose=False,
        )
        self.get_logger().info("YOLO inference backend warm-up completed.")

    def image_callback(self, msg: Image) -> None:
        """Special function: Run decimated inference only when explicitly ACTIVE."""
        if not self.skill_active:
            return
        self.frame_count += 1
        if (self.frame_count - 1) % self.inference_stride != 0:
            return
        if self.bridge is None or self.detections_pub is None:
            return
        if self.inference_backend in {"yolo", "yoloe"} and self.model is None:
            return

        # Section: Qwen inference runs in a worker so the ROS camera callback remains responsive.
        if self.inference_backend == "qwen":
            now = self.get_clock().now().nanoseconds * 1.0e-9
            period = max(0.1, float(self.get_parameter("qwen_inference_period_sec").value))
            # Section: A new GPSR object query bypasses only the sampling delay, never concurrent model work.
            with self.qwen_query_lock:
                force_inference = self.qwen_force_inference
                query = copy.deepcopy(self.qwen_query)
                if force_inference:
                    self.qwen_force_inference = False
            if (not force_inference and now - self.qwen_last_inference_time < period) or self.qwen_inference_running:
                return
            try:
                rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            except Exception as exc:
                self.get_logger().error(f"Failed to convert RGB image for Qwen: {exc}")
                return
            self.qwen_last_inference_time = now
            self.qwen_inference_running = True
            self.qwen_inference_thread = threading.Thread(
                target=self.qwen_inference_worker,
                args=(copy.deepcopy(msg.header), np.asarray(rgb_img).copy(), query),
                daemon=True,
            )
            self.qwen_inference_thread.start()
            return

        # Section: Convert the camera image and execute the configured YOLO model.
        try:
            bgr_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert RGB image: {exc}")
            return
        detections = self.detect_objects(bgr_img)
        dets_msg = ObjectDetections()
        dets_msg.stamp = msg.header.stamp
        for detection in detections:
            detection.stamp = msg.header.stamp
            dets_msg.detections.append(detection)
        self.detections_pub.publish(dets_msg)
        # Section: Readiness becomes true only after one successful live camera inference.
        if not self.detector_ready:
            self.detector_ready = True
            if self.ready_pub is not None:
                self.ready_pub.publish(Bool(data=True))
            self.get_logger().info("[GPSR_LIFECYCLE] YOLO detector: READY")

        # Section: Debug output remains gated by the same ACTIVE callback.
        if self.publish_annotated_image or self.gui_available:
            self.publish_debug_image(msg, bgr_img, detections)

    def load_qwen_model(self) -> None:
        """Special function: Load the local Qwen VLM and production world-sensing prompt once."""
        # Section: Import heavy model libraries only when the Qwen backend is selected.
        if self.qwen_model is not None and self.qwen_processor is not None:
            return
        # Section: Validate local assets before allocating GPU memory.
        model_path = Path(str(self.get_parameter("qwen_model_path").value)).expanduser().resolve()
        prompt_path = Path(str(self.get_parameter("qwen_prompt_path").value)).expanduser().resolve()
        if not model_path.exists():
            raise FileNotFoundError(f"Qwen model path does not exist: {model_path}")
        if not prompt_path.is_file():
            raise FileNotFoundError(f"Qwen world-sensing prompt does not exist: {prompt_path}")
        # Section: Match test_camera.py's local-only UnifoLM-ER-1 NF4 loading policy.
        from transformers import (
            AutoProcessor,
            BitsAndBytesConfig,
            Qwen3VLForConditionalGeneration,
        )
        quant_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16,
            bnb_4bit_use_double_quant=True,
        )
        self.qwen_processor = AutoProcessor.from_pretrained(str(model_path), local_files_only=True)
        self.qwen_model = Qwen3VLForConditionalGeneration.from_pretrained(
            str(model_path),
            quantization_config=quant_config,
            device_map="auto",
            low_cpu_mem_usage=True,
            local_files_only=True,
        )
        # Section: Disable training behavior because this lifecycle node performs inference only.
        self.qwen_model.eval()
        self.qwen_torch = torch
        self.qwen_prompt = prompt_path.read_text(encoding="utf-8").strip()
        if not self.qwen_prompt:
            raise ValueError(f"Qwen world-sensing prompt is empty: {prompt_path}")
        # Section: Bias Qwen toward the same canonical detector labels consumed by GPSR object knowledge.
        if self.yolo_class_prompts:
            self.qwen_prompt += (
                "\n\nPrefer these configured object labels when visible: "
                + ", ".join(self.yolo_class_prompts)
                + "."
            )
        self.get_logger().info(f"Loaded UnifoLM-ER-1 world-sensing model in NF4: {model_path}")

    def qwen_inference_worker(
        self,
        header: Any,
        rgb_img: np.ndarray,
        query: dict[str, Any],
    ) -> None:
        """Special function: Convert one Qwen camera response into the existing detection topic."""
        try:
            # Section: Serialize model access and prevent cleanup from invalidating model handles.
            with self.qwen_model_lock:
                if not self.skill_active or self.qwen_model is None or self.qwen_processor is None:
                    return
                from PIL import Image as PilImage
                messages = [{
                    "role": "user",
                    "content": [
                        {
                            "type": "image",
                            "image": PilImage.fromarray(rgb_img),
                            "resized_height": int(self.get_parameter("qwen_image_size").value),
                            "resized_width": int(self.get_parameter("qwen_image_size").value),
                        },
                        # Section: The validated target query augments the static world-sensing schema.
                        {"type": "text", "text": self.qwen_prompt + self.qwen_query_instruction(query)},
                    ],
                }]
                inputs = self.qwen_processor.apply_chat_template(
                    messages,
                    tokenize=True,
                    add_generation_prompt=True,
                    return_dict=True,
                    return_tensors="pt",
                ).to(self.qwen_model.device)
                with self.qwen_torch.inference_mode():
                    output_ids = self.qwen_model.generate(
                        **inputs,
                        max_new_tokens=max(32, int(self.get_parameter("qwen_max_new_tokens").value)),
                        do_sample=False,
                    )
                generated_ids = output_ids[:, inputs["input_ids"].shape[1]:]
                response = self.qwen_processor.batch_decode(
                    generated_ids,
                    skip_special_tokens=True,
                    clean_up_tokenization_spaces=False,
                )[0].strip()
            detections = self.parse_qwen_detections(response, int(rgb_img.shape[1]), int(rgb_img.shape[0]), header.stamp)
            if not self.skill_active:
                return
            self.publish_detection_frame(header, rgb_img, detections)
        except Exception as exc:
            self.get_logger().error(f"Qwen world-sensing inference failed: {exc}")
        finally:
            self.qwen_inference_running = False

    def parse_qwen_detections(
        self,
        response: str,
        image_width: int,
        image_height: int,
        stamp: Any,
    ) -> list[ObjectDetection]:
        """Special function: Validate Qwen JSON and convert normalized boxes into ROS detections."""
        # Section: Reject non-JSON output instead of turning model prose into false detections.
        text = str(response or "").strip()
        if text.startswith("```"):
            text = text.strip("`").replace("json\n", "", 1).strip()
        payload = json.loads(text)
        objects = payload.get("objects", []) if isinstance(payload, dict) else []
        if not isinstance(objects, list):
            raise ValueError("Qwen world-sensing response objects must be a list")
        detections: list[ObjectDetection] = []
        minimum_confidence = float(self.get_parameter("qwen_minimum_confidence").value)
        for item in objects:
            # Section: Every world object must provide semantic identity and a usable ROI.
            if not isinstance(item, dict):
                continue
            try:
                name = str(item.get("object_name") or item.get("label") or "").strip().lower()
                bbox = item.get("bbox_2d", item.get("bbox"))
                confidence = float(item.get("confidence", 0.0))
            except (TypeError, ValueError):
                continue
            if not name or not isinstance(bbox, list) or len(bbox) != 4 or confidence < minimum_confidence:
                continue
            try:
                coordinates = [float(value) for value in bbox]
            except (TypeError, ValueError):
                continue
            if not all(np.isfinite(coordinates)):
                continue
            try:
                coordinate_width = float(item.get("coordinate_width", 1000.0))
                coordinate_height = float(item.get("coordinate_height", 1000.0))
            except (TypeError, ValueError):
                continue
            if coordinate_width <= 0.0 or coordinate_height <= 0.0:
                continue
            x1 = int(max(0.0, min(float(image_width - 1), coordinates[0] * image_width / coordinate_width)))
            y1 = int(max(0.0, min(float(image_height - 1), coordinates[1] * image_height / coordinate_height)))
            x2 = int(max(float(x1 + 1), min(float(image_width), coordinates[2] * image_width / coordinate_width)))
            y2 = int(max(float(y1 + 1), min(float(image_height), coordinates[3] * image_height / coordinate_height)))
            detection = ObjectDetection()
            detection.stamp = stamp
            detection.name = name.replace(" ", "_").replace("-", "_")
            detection.class_id = -1
            detection.confidence = confidence
            detection.x1 = x1
            detection.y1 = y1
            detection.x2 = x2
            detection.y2 = y2
            detection.center_u = float(x1 + x2) / 2.0
            detection.center_v = float(y1 + y2) / 2.0
            # Section: Retain Qwen semantic evidence without changing the established bbox fields.
            if hasattr(detection, "image_width"):
                detection.image_width = int(image_width)
            if hasattr(detection, "image_height"):
                detection.image_height = int(image_height)
            if hasattr(detection, "attributes_json"):
                detection.attributes_json = json.dumps(
                    # Section: Use the class schema normalizer so parser test doubles need only expose parameters.
                    GpsrObjectDetectionNode.qwen_object_attributes(item),
                    sort_keys=True,
                )
            detections.append(detection)
        return detections

    def publish_detection_frame(self, header: Any, rgb_img: np.ndarray, detections: list[ObjectDetection]) -> None:
        """Special function: Publish Qwen detections through the existing GPSR detector contract."""
        # Section: Keep readiness and frame publication identical across Qwen and YOLO backends.
        if self.detections_pub is None:
            return
        dets_msg = ObjectDetections()
        dets_msg.stamp = header.stamp
        dets_msg.detections.extend(detections)
        self.detections_pub.publish(dets_msg)
        if not self.detector_ready:
            self.detector_ready = True
            if self.ready_pub is not None:
                self.ready_pub.publish(Bool(data=True))
            self.get_logger().info("[GPSR_LIFECYCLE] Qwen detector: READY")
        if self.publish_annotated_image or self.gui_available:
            bgr_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
            source = Image()
            source.header = header
            self.publish_debug_image(source, bgr_img, detections)

    def detect_objects(self, bgr_img: Any) -> list[ObjectDetection]:
        """Special function: Convert YOLO boxes into object detection messages."""
        try:
            results = self.model.predict(
                source=bgr_img,
                imgsz=self.image_size,
                conf=self.confidence_threshold,
                device=self.yolo_device,
                verbose=False,
            )
        except ValueError as exc:
            if self.requires_cuda_device(self.yolo_device) and "Invalid CUDA" in str(exc):
                raise RuntimeError(
                    f"YOLO CUDA execution failed for device '{self.yolo_device}'. "
                    "CPU fallback is disabled."
                ) from exc
            raise
        if not results or results[0].boxes is None:
            return []

        # Section: Filter requested class names and preserve every visible instance.
        detections: list[ObjectDetection] = []
        names = getattr(self.model, "names", {})
        for box in results[0].boxes:
            class_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())
            name = str(names.get(class_id, class_id)).strip().lower()
            if self.include_classes and name not in self.include_classes:
                continue
            if name in self.exclude_classes:
                continue
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
            detection = ObjectDetection()
            detection.name = name
            detection.class_id = class_id
            detection.confidence = confidence
            detection.x1 = int(max(0, x1))
            detection.y1 = int(max(0, y1))
            detection.x2 = int(max(0, x2))
            detection.y2 = int(max(0, y2))
            detection.center_u = float((x1 + x2) / 2.0)
            detection.center_v = float((y1 + y2) / 2.0)
            # Section: YOLO supplies geometry only; semantic Qwen attributes remain intentionally empty.
            if hasattr(detection, "image_width"):
                detection.image_width = int(bgr_img.shape[1])
            if hasattr(detection, "image_height"):
                detection.image_height = int(bgr_img.shape[0])
            if hasattr(detection, "attributes_json"):
                detection.attributes_json = "{}"
            detections.append(detection)
        return detections

    def publish_debug_image(
        self,
        source_msg: Image,
        bgr_img: Any,
        detections: list[ObjectDetection],
    ) -> None:
        """Special function: Publish and optionally display an annotated ACTIVE frame."""
        annotated = bgr_img.copy()
        for detection in detections:
            cv2.rectangle(
                annotated,
                (detection.x1, detection.y1),
                (detection.x2, detection.y2),
                (0, 180, 255),
                2,
            )
            cv2.putText(
                annotated,
                f"{detection.name} {detection.confidence:.2f}",
                (detection.x1, max(0, detection.y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 180, 255),
                1,
                cv2.LINE_AA,
            )
        if self.publish_annotated_image and self.annotated_pub is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug_msg.header = source_msg.header
            self.annotated_pub.publish(debug_msg)
        if self.gui_available:
            cv2.imshow(self.gui_window_name, annotated)
            cv2.waitKey(1)

    def resolve_yolo_device(self, requested_device: str) -> str:
        """Special function: Require CUDA when a CUDA-like device is requested."""
        device = str(requested_device).strip() or "cuda"
        if self.requires_cuda_device(device) and not self.cuda_is_available():
            raise RuntimeError(
                f"CUDA requested for YOLO object detection ({device}) but torch CUDA "
                "is unavailable. CPU fallback is disabled."
            )
        return device

    @staticmethod
    def requires_cuda_device(device: str) -> bool:
        """Special function: Return whether the configured device requires CUDA."""
        normalized = str(device).strip().lower()
        return (
            normalized in {"cuda", "gpu"}
            or normalized.startswith("cuda:")
            or normalized.replace(",", "").isdigit()
        )

    @staticmethod
    def cuda_is_available() -> bool:
        """Special function: Report whether torch can execute CUDA inference."""
        return bool(TORCH_AVAILABLE and torch.cuda.is_available())


def main(args: list[str] | None = None) -> None:
    """Special function: Run the lifecycle-managed GPSR YOLO detector."""
    rclpy.init(args=args)
    node = GpsrObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.release_resources()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
