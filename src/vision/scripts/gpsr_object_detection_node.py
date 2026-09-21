#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
"""Section: Lifecycle-managed GPSR object and human YOLO detector."""

from __future__ import annotations

import os
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

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
    """Special function: Run YOLO inference only while lifecycle ACTIVE."""

    def __init__(self) -> None:
        """Special function: Declare detector configuration without loading the model."""
        super().__init__("gpsr_object_detection_node")

        # Section: Declare camera, detector, and output parameters.
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/perception/raw_object_detections")
        self.declare_parameter("annotated_image_topic", "/perception/object_detections_image")
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

        # Section: Resource handles stay empty until CONFIGURE.
        self.bridge = None
        self.model = None
        self.image_subscription = None
        self.detections_pub = None
        self.annotated_pub = None
        self.ready_pub = None
        self.skill_active = False
        self.detector_ready = False
        self.frame_count = 0
        self.gui_available = False
        self.get_logger().info("GPSR YOLO lifecycle detector is UNCONFIGURED.")

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Load the model and allocate camera-facing ROS resources."""
        try:
            # Section: Read runtime parameters for the new clean configuration.
            self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
            self.image_size = int(self.get_parameter("image_size").value)
            self.inference_stride = max(1, int(self.get_parameter("inference_stride").value))
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
            self.yolo_device = self.resolve_yolo_device(
                str(self.get_parameter("yolo_device").value)
            )
            self.gui_available = self.show_gui and bool(os.environ.get("DISPLAY", "").strip())
            if self.show_gui and not self.gui_available:
                self.get_logger().warn(
                    "show_gui is true, but DISPLAY is empty; GUI is disabled."
                )

            # Section: Heavy model ownership begins in CONFIGURE.
            self.bridge = CvBridge()
            self.model = self.load_model(str(self.get_parameter("yolo_model_path").value))
            if self.model is None:
                raise RuntimeError("YOLO model could not be loaded")
            self.configure_open_vocabulary_classes()
            if bool(self.get_parameter("enable_model_warmup").value):
                self.warm_up_model()
            self.frame_count = 0
            self.detector_ready = False

            # Section: Allocate publishers and subscription without processing while INACTIVE.
            sensor_qos = QoSProfile(depth=10)
            sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.image_subscription = self.create_subscription(
                Image,
                str(self.get_parameter("rgb_topic").value),
                self.image_callback,
                sensor_qos,
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
                "[GPSR_LIFECYCLE] YOLO configured: "
                f"model={self.get_parameter('yolo_model_path').value}, "
                f"device={self.yolo_device}"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"YOLO detector configure failed: {exc}")
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
        self.get_logger().info("[GPSR_LIFECYCLE] YOLO detector: ACTIVE")
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
        """Special function: Unload YOLO and destroy configured ROS resources."""
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
        self.bridge = None
        self.frame_count = 0
        self.detector_ready = False
        if self.gui_available:
            cv2.destroyAllWindows()
        self.gui_available = False

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
        if self.model is None or self.bridge is None or self.detections_pub is None:
            return

        # Section: Convert the camera image and execute the configured model.
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
