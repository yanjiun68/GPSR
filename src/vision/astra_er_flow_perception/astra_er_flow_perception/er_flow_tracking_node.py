import json
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from std_msgs.msg import Float32, String

from .depth_roi import DepthROI
from .er_flow_backend import ERFlowBackend
from .template_tracker import TemplateTracker

# Section: Keep runtime defaults aligned with the locally installed ER-Flow checkpoint.
DEFAULT_ER_FLOW_MODEL_PATH = "/home/yanjiun68/ai/models/UnifoLM-ER-Flow"


class ERFlowTrackingNode(Node):
    def __init__(self):
        super().__init__("er_flow_tracking_node")

        # Input/output topics
        self.declare_parameter(
            "aligned_color_topic", "/camera/aligned_color_to_depth/image_raw"
        )
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("command_topic", "/astra_ai/command")
        self.declare_parameter("result_topic", "/astra_ai/result")
        self.declare_parameter("status_topic", "/astra_ai/status")
        self.declare_parameter("depth_output_topic", "/astra_ai/target_depth")
        self.declare_parameter("roi_output_topic", "/astra_ai/target_roi")
        self.declare_parameter(
            "tracking_image_topic", "/astra_ai/tracking_image"
        )

        # ER-Flow model
        self.declare_parameter(
            "model_path", DEFAULT_ER_FLOW_MODEL_PATH
        )
        self.declare_parameter("processor_path", DEFAULT_ER_FLOW_MODEL_PATH)
        self.declare_parameter("device_map", "auto")
        self.declare_parameter("dtype", "bfloat16")
        self.declare_parameter("max_new_tokens", 128)
        # Section: Offload enough weights to CPU to preserve inference headroom on an 8 GiB GPU.
        self.declare_parameter("gpu_max_memory_gib", 5)
        self.declare_parameter("cpu_max_memory_gib", 8)
        # Section: Enforce NF4 weights with nested quantization for this deployment.
        self.declare_parameter("quantization_mode", "nf4")
        self.declare_parameter("use_double_quant", True)
        self.declare_parameter("prompt_file", "")
        self.declare_parameter("inference_interval_frames", 15)
        self.declare_parameter("min_model_confidence", 0.20)

        # Tracking / synchronization
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("search_expansion", 2.0)
        self.declare_parameter("tracker_match_threshold", 0.35)
        self.declare_parameter("template_update_rate", 0.04)

        # Depth
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 10.0)
        self.declare_parameter("roi_inner_fraction", 0.70)
        self.declare_parameter("min_valid_depth_pixels", 20)

        self.bridge = CvBridge()
        self.target = ""
        self.user_prompt = ""
        self.tracking_enabled = False
        self.frame_count = 0
        self.last_result_time = None
        self.smoothed_fps = 0.0

        self.depth_info = None
        self._state_lock = threading.Lock()
        self._inference_running = False
        self._inference_thread = None
        self._pending_detection = None
        self._last_inference_error = ""

        prompt_path = self.get_parameter("prompt_file").value
        if not prompt_path:
            prompt_path = os.path.join(
                get_package_share_directory("astra_er_flow_perception"),
                "prompts",
                "er_flow_target_prompt.txt",
            )
        prompt_template = Path(os.path.expanduser(prompt_path)).read_text(
            encoding="utf-8"
        )

        model_path = os.path.expanduser(self.get_parameter("model_path").value)
        processor_path = self.get_parameter("processor_path").value
        if not processor_path:
            processor_path = model_path

        self.backend = ERFlowBackend(
            model_path=model_path,
            processor_path=processor_path,
            device_map=self.get_parameter("device_map").value,
            dtype=self.get_parameter("dtype").value,
            max_new_tokens=int(self.get_parameter("max_new_tokens").value),
            gpu_max_memory_gib=int(
                self.get_parameter("gpu_max_memory_gib").value
            ),
            cpu_max_memory_gib=int(
                self.get_parameter("cpu_max_memory_gib").value
            ),
            quantization_mode=self.get_parameter("quantization_mode").value,
            use_double_quant=bool(
                self.get_parameter("use_double_quant").value
            ),
            prompt_template=prompt_template,
        )

        self.tracker = TemplateTracker(
            search_expansion=float(self.get_parameter("search_expansion").value),
            match_threshold=float(
                self.get_parameter("tracker_match_threshold").value
            ),
            template_update_rate=float(
                self.get_parameter("template_update_rate").value
            ),
        )

        self.depth_roi = DepthROI(
            depth_scale=float(self.get_parameter("depth_scale").value),
            min_depth_m=float(self.get_parameter("min_depth_m").value),
            max_depth_m=float(self.get_parameter("max_depth_m").value),
            roi_inner_fraction=float(
                self.get_parameter("roi_inner_fraction").value
            ),
            min_valid_pixels=int(
                self.get_parameter("min_valid_depth_pixels").value
            ),
        )

        self.create_subscription(
            String,
            self.get_parameter("command_topic").value,
            self.command_cb,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("depth_info_topic").value,
            self.depth_info_cb,
            10,
        )

        self.color_sub = Subscriber(
            self,
            Image,
            self.get_parameter("aligned_color_topic").value,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = Subscriber(
            self,
            Image,
            self.get_parameter("depth_topic").value,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop").value),
        )
        self.sync.registerCallback(self.frame_cb)

        self.result_pub = self.create_publisher(
            String, self.get_parameter("result_topic").value, 10
        )
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self.depth_pub = self.create_publisher(
            Float32, self.get_parameter("depth_output_topic").value, 10
        )
        self.roi_pub = self.create_publisher(
            RegionOfInterest, self.get_parameter("roi_output_topic").value, 10
        )
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter("tracking_image_topic").value, 10
        )

        self.publish_status(
            "ready",
            {
                "model_path": model_path,
                "quantization": self.backend.quantization_mode,
                "double_quant": self.backend.use_double_quant,
                "command_topic": self.get_parameter("command_topic").value,
                "result_topic": self.get_parameter("result_topic").value,
            },
        )

    def depth_info_cb(self, msg):
        self.depth_info = msg

    def publish_status(self, state, extra=None):
        """Special function: Publish status only while the ROS context is valid."""
        # Section: Avoid publishing from an inference thread after SIGINT shutdown.
        if not rclpy.ok():
            return
        payload = {"state": state}
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(msg)

    def command_cb(self, msg):
        text = msg.data.strip()
        if not text:
            return

        action = "track"
        target = ""
        prompt = ""

        try:
            obj = json.loads(text)
            if isinstance(obj, dict):
                action = str(obj.get("action", "track")).lower()
                target = str(obj.get("target", "")).strip()
                prompt = str(obj.get("prompt", "")).strip()
            else:
                target = str(obj).strip()
        except json.JSONDecodeError:
            target = text

        if action in ("stop", "cancel", "clear"):
            with self._state_lock:
                self.tracking_enabled = False
                self.target = ""
                self.user_prompt = ""
                self._pending_detection = None
                self.tracker.reset()
            self.publish_status("stopped")
            return

        if not target:
            self.publish_status(
                "command_error",
                {"message": "track command requires a non-empty target"},
            )
            return

        with self._state_lock:
            self.target = target
            self.user_prompt = prompt
            self.tracking_enabled = True
            self.frame_count = 0
            self._pending_detection = None
            self._last_inference_error = ""
            self.tracker.reset()

        self.publish_status(
            "tracking_requested",
            {"target": target, "prompt_override": bool(prompt)},
        )

    def should_infer(self):
        interval = max(
            1, int(self.get_parameter("inference_interval_frames").value)
        )
        return self.tracker.roi is None or (self.frame_count % interval == 0)

    def start_inference(self, image, target, user_prompt):
        with self._state_lock:
            if self._inference_running:
                return
            self._inference_running = True

        thread = threading.Thread(
            target=self._inference_worker,
            args=(image.copy(), target, user_prompt),
            daemon=True,
        )
        # Section: Retain the worker so shutdown can wait for active CUDA kernels.
        self._inference_thread = thread
        thread.start()

    def wait_for_inference(self, timeout_sec=30.0):
        """Special function: Wait briefly for the active model generation to finish."""
        # Section: Copy the reference under lock and join without blocking callbacks.
        with self._state_lock:
            thread = self._inference_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=float(timeout_sec))

    def _inference_worker(self, image, target, user_prompt):
        started = time.perf_counter()
        try:
            result = self.backend.infer_roi(image, target, user_prompt)
            result["inference_ms"] = (time.perf_counter() - started) * 1000.0
            result["source_image"] = image
            with self._state_lock:
                self._pending_detection = result
                self._last_inference_error = ""
        except Exception as exc:
            with self._state_lock:
                self._last_inference_error = str(exc)
            self.publish_status(
                "inference_error",
                {"target": target, "message": str(exc)},
            )
        finally:
            with self._state_lock:
                self._inference_running = False

    def consume_pending_detection(self):
        with self._state_lock:
            pending = self._pending_detection
            self._pending_detection = None
        return pending

    def update_fps(self):
        now = time.perf_counter()
        if self.last_result_time is not None:
            dt = now - self.last_result_time
            if dt > 0:
                fps = 1.0 / dt
                if self.smoothed_fps <= 0:
                    self.smoothed_fps = fps
                else:
                    self.smoothed_fps = 0.9 * self.smoothed_fps + 0.1 * fps
        self.last_result_time = now

    def point_from_depth(self, u, v, z):
        if self.depth_info is None or not np.isfinite(z):
            return None

        fx = float(self.depth_info.k[0])
        fy = float(self.depth_info.k[4])
        cx = float(self.depth_info.k[2])
        cy = float(self.depth_info.k[5])

        if fx == 0.0 or fy == 0.0:
            return None

        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        return {"x": x, "y": y, "z": z}

    def publish_outputs(
        self,
        color_msg,
        color,
        depth,
        roi,
        source,
        tracking_score,
        model_confidence,
        inference_ms=None,
    ):
        x, y, w, h = [int(v) for v in roi]
        center_u = int(round(x + w / 2))
        center_v = int(round(y + h / 2))

        depth_stats = self.depth_roi.estimate(depth, roi)
        depth_m = None if depth_stats is None else depth_stats["depth_m"]
        position = (
            None
            if depth_m is None
            else self.point_from_depth(center_u, center_v, depth_m)
        )

        self.update_fps()

        result = {
            "target": self.target,
            "quantization": self.backend.quantization_mode,
            "nf4_verified": self.backend.quantization_verified,
            "quantized_linear_layers": self.backend.quantized_linear_count,
            "found": True,
            "tracking": True,
            "source": source,
            "roi_xywh": [x, y, w, h],
            "center_uv": [center_u, center_v],
            "depth_m": depth_m,
            "position_depth_frame_m": position,
            "model_confidence": model_confidence,
            "tracker_score": tracking_score,
            "output_fps": self.smoothed_fps,
            "frame_id": color_msg.header.frame_id,
            "stamp": {
                "sec": int(color_msg.header.stamp.sec),
                "nanosec": int(color_msg.header.stamp.nanosec),
            },
        }
        if depth_stats is not None:
            result["depth_stats"] = depth_stats
        if inference_ms is not None:
            result["inference_ms"] = float(inference_ms)

        result_msg = String()
        result_msg.data = json.dumps(result, separators=(",", ":"))
        self.result_pub.publish(result_msg)

        roi_msg = RegionOfInterest()
        roi_msg.x_offset = int(x)
        roi_msg.y_offset = int(y)
        roi_msg.width = int(w)
        roi_msg.height = int(h)
        roi_msg.do_rectify = False
        self.roi_pub.publish(roi_msg)

        if depth_m is not None:+ 
            dmsg = Float32()
            dmsg.data = float(depth_m)
            self.depth_pub.publish(dmsg)

        debug = color.copy()
        cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
        label = self.target
        if depth_m is not None:
            label += f" {depth_m:.3f} m"
        cv2.putText(
            debug,
            label,
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = color_msg.header
        self.debug_pub.publish(debug_msg)

    def publish_not_found(self, color_msg, reason):
        self.update_fps()
        result = {
            "target": self.target,
            "quantization": self.backend.quantization_mode,
            "nf4_verified": self.backend.quantization_verified,
            "quantized_linear_layers": self.backend.quantized_linear_count,
            "found": False,
            "tracking": bool(self.tracking_enabled),
            "source": "none",
            "reason": reason,
            "depth_m": None,
            "output_fps": self.smoothed_fps,
            "frame_id": color_msg.header.frame_id,
            "stamp": {
                "sec": int(color_msg.header.stamp.sec),
                "nanosec": int(color_msg.header.stamp.nanosec),
            },
        }
        msg = String()
        msg.data = json.dumps(result, separators=(",", ":"))
        self.result_pub.publish(msg)

    def frame_cb(self, color_msg, depth_msg):
        with self._state_lock:
            enabled = self.tracking_enabled
            target = self.target
            user_prompt = self.user_prompt

        if not enabled:
            return

        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
        except Exception as exc:
            self.publish_status("image_decode_error", {"message": str(exc)})
            return

        self.frame_count += 1

        source = None
        model_conf = None
        inference_ms = None

        pending = self.consume_pending_detection()
        if pending is not None:
            min_conf = float(
                self.get_parameter("min_model_confidence").value
            )
            if (
                pending.get("found")
                and pending.get("roi_xywh")
                and float(pending.get("confidence", 0.0)) >= min_conf
            ):
                source_image = pending.pop("source_image", color)
                if self.tracker.initialize(
                    source_image, pending["roi_xywh"]
                ):
                    source = "er_flow"
                    model_conf = float(pending.get("confidence", 0.0))
                    inference_ms = pending.get("inference_ms")

        tracker_result = self.tracker.update(color)
        if tracker_result is not None:
            if source is None:
                source = "tracker"
            roi = tracker_result["roi"]
            score = float(tracker_result["score"])

            self.publish_outputs(
                color_msg=color_msg,
                color=color,
                depth=depth,
                roi=roi,
                source=source,
                tracking_score=score,
                model_confidence=model_conf,
                inference_ms=inference_ms,
            )
        else:
            self.publish_not_found(
                color_msg,
                "waiting_for_er_flow" if self._inference_running else "target_not_localized",
            )

        if self.should_infer():
            self.start_inference(color, target, user_prompt)


def main(args=None):
    rclpy.init(args=args)
    node = ERFlowTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Section: Let active BitsAndBytes CUDA kernels finish before process teardown.
        node.wait_for_inference(timeout_sec=30.0)
        node.destroy_node()
        # Section: Launch may already have shut down the shared rclpy context on SIGINT.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
