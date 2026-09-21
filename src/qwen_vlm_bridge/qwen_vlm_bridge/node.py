"""ROS 2 node that connects final STT or debug text to a persistent Qwen engine."""

from __future__ import annotations

import hashlib
import json
import queue
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from qwen_vlm_bridge.contract import (
    ContractError,
    DEFAULT_ALLOWED_SKILLS,
    PlannerRequest,
    parse_input,
)
from qwen_vlm_bridge.engine import QwenPlannerEngine


def retained_status_qos() -> QoSProfile:
    """Special function: Retain the latest model status for late-joining diagnostics."""
    # Section: One reliable transient sample exposes READY or ERROR immediately to new subscribers.
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class QwenTextPlannerNode(Node):
    """Special function: Arbitrate STT and debug requests through one loaded Qwen model."""

    def __init__(self) -> None:
        """Special function: Configure ROS interfaces and start the persistent model worker."""
        super().__init__("qwen_text_planner")

        # Section: Declare model and prompt settings compatible with the original test_text.py paths.
        self.declare_parameter("model_path", "/home/yanjiun68/ai/models/Qwen3-VL-4B-Instruct")
        self.declare_parameter(
            "system_prompt_path",
            "/home/yanjiun68/ai/qwen3v1/system_prompt_actionqueue_test.txt",
        )
        self.declare_parameter("max_new_tokens", 256)
        self.declare_parameter("load_in_4bit", True)
        self.declare_parameter("local_files_only", True)
        self.declare_parameter("allowed_skills", list(DEFAULT_ALLOWED_SKILLS))

        # Section: Declare independent speech and manual-prototyping input controls.
        self.declare_parameter("enable_stt_input", True)
        self.declare_parameter("stt_input_topic", "/voice/stt/transcript")
        self.declare_parameter("enable_vad_input", False)
        self.declare_parameter("vad_input_topic", "/voice/stt/utterance_transcript")
        self.declare_parameter("enable_debug_input", True)
        self.declare_parameter("debug_input_topic", "/ai/model/qwen_vlm/debug/input")
        self.declare_parameter("max_instruction_chars", 4000)
        self.declare_parameter("max_pending_requests", 1)
        self.declare_parameter("stt_dedupe_window_sec", 2.0)

        # Section: Declare observable outputs without exposing an unguarded robot-execution path.
        self.declare_parameter("plan_topic", "/ai/model/qwen_vlm/plan")
        self.declare_parameter("raw_response_topic", "/ai/model/qwen_vlm/raw_response")
        self.declare_parameter("status_topic", "/ai/model/qwen_vlm/status")
        self.declare_parameter("result_topic", "/ai/model/qwen_vlm/result")
        self.declare_parameter("debug_result_topic", "/ai/model/qwen_vlm/debug/result")

        # Section: Create publishers before model loading so initialization failures remain observable.
        self.plan_pub = self.create_publisher(
            String, str(self.get_parameter("plan_topic").value), 10
        )
        self.raw_response_pub = self.create_publisher(
            String, str(self.get_parameter("raw_response_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            retained_status_qos(),
        )
        self.result_pub = self.create_publisher(
            String, str(self.get_parameter("result_topic").value), 10
        )
        self.debug_result_pub = self.create_publisher(
            String, str(self.get_parameter("debug_result_topic").value), 10
        )

        # Section: Create only the explicitly enabled subscriptions to prevent duplicate STT routing.
        self.stt_sub = None
        if bool(self.get_parameter("enable_stt_input").value):
            self.stt_sub = self.create_subscription(
                String,
                str(self.get_parameter("stt_input_topic").value),
                lambda msg: self._input_callback(msg, "stt", False),
                10,
            )
        self.vad_sub = None
        if bool(self.get_parameter("enable_vad_input").value):
            self.vad_sub = self.create_subscription(
                String,
                str(self.get_parameter("vad_input_topic").value),
                lambda msg: self._input_callback(msg, "vad", False),
                10,
            )
        self.debug_sub = None
        if bool(self.get_parameter("enable_debug_input").value):
            self.debug_sub = self.create_subscription(
                String,
                str(self.get_parameter("debug_input_topic").value),
                lambda msg: self._input_callback(msg, "debug", True),
                10,
            )

        # Section: Bound pending work while allowing callbacks and diagnostics to remain responsive.
        capacity = max(1, int(self.get_parameter("max_pending_requests").value))
        self._request_queue: queue.Queue[PlannerRequest | None] = queue.Queue(maxsize=capacity)
        self._shutdown_event = threading.Event()
        self._ready = False
        self._load_error = ""
        self._recent_stt: dict[str, float] = {}
        self._recent_stt_lock = threading.Lock()

        # Section: Construct the lightweight engine now; the worker owns expensive model loading.
        self.engine = QwenPlannerEngine(
            model_path=str(self.get_parameter("model_path").value),
            system_prompt_path=str(self.get_parameter("system_prompt_path").value),
            max_new_tokens=int(self.get_parameter("max_new_tokens").value),
            load_in_4bit=bool(self.get_parameter("load_in_4bit").value),
            local_files_only=bool(self.get_parameter("local_files_only").value),
            allowed_skills=list(self.get_parameter("allowed_skills").value),
        )
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def _input_callback(self, msg: String, source: str, allow_envelope: bool) -> None:
        """Special function: Validate and enqueue one final transcript or manual debug command."""
        # Section: Parse input before touching the bounded inference queue.
        try:
            request = parse_input(
                msg.data,
                source,
                allow_envelope=allow_envelope,
                max_instruction_chars=int(self.get_parameter("max_instruction_chars").value),
            )
        except ContractError as exc:
            self._publish_rejection(source, "", str(exc))
            return

        # Section: This bridge is deliberately publish-only until a guarded scheduler adapter exists.
        if request.submit_requested:
            self._publish_rejection(
                source,
                request.request_id,
                "direct plan submission is disabled; use the GPSR scheduler",
            )
            return

        # Section: Suppress only near-simultaneous STT duplicates; repeated debug tests remain intentional.
        if source in {"stt", "vad"} and self._is_duplicate_stt(request.instruction):
            self._publish_rejection(source, request.request_id, "duplicate transcript")
            return

        # Section: Report permanent model-load failure instead of accumulating requests indefinitely.
        if self._load_error:
            self._publish_rejection(source, request.request_id, self._load_error)
            return

        # Section: Queue at most the configured pending count and reject overload deterministically.
        try:
            self._request_queue.put_nowait(request)
        except queue.Full:
            self._publish_rejection(source, request.request_id, "Qwen planner is busy")
            return
        self._publish_status("QUEUED", request.request_id, source, "request accepted")

    def _is_duplicate_stt(self, instruction: str) -> bool:
        """Special function: Detect duplicate speech publications within a short configurable window."""
        # Section: Hash normalized text without retaining spoken content in the deduplication cache.
        digest = hashlib.sha256(instruction.casefold().encode("utf-8")).hexdigest()
        now = time.monotonic()
        window = max(0.0, float(self.get_parameter("stt_dedupe_window_sec").value))
        with self._recent_stt_lock:
            stale = [key for key, stamp in self._recent_stt.items() if now - stamp > window]
            for key in stale:
                self._recent_stt.pop(key, None)
            if window > 0.0 and digest in self._recent_stt:
                return True
            self._recent_stt[digest] = now
        return False

    def _worker_loop(self) -> None:
        """Special function: Load Qwen once and process queued requests serially."""
        # Section: Expose model initialization independently from ROS process startup.
        self._publish_status("LOADING", "", "system", "loading Qwen model")
        try:
            self.engine.load()
        except Exception as exc:
            self._load_error = f"Qwen model failed to load: {exc}"
            self.get_logger().error(self._load_error)
            self._publish_status("ERROR", "", "system", self._load_error)
            return
        self._ready = True
        self.get_logger().info("Qwen text planner is ready.")
        self._publish_status("READY", "", "system", "model loaded")

        # Section: A single worker guarantees ordered, non-overlapping GPU inference.
        while not self._shutdown_event.is_set():
            try:
                request = self._request_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if request is None:
                self._request_queue.task_done()
                break
            try:
                self._process_request(request)
            finally:
                self._request_queue.task_done()

    def _process_request(self, request: PlannerRequest) -> None:
        """Special function: Generate, validate, and publish one correlated Qwen plan."""
        # Section: Record wall-clock model latency separately from time spent waiting in the queue.
        self._publish_status(
            "INFERENCING", request.request_id, request.source, "generating semantic plan"
        )
        started = time.monotonic()
        raw_response = ""
        try:
            plan, raw_response = self.engine.generate_plan(request.instruction)
            latency_ms = round((time.monotonic() - started) * 1000.0, 3)

            # Section: Publish raw and validated outputs independently for model troubleshooting.
            self.raw_response_pub.publish(String(data=raw_response))
            self.plan_pub.publish(String(data=self.engine.format_plan(plan)))
            result = {
                "request_id": request.request_id,
                "source": request.source,
                "success": True,
                "requires_clarification": bool(plan["requires_clarification"]),
                "plan": plan,
                "raw_response": raw_response,
                "error": "",
                "latency_ms": latency_ms,
                "dry_run": True,
                "submit_requested": request.submit_requested,
                "submitted": False,
            }
            self._publish_result(request, result)
            message = (
                "clarification required"
                if bool(plan["requires_clarification"])
                else "validated plan published"
            )
            self._publish_status("READY", request.request_id, request.source, message)
        except Exception as exc:
            latency_ms = round((time.monotonic() - started) * 1000.0, 3)
            error = str(exc)
            self.get_logger().error(
                f"Qwen request failed: request_id={request.request_id} error={error}"
            )
            result = {
                "request_id": request.request_id,
                "source": request.source,
                "success": False,
                "requires_clarification": False,
                "plan": {},
                "raw_response": raw_response,
                "error": error,
                "latency_ms": latency_ms,
                "dry_run": True,
                "submit_requested": request.submit_requested,
                "submitted": False,
            }
            self._publish_result(request, result)
            self._publish_status("READY", request.request_id, request.source, error)

    def _publish_result(self, request: PlannerRequest, result: dict[str, Any]) -> None:
        """Special function: Publish common results and mirror debug results on their dedicated topic."""
        # Section: Stable JSON makes shell tests and rosbag inspection deterministic.
        message = String(data=json.dumps(result, ensure_ascii=False, sort_keys=True))
        self.result_pub.publish(message)
        if request.source == "debug":
            self.debug_result_pub.publish(message)

    def _publish_rejection(self, source: str, request_id: str, error: str) -> None:
        """Special function: Report an input rejection without invoking the model."""
        # Section: Rejections use the same result shape as completed inference requests.
        result = {
            "request_id": request_id,
            "source": source,
            "success": False,
            "requires_clarification": False,
            "plan": {},
            "raw_response": "",
            "error": error,
            "latency_ms": 0.0,
            "dry_run": True,
            "submit_requested": False,
            "submitted": False,
        }
        message = String(data=json.dumps(result, ensure_ascii=False, sort_keys=True))
        self.result_pub.publish(message)
        if source == "debug":
            self.debug_result_pub.publish(message)
        self._publish_status("REJECTED", request_id, source, error)

    def _publish_status(
        self,
        state: str,
        request_id: str,
        source: str,
        message: str,
    ) -> None:
        """Special function: Publish a machine-readable planner lifecycle snapshot."""
        # Section: Status never contains the spoken instruction, reducing accidental transcript logging.
        payload = {
            "state": state,
            "request_id": request_id,
            "source": source,
            "message": message,
            "model_ready": self._ready,
            "queue_size": self._request_queue.qsize() if hasattr(self, "_request_queue") else 0,
        }
        self.status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True))
        )

    def destroy_node(self) -> bool:
        """Special function: Stop accepting worker work before destroying ROS resources."""
        # Section: Signal shutdown without blocking indefinitely on an active GPU generation call.
        self._shutdown_event.set()
        try:
            self._request_queue.put_nowait(None)
        except queue.Full:
            pass
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Special function: Run the Qwen text planner with ROS 2 callbacks on the main thread."""
    # Section: Standard rclpy ownership keeps shutdown behavior compatible with ros2 launch.
    rclpy.init(args=args)
    node = QwenTextPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
