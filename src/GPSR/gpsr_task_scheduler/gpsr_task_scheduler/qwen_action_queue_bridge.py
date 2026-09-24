"""Section: Validate Qwen ActionQueue JSON and route it to the ROS executor."""

# Section: Standard-library support for JSON validation, IDs, locking, and typing.
from __future__ import annotations

import json
import threading
import uuid
from typing import Any

# Section: ROS action, node, message, and status interfaces used by the bridge.
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from robot_ai_interfaces.action import ExecuteActionQueue
from robot_ai_interfaces.msg import ActionQueue, ActionStep
from std_msgs.msg import String


# Section: Keep Qwen aliases separate from the executor's canonical action vocabulary.
QWEN_ACTION_ALIASES = {
    "MOVE_TO": "NAVIGATE",
    "PICK": "PICK_OBJECT",
    "PLACE": "PLACE_OBJECT",
    "HANDOVER": "HANDOVER_OBJECT",
    "FOLLOW": "FOLLOW_PERSON",
    "GUIDE": "GUIDE_PERSON",
    "SPEAK": "SAY",
    "ANSWER_QUESTION": "ASK_AND_LISTEN",
    # Section: Feature-specific object searches are constraints on the one FIND_OBJECT primitive.
    "FIND_EXTREME_OBJECT": "FIND_OBJECT",
}


# Section: Match the action identifiers accepted by robot_ai_interfaces/ActionStep.
CANONICAL_ACTION_TYPES = frozenset({
    "NAVIGATE",
    "FIND_OBJECT",
    "FIND_PERSON",
    "FIND_PERSON_BY_ATTRIBUTE",
    "COUNT_OBJECTS",
    "COUNT_PEOPLE",
    "EXTRACT_PERSON_INFO",
    "PICK_OBJECT",
    "PLACE_OBJECT",
    "HANDOVER_OBJECT",
    "FOLLOW_PERSON",
    "GUIDE_PERSON",
    "SAY",
    "ASK_AND_LISTEN",
})


# Section: Reject model actions that require a scheduler semantic compiler or a missing skill.
UNSUPPORTED_QWEN_ACTIONS = frozenset({
    "OBSERVE",
    "VERIFY_PERSON",
    "DESCRIBE_PERSON",
    "SPEAK_RESULT",
})


def _string_value(value: Any) -> str:
    """Special function: Convert an optional model value into a trimmed string."""
    return str(value or "").strip()


def _first_string(mapping: dict[str, Any], *keys: str) -> str:
    """Special function: Select the first non-empty string from a Qwen parameter map."""
    for key in keys:
        value = _string_value(mapping.get(key))
        if value:
            return value
    return ""


def _as_string_list(value: Any, field_name: str) -> list[str]:
    """Special function: Validate an optional JSON string-list field."""
    if value is None:
        return []
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"{field_name} must be a list of strings.")
    return [item.strip() for item in value if item.strip()]


def _append_object_filter(params: dict[str, Any], feature: str, value: str) -> None:
    """Special function: Add one canonical object-query constraint without duplicating it."""
    # Section: FIND_OBJECT and COUNT_OBJECTS share an ordered JSON filter list with local perception.
    filters = params.get("filters", [])
    if filters is None:
        filters = []
    if not isinstance(filters, list):
        raise ValueError("Object-query filters must be a list.")
    candidate = {"feature": feature, "operator": "equals", "value": value}
    normalized = [item for item in filters if isinstance(item, dict)]
    if candidate not in normalized:
        normalized.append(candidate)
    params["filters"] = normalized


def _normalize_object_constraints(params: dict[str, Any], action_name: str) -> None:
    """Special function: Convert Qwen feature aliases into the shared object-query schema."""
    # Section: Largest and smallest select a candidate; they never create a separate executor action.
    mode = _string_value(params.get("mode", params.get("size_relation"))).casefold()
    mode = {"biggest": "largest"}.get(mode, mode)
    if mode:
        if mode not in {"largest", "smallest"}:
            raise ValueError(f"{action_name} mode must be largest or smallest.")
        _append_object_filter(params, "size_relation", mode)

    # Section: Preserve only image-verifiable relations for the current RGB perception contract.
    relation = _string_value(params.get("spatial_relation", params.get("relation"))).casefold()
    relation = {
        "left": "left_of",
        "right": "right_of",
        "under": "below",
        "over": "above",
        "next_to": "near",
        "beside": "near",
        "far": "far_from",
    }.get(relation, relation)
    if relation:
        if relation not in {"left_of", "right_of", "above", "below", "near", "far_from"}:
            raise ValueError(f"{action_name} relation is not supported by image-space perception.")
        reference = _first_string(params, "reference_object", "reference", "relative_to")
        if not reference:
            raise ValueError(f"{action_name} relation requires reference_object.")
        _append_object_filter(params, "spatial_relation", relation)
        _append_object_filter(params, "reference_object", reference)

    # Section: Qwen may supply semantic characteristics in a compact attributes object.
    attributes = params.get("attributes", {})
    if attributes is not None and not isinstance(attributes, dict):
        raise ValueError(f"{action_name} attributes must be an object.")
    for raw_feature, value in dict(attributes or {}).items():
        feature = {
            "color": "object_color",
            "approximate_shape": "shape",
        }.get(_string_value(raw_feature).casefold(), _string_value(raw_feature).casefold())
        normalized_value = _string_value(value)
        if feature in {"object_color", "shape", "orientation"} and normalized_value:
            _append_object_filter(params, feature, normalized_value)


def _payload_command_id(payload_text: str) -> str:
    """Special function: Recover an optional command ID for rejected Qwen payloads."""
    try:
        payload = json.loads(str(payload_text))
    except json.JSONDecodeError:
        return ""
    if not isinstance(payload, dict):
        return ""
    return _string_value(payload.get("command_id", payload.get("task_id")))


def _canonical_action_name(raw_name: Any) -> str:
    """Special function: Normalize one Qwen action name to an executor primitive."""
    supplied = _string_value(raw_name).upper()
    if supplied in UNSUPPORTED_QWEN_ACTIONS:
        raise ValueError(f"Qwen action {supplied!r} is not supported by the executor.")
    canonical = QWEN_ACTION_ALIASES.get(supplied, supplied)
    if canonical not in CANONICAL_ACTION_TYPES:
        raise ValueError(f"Unknown or unsupported Qwen action: {supplied!r}.")
    return canonical


def _normalized_parameters(action_type: str, raw_params: dict[str, Any]) -> dict[str, Any]:
    """Special function: Add executor-compatible parameter aliases without discarding Qwen data."""
    params = dict(raw_params)

    # Section: Preserve the executor's named navigation target contract.
    if action_type == "NAVIGATE":
        location = _first_string(params, "target_location", "location", "destination")
        if location:
            params["target_location"] = location

    # Section: Preserve the remote object-search target and location contract.
    if action_type == "FIND_OBJECT":
        object_name = _first_string(params, "object_name", "object", "target_object")
        category = _first_string(params, "category", "filter", "target_category")
        location = _first_string(params, "target_location", "location")
        if object_name:
            params["object_name"] = object_name
        if category:
            params["category"] = category
        if location:
            params["location"] = location
        # Section: Normalize every Qwen feature variation into the shared local FIND_OBJECT contract.
        _normalize_object_constraints(params, "FIND_OBJECT")

    # Section: COUNT_OBJECTS forwards the same Qwen-verifiable constraints through filter_json.
    if action_type == "COUNT_OBJECTS":
        object_name = _first_string(params, "object_name", "object", "target_object")
        category = _first_string(params, "category", "filter", "target_category")
        location = _first_string(params, "target_location", "location")
        if object_name:
            params["object_name"] = object_name
        if category:
            params["category"] = category
        if location:
            params["location"] = location
        _normalize_object_constraints(params, "COUNT_OBJECTS")

    # Section: Convert criteria searches into the existing attribute-search payload shape.
    if action_type == "FIND_PERSON_BY_ATTRIBUTE":
        criteria = params.get("criteria", params.get("selector", {}))
        if not isinstance(criteria, dict) or not criteria:
            raise ValueError("FIND_PERSON_BY_ATTRIBUTE requires a non-empty criteria object.")
        params["selector"] = dict(criteria)

    # Section: Preserve person, destination, object, and speech fields used by executor clients.
    if action_type in {"FIND_PERSON", "FOLLOW_PERSON", "GUIDE_PERSON", "HANDOVER_OBJECT"}:
        person = _first_string(params, "target_person", "person", "recipient")
        if person:
            params["target_person"] = person
    if action_type in {"GUIDE_PERSON"}:
        destination = _first_string(params, "target_location", "destination", "location")
        if destination:
            params["target_location"] = destination
    if action_type in {"PICK_OBJECT", "PLACE_OBJECT", "HANDOVER_OBJECT"}:
        object_name = _first_string(params, "object_name", "object", "target_object")
        if object_name:
            params["object_name"] = object_name
            params["target_object"] = object_name
    if action_type == "SAY":
        text = _first_string(params, "text", "message")
        if text:
            params["text"] = text
    if action_type == "ASK_AND_LISTEN":
        question = _first_string(params, "question", "text", "message")
        if question:
            params["question"] = question

    return params


def _build_action_step(
    action: dict[str, Any],
    action_index: int,
    task_id: str,
    previous_step_id: str,
    default_timeout_sec: float,
    default_retries: int,
) -> ActionStep:
    """Special function: Convert one Qwen action object into a typed ROS ActionStep."""
    raw_name = action.get("skill", action.get("action_type"))
    raw_params = action.get("params", action.get("arguments", action.get("parameters", {})))
    if not isinstance(raw_params, dict):
        raise ValueError(f"Action {action_index} params must be a JSON object.")
    # Section: Route criterion-based person searches to the executor's attribute primitive.
    if _string_value(raw_name).upper() == "FIND_PERSON" and raw_params.get("criteria"):
        action_type = "FIND_PERSON_BY_ATTRIBUTE"
    else:
        action_type = _canonical_action_name(raw_name)
    params = _normalized_parameters(action_type, raw_params)

    # Section: Generate stable identifiers when the model only provides ordered actions.
    step_id = _string_value(action.get("step_id")) or f"qwen_step_{action_index + 1}_{action_type.lower()}"
    timeout_value = action.get("timeout_sec", params.get("timeout_sec", default_timeout_sec))
    retries_value = action.get("max_retries", default_retries)
    try:
        timeout_sec = float(timeout_value)
        max_retries = max(0, int(retries_value))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Action {action_index} timeout or retries are invalid.") from exc
    if timeout_sec <= 0.0:
        raise ValueError(f"Action {action_index} timeout_sec must be positive.")

    # Section: Ordered Qwen actions depend on the successful completion of the prior action by default.
    dependencies = _as_string_list(action.get("requires_success"), "requires_success")
    if not dependencies and previous_step_id:
        dependencies = [previous_step_id]

    # Section: Populate the flattened fields consumed by C++ executor clients.
    target_object = _first_string(params, "target_object", "object_name", "object")
    target_person = _first_string(params, "target_person", "person", "recipient")
    target_location = _first_string(params, "target_location", "location", "destination")
    target_category = _first_string(params, "target_category", "category", "filter")

    message = ActionStep()
    message.step_id = step_id
    message.action_type = action_type
    message.task_id = task_id
    message.task_family = "qwen_json"
    message.target_object = target_object
    message.target_person = target_person
    message.target_location = target_location
    message.target_category = target_category
    message.parameters_json = json.dumps(params, ensure_ascii=False, sort_keys=True)
    message.timeout_sec = timeout_sec
    message.max_retries = max_retries
    message.requires_success = dependencies
    message.requires_data = _as_string_list(action.get("requires_data"), "requires_data")
    message.result_variable = _string_value(action.get("result_variable"))
    return message


def parse_qwen_action_queue(
    payload_text: str,
    *,
    default_timeout_sec: float,
    default_retries: int,
) -> ActionQueue:
    """Special function: Validate Qwen JSON and create the executor's typed queue."""
    try:
        payload = json.loads(str(payload_text))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Qwen output is not valid JSON: {exc.msg}.") from exc
    if not isinstance(payload, dict):
        raise ValueError("Qwen ActionQueue output must be a JSON object.")
    if bool(payload.get("requires_clarification", False)):
        missing = payload.get("missing", [])
        raise ValueError(f"Qwen requested clarification for: {missing!r}.")

    actions = payload.get("actions")
    if not isinstance(actions, list) or not actions:
        raise ValueError("Qwen ActionQueue output must contain a non-empty actions list.")

    # Section: Keep Qwen task identity correlated with queue status and executor results.
    task_id = _string_value(payload.get("task_id", payload.get("command_id")))
    task_id = task_id or f"qwen_{uuid.uuid4().hex}"
    queue = ActionQueue()
    queue.task_id = task_id
    previous_step_id = ""
    seen_step_ids: set[str] = set()
    for action_index, raw_action in enumerate(actions):
        if not isinstance(raw_action, dict):
            raise ValueError(f"Qwen action {action_index} must be a JSON object.")
        step = _build_action_step(
            raw_action,
            action_index,
            task_id,
            previous_step_id,
            default_timeout_sec,
            default_retries,
        )
        if step.step_id in seen_step_ids:
            raise ValueError(f"Qwen queue contains duplicate step_id {step.step_id!r}.")
        # Section: Dependencies may only reference earlier steps in an ordered executable queue.
        unknown_dependencies = [
            dependency
            for dependency in step.requires_success
            if dependency not in seen_step_ids
        ]
        if unknown_dependencies:
            raise ValueError(
                f"Qwen step {step.step_id!r} references unknown or future dependencies: "
                f"{unknown_dependencies!r}."
            )
        seen_step_ids.add(step.step_id)
        queue.steps.append(step)
        previous_step_id = step.step_id
    return queue


class QwenActionQueueBridge(Node):
    """Special function: Expose Qwen JSON as a guarded typed ActionQueue pipeline."""

    def __init__(self) -> None:
        """Special function: Configure isolated Qwen topics and the shared executor client."""
        super().__init__("qwen_action_queue_bridge")

        # Section: Keep Qwen traffic separate from the local /gpsr/command owner.
        self.declare_parameter("input_topic", "/ai/qwen/action_queue_json")
        self.declare_parameter("queue_topic", "/gpsr/qwen_action_queue")
        self.declare_parameter("status_topic", "/gpsr/qwen_action_queue_status")
        self.declare_parameter("command_result_topic", "/gpsr/command_result")
        self.declare_parameter("executor_action", "/execute_action_queue")
        self.declare_parameter("scheduler_state_topic", "/gpsr/state")
        self.declare_parameter("auto_execute", False)
        self.declare_parameter("require_scheduler_idle", True)
        self.declare_parameter("default_timeout_sec", 30.0)
        self.declare_parameter("default_retries", 0)

        # Section: Serialize Qwen admission so one model result cannot overwrite another queue.
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False
        self._scheduler_state = ""

        # Section: Receive model output, publish typed inspection data, and expose execution status.
        self._input_sub = self.create_subscription(
            String,
            str(self.get_parameter("input_topic").value),
            self._input_callback,
            10,
            callback_group=self._callback_group,
        )
        self._queue_pub = self.create_publisher(
            ActionQueue,
            str(self.get_parameter("queue_topic").value),
            10,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self._command_result_pub = self.create_publisher(
            String,
            str(self.get_parameter("command_result_topic").value),
            10,
        )
        self._state_sub = self.create_subscription(
            String,
            str(self.get_parameter("scheduler_state_topic").value),
            self._scheduler_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self._executor_client = ActionClient(
            self,
            ExecuteActionQueue,
            str(self.get_parameter("executor_action").value),
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Qwen ActionQueue bridge ready; auto_execute=%s; input=%s; queue=%s"
            % (
                bool(self.get_parameter("auto_execute").value),
                self.get_parameter("input_topic").value,
                self.get_parameter("queue_topic").value,
            )
        )

    def _publish_status(self, phase: str, message: str, **extra: Any) -> None:
        """Special function: Publish one machine-readable validation or execution status."""
        payload = {"phase": str(phase), "message": str(message)}
        payload.update(extra)
        status = String()
        status.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self._status_pub.publish(status)

    def _release(self) -> None:
        """Special function: Reopen bridge admission after queue publication or execution."""
        with self._lock:
            self._busy = False

    def _publish_command_result(
        self,
        command_id: str,
        accepted: bool,
        success: bool,
        message: str,
        outcome: str = "",
    ) -> None:
        """Special function: Complete the local voice transaction with a compatible JSON result."""
        payload = {
            "accepted": bool(accepted),
            "success": bool(success),
            "command_id": str(command_id),
            "source": "qwen_action_queue",
            "message": str(message),
        }
        if outcome:
            payload["execution"] = {
                "success": bool(success),
                "outcome": str(outcome),
            }
        result = String()
        result.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self._command_result_pub.publish(result)

    def _scheduler_state_callback(self, message: String) -> None:
        """Special function: Cache the scheduler state used to avoid competing execution owners."""
        try:
            payload = json.loads(str(message.data))
            state = payload.get("state", "") if isinstance(payload, dict) else ""
        except json.JSONDecodeError:
            state = ""
        with self._lock:
            self._scheduler_state = str(state)

    def _input_callback(self, message: String) -> None:
        """Special function: Validate one Qwen queue and optionally dispatch it."""
        with self._lock:
            if self._busy:
                self._publish_status("REJECTED", "qwen_queue_busy")
                return
            self._busy = True

        try:
            queue = parse_qwen_action_queue(
                message.data,
                default_timeout_sec=float(self.get_parameter("default_timeout_sec").value),
                default_retries=int(self.get_parameter("default_retries").value),
            )
        except (TypeError, ValueError) as exc:
            self._publish_status("REJECTED", str(exc))
            if bool(self.get_parameter("auto_execute").value):
                self._publish_command_result(
                    _payload_command_id(message.data),
                    accepted=False,
                    success=False,
                    message=str(exc),
                    outcome="invalid_qwen_queue",
                )
            self._release()
            return

        # Section: Always publish the validated typed queue for inspection before execution.
        self._queue_pub.publish(queue)
        self._publish_status(
            "QUEUE_READY",
            "validated_qwen_action_queue",
            task_id=queue.task_id,
            step_count=len(queue.steps),
            auto_execute=bool(self.get_parameter("auto_execute").value),
        )

        # Section: Inspection-only mode prevents accidental duplicate execution with local GPSR.
        if not bool(self.get_parameter("auto_execute").value):
            self._release()
            return

        # Section: Require scheduler idleness when enabled to reduce local/Qwen executor races.
        if bool(self.get_parameter("require_scheduler_idle").value):
            with self._lock:
                scheduler_state = self._scheduler_state
            if scheduler_state != "WAITING_FOR_COMMAND":
                self._publish_status(
                    "EXECUTION_BLOCKED",
                    "scheduler_is_not_idle",
                    scheduler_state=scheduler_state,
                )
                self._publish_command_result(
                    queue.task_id,
                    accepted=False,
                    success=False,
                    message="scheduler_is_not_idle",
                    outcome="execution_blocked",
                )
                self._release()
                return

        if not self._executor_client.wait_for_server(timeout_sec=1.0):
            self._publish_status("EXECUTION_FAILED", "action_queue_executor_unavailable")
            self._publish_command_result(
                queue.task_id,
                accepted=False,
                success=False,
                message="action_queue_executor_unavailable",
                outcome="executor_unavailable",
            )
            self._release()
            return

        # Section: Submit the same typed action goal consumed by the local scheduler.
        goal = ExecuteActionQueue.Goal()
        goal.queue = queue
        future = self._executor_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result_future: self._goal_response_callback(result_future, queue.task_id)
        )

    def _goal_response_callback(self, future: Any, task_id: str) -> None:
        """Special function: Handle executor admission and attach the result callback."""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status("EXECUTION_FAILED", f"goal_submission_error: {exc}")
            self._publish_command_result(
                task_id,
                accepted=False,
                success=False,
                message=f"goal_submission_error: {exc}",
                outcome="goal_submission_error",
            )
            self._release()
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status("EXECUTION_FAILED", "action_queue_goal_rejected")
            self._publish_command_result(
                task_id,
                accepted=False,
                success=False,
                message="action_queue_goal_rejected",
                outcome="goal_rejected",
            )
            self._release()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result_future: self._result_callback(result_future, task_id)
        )

    def _result_callback(self, future: Any, task_id: str) -> None:
        """Special function: Publish the terminal executor result and release admission."""
        try:
            wrapped = future.result()
            result = wrapped.result
            self._publish_status(
                "EXECUTION_FINISHED",
                str(result.outcome),
                success=bool(result.success),
                outcome=str(result.outcome),
                result_count=len(result.results),
            )
            self._publish_command_result(
                task_id,
                accepted=True,
                success=bool(result.success),
                message=str(result.outcome),
                outcome=str(result.outcome),
            )
        except Exception as exc:
            self._publish_status("EXECUTION_FAILED", f"result_error: {exc}")
            self._publish_command_result(
                task_id,
                accepted=False,
                success=False,
                message=f"result_error: {exc}",
                outcome="result_error",
            )
        finally:
            self._release()


def main(args: list[str] | None = None) -> None:
    """Special function: Run the Qwen ActionQueue bridge node."""
    rclpy.init(args=args)
    node = QwenActionQueueBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
