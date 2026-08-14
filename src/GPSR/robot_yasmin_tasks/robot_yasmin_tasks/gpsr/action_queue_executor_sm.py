"""Section: GPSR action queue executor and ROS action server."""

from __future__ import annotations

import json
import threading
from collections.abc import Callable
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from yasmin import Blackboard, State, StateMachine
from yasmin_viewer import YasminViewerPub

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.action_dispatcher_state import ActionDispatcherState
from robot_yasmin_tasks.gpsr.action_types import ALLOWED_ACTION_TYPES
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import make_skill_result, normalize_action_step
from robot_yasmin_tasks.gpsr.outcomes import (
    CANCELED,
    FATAL_FAILURE,
    FAILED,
    INVALID_INPUT,
    RECOVERABLE_FAILURE,
    SUCCESS,
    is_recoverable,
)
from robot_yasmin_tasks.ros_clients.lifecycle_client import NullSkillResourceManager
from robot_yasmin_tasks.skills.recovery_skill_sm import RecoverySkillSM


class QueueAdmissionGuard:
    """Special function: Serialize top-level queue ownership across executor threads."""

    def __init__(self) -> None:
        """Special function: Initialize an idle guarded admission flag."""
        self._lock = threading.Lock()
        self._active = False

    def try_admit(self) -> bool:
        """Special function: Atomically admit only the first concurrent queue."""
        with self._lock:
            if self._active:
                return False
            self._active = True
            return True

    def release(self) -> None:
        """Special function: Release queue ownership after terminal cleanup."""
        with self._lock:
            self._active = False


class CheckQueueState(State):
    """Special function: Validate queue and initialize bookkeeping."""

    def __init__(self) -> None:
        """Special function: Declare queue check outcomes."""
        super().__init__([SUCCESS, "has_actions", INVALID_INPUT])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Prepare queue execution blackboard state."""
        steps = list(
            getattr(
                bb_get(blackboard, bb.ACTION_QUEUE, []),
                "steps",
                bb_get(blackboard, bb.ACTION_QUEUE, []),
            )
        )
        blackboard[bb.ACTION_QUEUE] = steps
        blackboard[bb.CURRENT_ACTION_INDEX] = int(
            bb_get(blackboard, bb.CURRENT_ACTION_INDEX, 0) or 0
        )
        blackboard[bb.SKILL_RESULTS] = list(bb_get(blackboard, bb.SKILL_RESULTS, []))
        blackboard[bb.RECOVERY_ATTEMPTS] = dict(
            bb_get(blackboard, bb.RECOVERY_ATTEMPTS, {})
        )
        blackboard[bb.COMPLETED_STEP_IDS] = set(
            bb_get(blackboard, bb.COMPLETED_STEP_IDS, set())
        )
        blackboard[bb.RESULT_VARIABLES] = dict(
            bb_get(blackboard, bb.RESULT_VARIABLES, {})
        )
        blackboard[bb.ACTIVE_LIFECYCLE_SKILL] = ""
        blackboard[bb.ACTIVE_SKILL_TARGET] = ""
        blackboard[bb.ACTIVE_SKILL_GENERATION] = ""
        return SUCCESS if not steps else "has_actions"


class SelectNextActionState(State):
    """Special function: Select the next action from the queue."""

    def __init__(self) -> None:
        """Special function: Declare selection outcomes."""
        super().__init__([SUCCESS, INVALID_INPUT])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Store CURRENT_ACTION from the committed queue index."""
        queue = list(bb_get(blackboard, bb.ACTION_QUEUE, []))
        index = int(bb_get(blackboard, bb.CURRENT_ACTION_INDEX, 0) or 0)
        # Section: Reject an inconsistent queue cursor before indexing the queue.
        if index < 0 or index >= len(queue):
            blackboard[bb.FAILURE_REASON] = (
                f"Action queue index {index} is outside queue length {len(queue)}."
            )
            return INVALID_INPUT
        action = normalize_action_step(queue[index])
        blackboard[bb.CURRENT_ACTION] = action
        if action.action_type not in ALLOWED_ACTION_TYPES:
            blackboard[bb.FAILURE_REASON] = f"Action type {action.action_type} is not allowed."
            return INVALID_INPUT
        return SUCCESS


class CheckPrerequisitesState(State):
    """Special function: Validate completed-step and result-data dependencies before prepare."""

    def __init__(self) -> None:
        """Special function: Declare prerequisite routing outcomes."""
        super().__init__([SUCCESS, "skip_dispatch"])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Reject a dependent action when required success or data is absent."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        completed = set(bb_get(blackboard, bb.COMPLETED_STEP_IDS, set()))
        result_variables = dict(bb_get(blackboard, bb.RESULT_VARIABLES, {}))
        missing_success = [
            step_id for step_id in action.requires_success
            if step_id not in completed
        ]
        missing_data = [
            key for key in action.requires_data
            if key not in result_variables and key not in blackboard
        ]
        if not missing_success and not missing_data:
            return SUCCESS

        # Section: Store a structured fatal result and use the normal cleanup/result barrier.
        message = (
            f"Prerequisites missing for {action.step_id or action.action_type}: "
            f"requires_success={missing_success}, requires_data={missing_data}"
        )
        blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
            action.action_type,
            INVALID_INPUT,
            message,
            {
                "missing_requires_success": missing_success,
                "missing_requires_data": missing_data,
            },
            fatal=True,
        ).to_dict()
        blackboard[bb.DISPATCH_OUTCOME] = INVALID_INPUT
        return "skip_dispatch"


class StartSkillResourcesState(State):
    """Special function: Acquire and start ephemeral resources for the selected action."""

    def __init__(self, resource_manager: Any) -> None:
        """Special function: Store the logical resource manager dependency."""
        super().__init__([SUCCESS, "skip_dispatch"])
        self.resource_manager = resource_manager

    @staticmethod
    def _cancel_requested(blackboard: Blackboard) -> bool:
        """Special function: Check top-level queue cancellation without exposing the goal handle."""
        callback = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        return bool(callback()) if callable(callback) else False

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Start a fresh managed generation or fail closed."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        if self._cancel_requested(blackboard):
            result = make_skill_result(
                action.action_type,
                CANCELED,
                "Queue canceled before skill resource startup.",
                fatal=True,
            )
            blackboard[bb.LAST_SKILL_RESULT] = result.to_dict()
            blackboard[bb.DISPATCH_OUTCOME] = CANCELED
            return "skip_dispatch"
        try:
            started, message = self.resource_manager.start(blackboard, action.action_type)
        except Exception as exc:
            started, message = False, f"Managed resource startup exception: {exc}"
        if started:
            return SUCCESS
        cleanup_confirmed = bool(
            bb_get(blackboard, bb.START_CLEANUP_CONFIRMED, True)
        )
        outcome = (
            CANCELED
            if self._cancel_requested(blackboard) and cleanup_confirmed
            else FATAL_FAILURE
        )
        result = make_skill_result(
            action.action_type,
            outcome,
            message,
            fatal=True,
        )
        blackboard[bb.LAST_SKILL_RESULT] = result.to_dict()
        blackboard[bb.DISPATCH_OUTCOME] = outcome
        return "skip_dispatch"


class DispatchActionState(State):
    """Special function: Dispatch and classify one action outcome."""

    def __init__(self, dispatcher: ActionDispatcherState) -> None:
        """Special function: Store the existing dispatcher state."""
        super().__init__([SUCCESS, RECOVERABLE_FAILURE, FATAL_FAILURE])
        self.dispatcher = dispatcher

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Execute dispatch while converting exceptions into cleanup-safe failure."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        try:
            outcome = self.dispatcher.execute(blackboard)
        except Exception as exc:
            outcome = FATAL_FAILURE
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
                action.action_type,
                FATAL_FAILURE,
                f"Skill execution exception: {exc}",
                fatal=True,
            ).to_dict()
            blackboard[bb.DISPATCH_OUTCOME] = outcome
        if outcome == SUCCESS:
            return SUCCESS
        if is_recoverable(outcome):
            return RECOVERABLE_FAILURE
        return FATAL_FAILURE


class StopSkillResourcesState(State):
    """Special function: Enforce teardown before any result is committed."""

    def __init__(self, resource_manager: Any) -> None:
        """Special function: Store the logical resource manager dependency."""
        super().__init__([SUCCESS])
        self.resource_manager = resource_manager

    @staticmethod
    def _cancel_requested(blackboard: Blackboard) -> bool:
        """Special function: Check whether cancellation arrived during skill execution."""
        callback = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        return bool(callback()) if callable(callback) else False

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Stop the active generation and fail closed on uncertain teardown."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        prior_result = bb_get(
            blackboard,
            bb.LAST_SKILL_RESULT,
            make_skill_result(action.action_type, FAILED).to_dict(),
        )
        try:
            stopped, message = self.resource_manager.stop(blackboard)
        except Exception as exc:
            stopped, message = False, f"Managed resource teardown exception: {exc}"
        if not stopped:
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
                action.action_type,
                FATAL_FAILURE,
                f"FAILED_TEARDOWN: {message}",
                {"skill_result_before_teardown": prior_result},
                fatal=True,
            ).to_dict()
            blackboard[bb.DISPATCH_OUTCOME] = FATAL_FAILURE
            return SUCCESS
        prior_outcome = str(bb_get(blackboard, bb.DISPATCH_OUTCOME, FAILED))
        cleanup_confirmed = bool(
            bb_get(blackboard, bb.START_CLEANUP_CONFIRMED, True)
        )
        if (
            self._cancel_requested(blackboard)
            and prior_outcome != FATAL_FAILURE
            and cleanup_confirmed
        ):
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
                action.action_type,
                CANCELED,
                "Queue canceled after managed skill teardown completed.",
                {"skill_result_before_cancel": prior_result},
                fatal=True,
            ).to_dict()
            blackboard[bb.DISPATCH_OUTCOME] = CANCELED
        return SUCCESS


class StoreSkillResultState(State):
    """Special function: Commit a result only after the teardown barrier succeeds."""

    def __init__(self) -> None:
        """Special function: Declare result routing outcomes."""
        super().__init__([SUCCESS, RECOVERABLE_FAILURE, FATAL_FAILURE, CANCELED])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Append the result and increment only a fully cleaned success."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        outcome = str(bb_get(blackboard, bb.DISPATCH_OUTCOME, FAILED))
        result = bb_get(
            blackboard,
            bb.LAST_SKILL_RESULT,
            make_skill_result(action.action_type, outcome).to_dict(),
        )
        results = list(bb_get(blackboard, bb.SKILL_RESULTS, []))
        results.append(result)
        blackboard[bb.SKILL_RESULTS] = results
        if outcome == SUCCESS:
            # Section: Commit dependency metadata only after lifecycle cleanup succeeds.
            if action.step_id:
                completed = set(bb_get(blackboard, bb.COMPLETED_STEP_IDS, set()))
                completed.add(action.step_id)
                blackboard[bb.COMPLETED_STEP_IDS] = completed
            if action.result_variable:
                result_variables = dict(bb_get(blackboard, bb.RESULT_VARIABLES, {}))
                result_value = dict(result).get("result", {})
                result_variables[action.result_variable] = result_value
                blackboard[bb.RESULT_VARIABLES] = result_variables
                blackboard[action.result_variable] = result_value
            blackboard[bb.CURRENT_ACTION_INDEX] = int(
                bb_get(blackboard, bb.CURRENT_ACTION_INDEX, 0) or 0
            ) + 1
            return SUCCESS
        if outcome == CANCELED:
            return CANCELED
        return RECOVERABLE_FAILURE if is_recoverable(outcome) else FATAL_FAILURE


class CheckCompletionState(State):
    """Special function: Check whether every queued action has been committed."""

    def __init__(self) -> None:
        """Special function: Declare completion outcomes."""
        super().__init__([SUCCESS, "continue"])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Compare committed queue index to queue length."""
        index = int(bb_get(blackboard, bb.CURRENT_ACTION_INDEX, 0) or 0)
        queue_length = len(list(bb_get(blackboard, bb.ACTION_QUEUE, [])))
        return SUCCESS if index >= queue_length else "continue"


class RecoveryState(State):
    """Special function: Run deterministic bounded recovery after complete cleanup."""

    def __init__(self) -> None:
        """Special function: Declare recovery outcomes."""
        super().__init__([SUCCESS, FATAL_FAILURE])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Retry the current action once with a future fresh generation."""
        return SUCCESS if RecoverySkillSM()(blackboard) == SUCCESS else FATAL_FAILURE


class ActionQueueExecutorSM(StateMachine):
    """Special function: Execute symbolic GPSR queues with managed-resource barriers."""

    def __init__(
        self,
        factories: dict[str, Callable[[], Any]] | None = None,
        resource_manager: Any | None = None,
    ) -> None:
        """Special function: Extend the existing graph without replacing its dispatcher."""
        super().__init__(
            outcomes=[SUCCESS, FAILED, INVALID_INPUT, FATAL_FAILURE, CANCELED],
            handle_sigint=True,
        )
        dispatcher = ActionDispatcherState(factories)
        resources = resource_manager or NullSkillResourceManager()

        # Section: Start and stop states surround every dispatch, including retries.
        self.add_state(
            "CHECK_QUEUE",
            CheckQueueState(),
            transitions={SUCCESS: SUCCESS, "has_actions": "SELECT_NEXT_ACTION", INVALID_INPUT: INVALID_INPUT},
        )
        self.add_state(
            "SELECT_NEXT_ACTION",
            SelectNextActionState(),
            transitions={SUCCESS: "CHECK_PREREQUISITES", INVALID_INPUT: INVALID_INPUT},
        )
        self.add_state(
            "CHECK_PREREQUISITES",
            CheckPrerequisitesState(),
            transitions={SUCCESS: "START_SKILL_RESOURCES", "skip_dispatch": "STOP_SKILL_RESOURCES"},
        )
        self.add_state(
            "START_SKILL_RESOURCES",
            StartSkillResourcesState(resources),
            transitions={SUCCESS: "DISPATCH_ACTION", "skip_dispatch": "STOP_SKILL_RESOURCES"},
        )
        self.add_state(
            "DISPATCH_ACTION",
            DispatchActionState(dispatcher),
            transitions={
                SUCCESS: "STOP_SKILL_RESOURCES",
                RECOVERABLE_FAILURE: "STOP_SKILL_RESOURCES",
                FATAL_FAILURE: "STOP_SKILL_RESOURCES",
            },
        )
        self.add_state(
            "STOP_SKILL_RESOURCES",
            StopSkillResourcesState(resources),
            transitions={SUCCESS: "STORE_SKILL_RESULT"},
        )
        self.add_state(
            "STORE_SKILL_RESULT",
            StoreSkillResultState(),
            transitions={
                SUCCESS: "CHECK_COMPLETION",
                RECOVERABLE_FAILURE: "RECOVERY",
                FATAL_FAILURE: FATAL_FAILURE,
                CANCELED: CANCELED,
            },
        )
        self.add_state(
            "CHECK_COMPLETION",
            CheckCompletionState(),
            transitions={SUCCESS: SUCCESS, "continue": "SELECT_NEXT_ACTION"},
        )
        self.add_state(
            "RECOVERY",
            RecoveryState(),
            transitions={SUCCESS: "START_SKILL_RESOURCES", FATAL_FAILURE: FATAL_FAILURE},
        )


def result_to_msg(msg_type: Any, data: dict[str, Any]) -> Any:
    """Special function: Convert a blackboard result dictionary to a ROS message."""
    msg = msg_type()
    msg.action_type = str(data.get("action_type", ""))
    msg.outcome = str(data.get("outcome", ""))
    msg.message = str(data.get("message", ""))
    msg.result_json = json.dumps(data.get("result", {}), sort_keys=True)
    msg.recoverable = bool(data.get("recoverable", False))
    msg.fatal = bool(data.get("fatal", False))
    return msg


def main() -> None:
    """Special function: Run the single-admission ExecuteActionQueue action server."""
    from robot_ai_interfaces.action import ExecuteActionQueue
    from robot_ai_interfaces.msg import SkillResult
    from robot_yasmin_tasks.gpsr.action_dispatcher_state import ACTION_TYPE_TO_SKILL
    from robot_yasmin_tasks.gpsr.action_types import (
        ASK,
        COUNT_OBJECTS,
        COUNT_PEOPLE,
        FIND_OBJECT,
        FIND_PERSON,
        GREET,
        NAVIGATE,
        REPORT,
        RETURN_TO_INSTRUCTION_POINT,
        SAY,
    )
    from robot_yasmin_tasks.ros_clients.dialogue_clients import DialogueClients
    from robot_yasmin_tasks.ros_clients.find_person_clients import FindPersonClients
    from robot_yasmin_tasks.ros_clients.lifecycle_client import LifecycleSkillResourceClient
    from robot_yasmin_tasks.ros_clients.nav2_client import Nav2Client
    from robot_yasmin_tasks.ros_clients.perception_clients import PerceptionClients
    from robot_yasmin_tasks.ros_clients.speech_clients import SpeechClients
    from robot_yasmin_tasks.skills.count_objects_skill_sm import CountObjectsSkillSM
    from robot_yasmin_tasks.skills.count_people_skill_sm import CountPeopleSkillSM
    from robot_yasmin_tasks.skills.dialogue_skill_sm import DialogueSkillSM
    from robot_yasmin_tasks.skills.find_object_skill_sm import FindObjectSkillSM
    from robot_yasmin_tasks.skills.find_person_skill_sm import FindPersonSkillSM
    from robot_yasmin_tasks.skills.greet_person_skill_sm import GreetPersonClient, GreetPersonSkillSM
    from robot_yasmin_tasks.skills.navigation_skill_sm import NavigationSkillSM
    from robot_yasmin_tasks.skills.report_skill_sm import ReportClient, ReportSkillSM
    from robot_yasmin_tasks.skills.return_instruction_point_sm import ReturnInstructionPointClient, ReturnInstructionPointSM
    from robot_yasmin_tasks.skills.speech_skill_sm import SpeechSkillSM

    # Section: Persistent executor clients remain shared while skill stacks are ephemeral.
    rclpy.init()
    node = rclpy.create_node("gpsr_executor")
    speech = SpeechClients(node=node, simulate=False)
    dialogue = DialogueClients(node=node, simulate=False, speech_client=speech)
    perception = PerceptionClients(node=node, simulate=False)
    # Section: FIND_PERSON reuses the persistent face identity and shared TTS/STT clients.
    find_person = FindPersonClients(node=node, simulate=False, dialogue=dialogue)
    navigation = Nav2Client(node=node, simulate=False)
    resources = LifecycleSkillResourceClient(node=node)
    factories = dict(ACTION_TYPE_TO_SKILL)
    factories[SAY] = lambda: SpeechSkillSM(speech)
    factories[ASK] = lambda: DialogueSkillSM(dialogue)
    factories[GREET] = lambda: GreetPersonSkillSM(GreetPersonClient(speech))
    factories[REPORT] = lambda: ReportSkillSM(ReportClient(speech))
    factories[NAVIGATE] = lambda: NavigationSkillSM(navigation)
    factories[RETURN_TO_INSTRUCTION_POINT] = lambda: ReturnInstructionPointSM(
        ReturnInstructionPointClient(navigation)
    )
    factories[FIND_OBJECT] = lambda: FindObjectSkillSM(node=node)
    factories[FIND_PERSON] = lambda: FindPersonSkillSM(find_person, perception)
    factories[COUNT_OBJECTS] = lambda: CountObjectsSkillSM(perception)
    factories[COUNT_PEOPLE] = lambda: CountPeopleSkillSM(perception)
    sm = ActionQueueExecutorSM(factories, resources)
    YasminViewerPub(sm, "GPSR_ACTION_QUEUE_EXECUTOR")

    # Section: Goal admission is serialized independently of executor thread count.
    admission = QueueAdmissionGuard()

    def goal_callback(_goal_request):
        """Special function: Reject a second queue while one queue owns the executor."""
        if not admission.try_admit():
            node.get_logger().warn("Rejected concurrent /execute_action_queue goal.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(_goal_handle):
        """Special function: Accept cancellation for cleanup-aware state routing."""
        return CancelResponse.ACCEPT

    def execute(goal_handle):
        """Special function: Execute one admitted queue and release admission after cleanup."""
        blackboard = Blackboard()
        blackboard[bb.ACTION_QUEUE] = list(goal_handle.request.queue.steps)
        blackboard[bb.CANCEL_REQUESTED] = lambda: goal_handle.is_cancel_requested
        try:
            outcome = sm(blackboard)
            result = ExecuteActionQueue.Result()
            result.success = outcome == SUCCESS
            result.outcome = outcome
            result.results = [
                result_to_msg(SkillResult, item)
                for item in list(bb_get(blackboard, bb.SKILL_RESULTS, []))
            ]
            if outcome == SUCCESS:
                goal_handle.succeed()
            elif outcome == CANCELED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        finally:
            admission.release()

    # Section: The persistent action server exposes the existing public queue API.
    action_server = ActionServer(
        node,
        ExecuteActionQueue,
        "execute_action_queue",
        execute_callback=execute,
        goal_callback=goal_callback,
        cancel_callback=cancel_callback,
    )
    node.get_logger().info("GPSR executor waiting for one goal at a time on /execute_action_queue.")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        action_server.destroy()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
