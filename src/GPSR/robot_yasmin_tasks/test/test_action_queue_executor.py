"""Section: Action queue executor tests."""

from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.action_queue_executor_sm import (
    ActionQueueExecutorSM,
    QueueAdmissionGuard,
    SelectNextActionState,
)
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import make_skill_result
from robot_yasmin_tasks.gpsr.models import summarize_action_queue_failure
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, FATAL_FAILURE, FAILED, INVALID_INPUT, SKILL_UNAVAILABLE, SUCCESS, TIMEOUT


class SequenceSkill:
    """Special function: Return configured outcomes in sequence."""
    calls: list[str] = []
    outcomes: dict[str, list[str]] = {}
    def __init__(self, action: str) -> None:
        """Special function: Store action name."""
        self.action = action
    def __call__(self, blackboard: Blackboard) -> str:
        """Special function: Store result and return next outcome."""
        self.calls.append(self.action)
        outcome = self.outcomes[self.action].pop(0)
        blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(self.action, outcome, recoverable=outcome == FAILED, fatal=outcome == FATAL_FAILURE).to_dict()
        return outcome


def factory(action: str):
    """Special function: Build sequence skill factory."""
    return lambda value=action: SequenceSkill(value)


def test_empty_queue_success() -> None:
    """Special function: Empty queue completes successfully."""
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = []
    assert ActionQueueExecutorSM({})(blackboard) == SUCCESS


def test_failure_summary_preserves_exact_skill_reason_and_details() -> None:
    """Special function: Terminal diagnostics identify the action and underlying client reason."""
    # Section: Model the navigation failure that previously appeared only as fatal_failure.
    message = summarize_action_queue_failure(
        "fatal_failure",
        [{
            "action_type": "NAVIGATE",
            "outcome": "fatal_failure",
            "message": "NavigateNamedPose action unavailable: /nav/navigate_named_pose",
            "result": {"target": "living_room"},
        }],
    )

    # Section: Require both the human-readable reason and structured target evidence.
    assert "action=NAVIGATE" in message
    assert "outcome=fatal_failure" in message
    assert "NavigateNamedPose action unavailable: /nav/navigate_named_pose" in message
    assert '"target": "living_room"' in message


def test_failure_summary_reports_missing_executor_result() -> None:
    """Special function: Abnormal empty results still explain the diagnostic limitation."""
    message = summarize_action_queue_failure("fatal_failure", [])
    assert "outcome=fatal_failure" in message
    assert "executor returned no failed SkillResult" in message


def test_select_next_action_normalizes_and_stores_current_action() -> None:
    """Special function: Regression guard keeps selection code inside execute()."""
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{
        "action_type": "find_object",
        "target_object": "bottle",
        "parameters": {"search_scope": "GLOBAL"},
    }]
    blackboard[bb.CURRENT_ACTION_INDEX] = 0
    assert SelectNextActionState().execute(blackboard) == SUCCESS
    selected = blackboard[bb.CURRENT_ACTION]
    assert selected.action_type == "FIND_OBJECT"
    assert selected.target_object == "bottle"
    assert selected.parameters["search_scope"] == "GLOBAL"


def test_legacy_report_normalizes_to_say_with_warning() -> None:
    """Special function: Temporary aliases normalize once at queue admission."""
    warnings: list[str] = []
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "REPORT", "parameters": {"text": "Done."}}]
    blackboard[bb.CURRENT_ACTION_INDEX] = 0
    state = SelectNextActionState(warning_logger=warnings.append)
    assert state.execute(blackboard) == SUCCESS
    assert blackboard[bb.CURRENT_ACTION].action_type == "SAY"
    assert blackboard[bb.CURRENT_ACTION].parameters["text"] == "Done."
    assert "REPORT" in warnings[0] and "SAY" in warnings[0]


def test_disabled_legacy_pick_is_rejected_before_dispatch() -> None:
    """Special function: Legacy aliases cannot bypass production capability policy."""
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "PICK", "target_object": "coke"}]
    blackboard[bb.CURRENT_ACTION_INDEX] = 0
    assert SelectNextActionState().execute(blackboard) == INVALID_INPUT
    assert "PICK_OBJECT" in blackboard[bb.FAILURE_REASON]


def test_multiple_actions_execute_in_order() -> None:
    """Special function: Multiple actions run in queue order."""
    SequenceSkill.calls = []
    SequenceSkill.outcomes = {"NAVIGATE": [SUCCESS], "SAY": [SUCCESS]}
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "NAVIGATE"}, {"action_type": "SAY"}]
    assert ActionQueueExecutorSM({"NAVIGATE": factory("NAVIGATE"), "SAY": factory("SAY")})(blackboard) == SUCCESS
    assert SequenceSkill.calls == ["NAVIGATE", "SAY"]


def test_failure_triggers_recovery_retry() -> None:
    """Special function: Recoverable failure retries once."""
    SequenceSkill.calls = []
    SequenceSkill.outcomes = {"NAVIGATE": [FAILED, SUCCESS]}
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "NAVIGATE"}]
    assert ActionQueueExecutorSM({"NAVIGATE": factory("NAVIGATE")})(blackboard) == SUCCESS
    assert SequenceSkill.calls == ["NAVIGATE", "NAVIGATE"]


class RecordingResourceManager:
    """Special function: Record managed generation ordering for executor tests."""

    def __init__(self, events: list[str], fail_stop: bool = False) -> None:
        """Special function: Store shared events and optional teardown failure."""
        self.events = events
        self.fail_stop = fail_stop
        self.generation_index = 0

    def start(self, blackboard: Blackboard, action_type: str) -> tuple[bool, str]:
        """Special function: Create a deterministic generation only for COUNT_OBJECTS."""
        if action_type != "COUNT_OBJECTS":
            blackboard[bb.ACTIVE_SKILL_TARGET] = ""
            blackboard[bb.ACTIVE_SKILL_GENERATION] = ""
            return True, "persistent action"
        self.generation_index += 1
        generation = f"generation-{self.generation_index}"
        blackboard[bb.ACTIVE_SKILL_TARGET] = "gpsr_count_objects"
        blackboard[bb.ACTIVE_SKILL_GENERATION] = generation
        self.events.append(f"start:{generation}")
        return True, "ready"

    def stop(self, blackboard: Blackboard) -> tuple[bool, str]:
        """Special function: Record and optionally fail the current teardown barrier."""
        generation = str(bb_get(blackboard, bb.ACTIVE_SKILL_GENERATION, ""))
        if not generation:
            return True, "persistent action"
        self.events.append(f"stop:{generation}")
        if self.fail_stop:
            return False, "graph still present"
        blackboard[bb.ACTIVE_SKILL_TARGET] = ""
        blackboard[bb.ACTIVE_SKILL_GENERATION] = ""
        return True, "stopped"


class EventSkill:
    """Special function: Emit execution events and configured outcomes."""

    def __init__(
        self,
        action_type: str,
        events: list[str],
        outcomes: list[str],
        cancel_flag: dict[str, bool] | None = None,
        raise_exception: bool = False,
    ) -> None:
        """Special function: Store deterministic skill behavior."""
        self.action_type = action_type
        self.events = events
        self.outcomes = outcomes
        self.cancel_flag = cancel_flag
        self.raise_exception = raise_exception

    def __call__(self, blackboard: Blackboard) -> str:
        """Special function: Execute one deterministic test skill call."""
        self.events.append(f"execute:{self.action_type}")
        if self.raise_exception:
            raise RuntimeError("test exception")
        if self.cancel_flag is not None:
            self.cancel_flag["requested"] = True
        outcome = self.outcomes.pop(0)
        blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
            self.action_type,
            outcome,
            recoverable=outcome == FAILED,
            fatal=outcome == FATAL_FAILURE,
        ).to_dict()
        return outcome


def test_managed_teardown_finishes_before_next_action() -> None:
    """Special function: Commit COUNT_OBJECTS only after stop and then execute SAY."""
    events: list[str] = []
    resources = RecordingResourceManager(events)
    factories = {
        "COUNT_OBJECTS": lambda: EventSkill("COUNT_OBJECTS", events, [SUCCESS]),
        "SAY": lambda: EventSkill("SAY", events, [SUCCESS]),
    }
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "COUNT_OBJECTS"}, {"action_type": "SAY"}]
    assert ActionQueueExecutorSM(factories, resources)(blackboard) == SUCCESS
    assert events == ["start:generation-1", "execute:COUNT_OBJECTS", "stop:generation-1", "execute:SAY"]
    assert blackboard[bb.CURRENT_ACTION_INDEX] == 2


def test_find_object_dispatch_occurs_only_after_active_verification() -> None:
    """Special function: Preserve the lifecycle ACTIVE barrier before skill dispatch."""
    events: list[str] = []

    class ActiveBarrierResources:
        """Special function: Record manager preparation and final cleanup barriers."""

        def start(self, _blackboard, action_type):
            """Special function: Model configure/activate and ACTIVE verification success."""
            events.extend([f"prepare:{action_type}", f"verify-active:{action_type}"])
            return True, "ACTIVE"

        def stop(self, _blackboard):
            """Special function: Model teardown and UNCONFIGURED verification success."""
            events.extend(["cleanup:FIND_OBJECT", "verify-unconfigured:FIND_OBJECT"])
            return True, "UNCONFIGURED"

    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "FIND_OBJECT"}]
    skill = EventSkill("FIND_OBJECT", events, [SUCCESS])

    assert ActionQueueExecutorSM(
        {"FIND_OBJECT": lambda: skill},
        ActiveBarrierResources(),
    )(blackboard) == SUCCESS
    assert events == [
        "prepare:FIND_OBJECT",
        "verify-active:FIND_OBJECT",
        "execute:FIND_OBJECT",
        "cleanup:FIND_OBJECT",
        "verify-unconfigured:FIND_OBJECT",
    ]


def test_teardown_failure_does_not_advance_queue() -> None:
    """Special function: Failed teardown converts success to fatal without index commit."""
    events: list[str] = []
    resources = RecordingResourceManager(events, fail_stop=True)
    factories = {"COUNT_OBJECTS": lambda: EventSkill("COUNT_OBJECTS", events, [SUCCESS])}
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "COUNT_OBJECTS"}]
    assert ActionQueueExecutorSM(factories, resources)(blackboard) == FATAL_FAILURE
    assert blackboard[bb.CURRENT_ACTION_INDEX] == 0
    assert events == ["start:generation-1", "execute:COUNT_OBJECTS", "stop:generation-1"]


def test_missing_launch_pipeline_reports_skill_unavailable() -> None:
    """Special function: Failed lifecycle preparation exposes a capability-specific result."""
    class MissingPipelineManager:
        """Special function: Reject resource startup as if its launch switch were false."""

        def start(self, _blackboard, _action_type):
            """Special function: Return a confirmed absent-capability failure."""
            return False, "required lifecycle node is unavailable"

        def stop(self, _blackboard):
            """Special function: Confirm that no partial resource generation remains."""
            return True, "nothing prepared"

    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "FIND_PERSON_BY_ATTRIBUTE"}]
    outcome = ActionQueueExecutorSM({}, MissingPipelineManager())(blackboard)
    assert outcome == FATAL_FAILURE
    assert blackboard[bb.LAST_SKILL_RESULT]["outcome"] == SKILL_UNAVAILABLE
    assert blackboard[bb.CURRENT_ACTION_INDEX] == 0


def test_recoverable_retry_uses_fresh_generation() -> None:
    """Special function: Recovery happens after teardown and relaunches a new generation."""
    events: list[str] = []
    resources = RecordingResourceManager(events)
    skill = EventSkill("COUNT_OBJECTS", events, [FAILED, SUCCESS])
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "COUNT_OBJECTS"}]
    assert ActionQueueExecutorSM({"COUNT_OBJECTS": lambda: skill}, resources)(blackboard) == SUCCESS
    assert events == [
        "start:generation-1",
        "execute:COUNT_OBJECTS",
        "stop:generation-1",
        "start:generation-2",
        "execute:COUNT_OBJECTS",
        "stop:generation-2",
    ]


def test_exception_still_runs_teardown() -> None:
    """Special function: A skill exception follows the same teardown barrier."""
    events: list[str] = []
    resources = RecordingResourceManager(events)
    factories = {
        "COUNT_OBJECTS": lambda: EventSkill(
            "COUNT_OBJECTS", events, [SUCCESS], raise_exception=True
        )
    }
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "COUNT_OBJECTS"}]
    assert ActionQueueExecutorSM(factories, resources)(blackboard) == FATAL_FAILURE
    assert events == ["start:generation-1", "execute:COUNT_OBJECTS", "stop:generation-1"]
    assert blackboard[bb.CURRENT_ACTION_INDEX] == 0


def test_timeout_stops_generation_before_recovery_retry() -> None:
    """Special function: Timeout cleanup completes before a fresh retry generation starts."""
    events: list[str] = []
    resources = RecordingResourceManager(events)
    skill = EventSkill("COUNT_OBJECTS", events, [TIMEOUT, SUCCESS])
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "COUNT_OBJECTS"}]
    assert ActionQueueExecutorSM({"COUNT_OBJECTS": lambda: skill}, resources)(blackboard) == SUCCESS
    assert events == [
        "start:generation-1",
        "execute:COUNT_OBJECTS",
        "stop:generation-1",
        "start:generation-2",
        "execute:COUNT_OBJECTS",
        "stop:generation-2",
    ]


def test_cancellation_waits_for_teardown_and_does_not_commit() -> None:
    """Special function: Cancellation becomes terminal only after managed stop succeeds."""
    events: list[str] = []
    cancel_flag = {"requested": False}
    resources = RecordingResourceManager(events)
    factories = {
        "COUNT_OBJECTS": lambda: EventSkill(
            "COUNT_OBJECTS", events, [SUCCESS], cancel_flag=cancel_flag
        )
    }
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "COUNT_OBJECTS"}]
    blackboard[bb.CANCEL_REQUESTED] = lambda: cancel_flag["requested"]
    assert ActionQueueExecutorSM(factories, resources)(blackboard) == CANCELED
    assert events == ["start:generation-1", "execute:COUNT_OBJECTS", "stop:generation-1"]
    assert blackboard[bb.CURRENT_ACTION_INDEX] == 0


def test_unconfirmed_startup_cleanup_is_fatal_not_canceled() -> None:
    """Special function: Never report canceled while startup teardown is uncertain."""
    cancel_flag = {"requested": False}

    # Section: Simulate cancellation after the supervisor accepted a managed start.
    class UnconfirmedStartupManager:
        """Special function: Model an accepted start whose cleanup result was lost."""

        def start(self, blackboard, _action_type):
            """Special function: Mark cleanup uncertain before surfacing cancellation."""
            blackboard[bb.START_CLEANUP_CONFIRMED] = False
            cancel_flag["requested"] = True
            return False, "ManagedProcess startup cleanup unconfirmed"

        def stop(self, _blackboard):
            """Special function: No generation is available for a separate stop request."""
            return True, "no retained generation"

    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{"action_type": "COUNT_OBJECTS"}]
    blackboard[bb.CANCEL_REQUESTED] = lambda: cancel_flag["requested"]
    executor = ActionQueueExecutorSM({}, UnconfirmedStartupManager())
    assert executor(blackboard) == FATAL_FAILURE
    assert blackboard[bb.CURRENT_ACTION_INDEX] == 0


def test_queue_admission_rejects_concurrent_goal() -> None:
    """Special function: Only one top-level queue owns the executor at a time."""
    admission = QueueAdmissionGuard()
    assert admission.try_admit() is True
    assert admission.try_admit() is False
    admission.release()
    assert admission.try_admit() is True


def test_missing_prerequisite_never_prepares_or_executes_skill() -> None:
    """Special function: A missing dependency fails before lifecycle preparation."""
    events: list[str] = []
    resources = RecordingResourceManager(events)
    blackboard = Blackboard()
    blackboard[bb.ACTION_QUEUE] = [{
        "step_id": "pick",
        "action_type": "COUNT_OBJECTS",
        "requires_success": ["find"],
        "max_retries": 0,
    }]
    outcome = ActionQueueExecutorSM(
        {"COUNT_OBJECTS": lambda: EventSkill("COUNT_OBJECTS", events, [SUCCESS])},
        resources,
    )(blackboard)
    assert outcome == FATAL_FAILURE
    assert events == []
    assert blackboard[bb.CURRENT_ACTION_INDEX] == 0
