"""Section: Central GPSR lifecycle-manager client tests."""

# Section: Test fixtures, deterministic fakes, and regression scenarios.

import json
from types import SimpleNamespace

from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.ros_clients.lifecycle_client import LifecycleSkillResourceClient


class ImmediateFuture:
    """Special function: Provide an already-completed service future."""

    def __init__(self, value) -> None:
        """Special function: Store the deterministic service response."""
        self.value = value

    def add_done_callback(self, callback) -> None:
        """Special function: Notify lifecycle client waiters immediately."""
        callback(self)

    def done(self) -> bool:
        """Special function: Report immediate completion."""
        return True

    def result(self):
        """Special function: Return the configured response."""
        return self.value


class PendingFuture:
    """Special function: Model a manager request that outlives the client wait deadline."""

    def add_done_callback(self, _callback) -> None:
        """Special function: Retain pending state without notifying the waiter."""
        return None

    def done(self) -> bool:
        """Special function: Report that the service operation remains in progress."""
        return False

    def result(self):
        """Special function: A pending service has no response value."""
        return None


class FakeServiceClient:
    """Special function: Record high-level lifecycle service calls."""

    def __init__(
        self,
        success: bool = True,
        message: str = "ok",
        state_json: str = "{}",
    ) -> None:
        """Special function: Store response policy and request history."""
        self.success = success
        self.message = message
        self.state_json = state_json
        self.requests = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        """Special function: Report the fake manager service as available."""
        del timeout_sec
        return True

    def call_async(self, request):
        """Special function: Capture the request and return an immediate response."""
        self.requests.append(request)
        return ImmediateFuture(
            SimpleNamespace(
                success=self.success,
                message=self.message,
                state_json=self.state_json,
            )
        )


class PendingServiceClient(FakeServiceClient):
    """Special function: Record a prepare request that never completes within the test."""

    def call_async(self, request):
        """Special function: Capture the request and return a pending future."""
        self.requests.append(request)
        return PendingFuture()


class FakeLogger:
    """Special function: Accept lifecycle-client informational logs."""

    def info(self, _message: str) -> None:
        """Special function: Ignore test log output."""
        return None

    def error(self, _message: str) -> None:
        """Special function: Ignore expected lifecycle prepare diagnostics."""
        return None


class FakeNode:
    """Special function: Supply the logger used by LifecycleSkillResourceClient."""

    def get_logger(self) -> FakeLogger:
        """Special function: Return a no-op logger."""
        return FakeLogger()


def make_client(
    prepare_success: bool = True,
    cleanup_success: bool = True,
) -> LifecycleSkillResourceClient:
    """Special function: Build a lifecycle client without a ROS graph."""
    client = LifecycleSkillResourceClient.__new__(LifecycleSkillResourceClient)
    client.node = FakeNode()
    client.transition_timeout_sec = 1.0
    client.operation_timeout_margin_sec = 0.1
    client.prepare_client = FakeServiceClient(prepare_success, "prepared")
    client.cleanup_client = FakeServiceClient(cleanup_success, "cleaned")
    client.recover_client = FakeServiceClient(True, "recovered")
    return client


def test_prepare_and_cleanup_retain_only_logical_skill_name() -> None:
    """Special function: Executor ownership contains no PID, PGID, or process generation."""
    client = make_client()
    blackboard = Blackboard()
    started, start_message = client.start(blackboard, "COUNT_OBJECTS")
    assert started
    assert start_message == "prepared"
    assert blackboard[bb.ACTIVE_LIFECYCLE_SKILL] == "COUNT_OBJECTS"
    assert client.prepare_client.requests[0].skill_name == "COUNT_OBJECTS"

    stopped, stop_message = client.stop(blackboard)
    assert stopped
    assert stop_message == "cleaned"
    assert blackboard[bb.ACTIVE_LIFECYCLE_SKILL] == ""
    assert client.cleanup_client.requests[0].skill_name == "COUNT_OBJECTS"


def test_cleanup_failure_keeps_ownership_and_fails_closed() -> None:
    """Special function: Failed cleanup remains visible to the executor teardown barrier."""
    client = make_client(cleanup_success=False)
    blackboard = Blackboard()
    assert client.start(blackboard, "COUNT_PEOPLE")[0]
    stopped, message = client.stop(blackboard)
    assert not stopped
    assert message == "cleaned"
    assert blackboard[bb.ACTIVE_LIFECYCLE_SKILL] == "COUNT_PEOPLE"


def test_cancellation_before_prepare_sends_no_manager_request() -> None:
    """Special function: Queue cancellation prevents lifecycle prepare dispatch."""
    client = make_client()
    blackboard = Blackboard()
    blackboard[bb.CANCEL_REQUESTED] = lambda: True
    started, message = client.start(blackboard, "FIND_OBJECT")
    assert not started
    assert "canceled" in message
    assert client.prepare_client.requests == []


def test_uncertain_prepare_queues_explicit_cleanup() -> None:
    """Special function: A client timeout cannot silently abandon a later-active group."""
    client = make_client()
    client.transition_timeout_sec = 0.01
    client.operation_timeout_margin_sec = 0.01
    client.prepare_client = PendingServiceClient()
    blackboard = Blackboard()

    started, message = client.start(blackboard, "FIND_OBJECT")

    assert not started
    assert "cleanup=cleaned" in message
    assert client.cleanup_client.requests[0].skill_name == "FIND_OBJECT"
    assert blackboard[bb.START_CLEANUP_CONFIRMED] is True
    assert blackboard[bb.ACTIVE_LIFECYCLE_SKILL] == ""


def test_matching_integrity_fault_recovers_exactly_once() -> None:
    """Special function: Matching scoped fault gets one bounded recovery and prepare."""
    client = make_client(prepare_success=False)
    fault_payload = json.dumps({
        "conflict_group": "vision_gpu",
        "integrity_faults": {"vision_gpu": "uncertain teardown"},
    })
    client.prepare_client = FakeServiceClient(False, "vision fault", fault_payload)
    client.recover_client = FakeServiceClient(True, "FIND_OBJECT ACTIVE", "{}")
    blackboard = Blackboard()

    started, message = client.start(blackboard, "FIND_OBJECT")

    assert started
    assert message == "FIND_OBJECT ACTIVE"
    assert len(client.prepare_client.requests) == 1
    assert len(client.recover_client.requests) == 1
    assert blackboard[bb.ACTIVE_LIFECYCLE_SKILL] == "FIND_OBJECT"


def test_nonmatching_or_absent_fault_does_not_recover() -> None:
    """Special function: Availability failures never invoke lifecycle recovery."""
    client = make_client(prepare_success=False)
    payload = json.dumps({
        "conflict_group": "vision_gpu",
        "integrity_faults": {"navigation_motion": "other group"},
    })
    client.prepare_client = FakeServiceClient(False, "required service missing", payload)
    blackboard = Blackboard()

    started, message = client.start(blackboard, "FIND_OBJECT")

    assert not started
    assert message == "required service missing"
    assert client.recover_client.requests == []
