"""Section: Central GPSR lifecycle-manager client tests."""

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


class FakeServiceClient:
    """Special function: Record high-level lifecycle service calls."""

    def __init__(self, success: bool = True, message: str = "ok") -> None:
        """Special function: Store response policy and request history."""
        self.success = success
        self.message = message
        self.requests = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        """Special function: Report the fake manager service as available."""
        del timeout_sec
        return True

    def call_async(self, request):
        """Special function: Capture the request and return an immediate response."""
        self.requests.append(request)
        return ImmediateFuture(
            SimpleNamespace(success=self.success, message=self.message, state_json="{}")
        )


class FakeLogger:
    """Special function: Accept lifecycle-client informational logs."""

    def info(self, _message: str) -> None:
        """Special function: Ignore test log output."""
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
    client.prepare_client = FakeServiceClient(prepare_success, "prepared")
    client.cleanup_client = FakeServiceClient(cleanup_success, "cleaned")
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
