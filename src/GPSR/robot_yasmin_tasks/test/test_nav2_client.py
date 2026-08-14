"""Section: Real NavigateNamedPose client behavior tests."""

from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.gpsr.outcomes import FAILED, SUCCESS
from robot_yasmin_tasks.ros_clients.nav2_client import Nav2Client


class ImmediateFuture:
    """Special function: Provide a completed ROS-future-shaped test double."""

    def __init__(self, result) -> None:
        """Special function: Store the deterministic future result."""
        self._result = result

    def add_done_callback(self, callback) -> None:
        """Special function: Invoke completion callbacks immediately."""
        callback(self)

    def done(self) -> bool:
        """Special function: Report that the test future has completed."""
        return True

    def result(self):
        """Special function: Return the stored deterministic value."""
        return self._result


class AcceptedGoal:
    """Special function: Provide an accepted named-navigation goal handle."""

    def __init__(self, success: bool, message: str = "") -> None:
        """Special function: Store the semantic navigation result."""
        self.accepted = True
        self._wrapped = SimpleNamespace(
            status=(
                GoalStatus.STATUS_SUCCEEDED
                if success
                else GoalStatus.STATUS_ABORTED
            ),
            result=SimpleNamespace(success=success, message=message),
        )
        self.cancel_calls = 0

    def get_result_async(self) -> ImmediateFuture:
        """Special function: Return an immediately completed action result."""
        return ImmediateFuture(self._wrapped)

    def cancel_goal_async(self) -> ImmediateFuture:
        """Special function: Record cancellation requests for timeout paths."""
        self.cancel_calls += 1
        return ImmediateFuture(SimpleNamespace())


class FakeActionClient:
    """Special function: Capture named-pose goals without a live ROS graph."""

    def __init__(self, goal_handle: AcceptedGoal, available: bool = True) -> None:
        """Special function: Configure server availability and the returned goal handle."""
        self.goal_handle = goal_handle
        self.available = available
        self.target_names = []

    def wait_for_server(self, timeout_sec: float) -> bool:
        """Special function: Return configured action-server availability."""
        return self.available

    def send_goal_async(self, goal) -> ImmediateFuture:
        """Special function: Capture the target name and accept the goal."""
        self.target_names.append(goal.target_name)
        return ImmediateFuture(self.goal_handle)


def real_client_with(fake_client: FakeActionClient) -> Nav2Client:
    """Special function: Construct a real-mode client around a test action transport."""
    client = Nav2Client(node=None, simulate=False)
    client.client = fake_client
    return client


def test_named_navigation_requires_confirmed_success() -> None:
    """Special function: A successful action result commits real navigation success."""
    fake = FakeActionClient(AcceptedGoal(True, "Reached instruction point."))
    client = real_client_with(fake)
    blackboard = Blackboard()
    action = ActionStepData(
        "RETURN_TO_INSTRUCTION_POINT",
        target_location="instruction_point",
        timeout_sec=10.0,
    )
    result = client.navigate_to_pose(blackboard, action)
    assert result.outcome == SUCCESS
    assert result.result["simulated"] is False
    assert fake.target_names == ["instruction_point"]
    assert blackboard[bb.TARGET_POSE] == "instruction_point"


def test_named_navigation_propagates_action_failure() -> None:
    """Special function: An aborted action cannot falsely report robot arrival."""
    fake = FakeActionClient(AcceptedGoal(False, "Planner failed."))
    client = real_client_with(fake)
    result = client.navigate_to_pose(
        Blackboard(),
        ActionStepData("NAVIGATE", target_location="kitchen", timeout_sec=10.0),
    )
    assert result.outcome == FAILED
    assert result.recoverable is True
    assert result.message == "Planner failed."


def test_named_navigation_reports_unavailable_server() -> None:
    """Special function: Missing navigation infrastructure is a recoverable failure."""
    fake = FakeActionClient(AcceptedGoal(True), available=False)
    client = real_client_with(fake)
    result = client.navigate_to_pose(
        Blackboard(),
        ActionStepData("NAVIGATE", target_location="kitchen", timeout_sec=10.0),
    )
    assert result.outcome == FAILED
    assert result.recoverable is True
    assert "unavailable" in result.message
