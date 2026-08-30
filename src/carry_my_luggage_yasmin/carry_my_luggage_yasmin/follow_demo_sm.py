"""Streamlined Carry My Luggage follow-demo workflow implemented with YASMIN."""

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from yasmin import Blackboard, State, StateMachine

from carry_my_luggage_yasmin import outcomes as out
from carry_my_luggage_yasmin.runtime import run_state_machine
from carry_my_luggage_yasmin.states import (
    BestEffortState,
    DelayState,
    LoadWaypointsState,
    RetryState,
    SequenceState,
    TerminalState,
    make_kill_state,
    make_managed_process_state,
    make_person_follow_state,
    make_trigger_state,
)


class PublishUnmappedModeState(State):
    """Select person-follow's unmapped operating mode."""

    def __init__(self, node: Node) -> None:
        """Create the mode publisher used immediately before following."""
        super().__init__({out.SUCCEEDED})
        self.publisher = node.create_publisher(String, "/person_follow/mode", 10)

    def execute(self, _blackboard: Blackboard) -> str:
        """Publish the same unmapped mode as the SMACC2 demo behavior."""
        message = String()
        message.data = "unmapped"
        self.publisher.publish(message)
        return out.SUCCEEDED


def build_follow_demo_state_machine(node: Node) -> StateMachine:
    """Build the currently active follow-demo transition graph."""
    # Section: Resolve the follow-demo waypoint copy installed with this YASMIN package.
    share = get_package_share_directory("carry_my_luggage_yasmin")
    waypoint_file = f"{share}/config/follow_demo_waypoints.yaml"

    # Section: Keep startup validation and waypoint preload from the source implementation.
    startup = SequenceState(
        [
            make_trigger_state(node, "/grasp/is_ready", response_timeout=10.0),
            LoadWaypointsState(waypoint_file),
        ]
    )
    destination = SequenceState(
        [
            make_managed_process_state(node, "/bringup/person_follow", 25.0),
            PublishUnmappedModeState(node),
            make_person_follow_state(node, face_hold_sec=10.0),
        ]
    )

    def make_cleanup() -> SequenceState:
        """Build an independent best-effort cleanup sequence."""
        return SequenceState(
            [
                BestEffortState(make_trigger_state(node, "/person_follow/stop", 1.0, 2.0)),
                BestEffortState(make_kill_state(node, "person_follow")),
                BestEffortState(make_kill_state(node, "bag_point_selection")),
            ]
        )

    cleanup = make_cleanup()
    abort_cleanup = SequenceState(
        [BestEffortState(make_trigger_state(node, "/grasp/rest")), make_cleanup()]
    )

    # Section: This graph follows the compiled headers, not the stale workflow markdown.
    machine = StateMachine([out.COMPLETED, out.ABORTED], handle_sigint=True)
    machine.add_state(
        "ST_STARTUP_BRINGUP",
        RetryState(startup, "demo_startup_attempts"),
        {
            out.SUCCEEDED: "ST_EXECUTE_GRASP",
            out.RETRY: "ST_STARTUP_BRINGUP",
            out.EXHAUSTED: "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_EXECUTE_GRASP",
        RetryState(make_trigger_state(node, "/grasp/execute"), "demo_grasp_attempts"),
        {
            out.SUCCEEDED: "ST_GOTO_DESTINATION",
            out.RETRY: "ST_EXECUTE_GRASP",
            out.EXHAUSTED: "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_GOTO_DESTINATION",
        RetryState(destination, "demo_destination_attempts"),
        {
            out.SUCCEEDED: "ST_WAIT_BEFORE_DROP",
            out.RETRY: "ST_GOTO_DESTINATION",
            out.EXHAUSTED: "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_WAIT_BEFORE_DROP",
        DelayState(5.0),
        {out.SUCCEEDED: "ST_ARM_REST", out.CANCELED: "ST_ABORTED"},
    )
    machine.add_state(
        "ST_ARM_REST",
        RetryState(make_trigger_state(node, "/grasp/rest"), "demo_arm_rest_attempts"),
        {
            out.SUCCEEDED: "ST_RETURN_HOME",
            out.RETRY: "ST_ARM_REST",
            out.EXHAUSTED: "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_RETURN_HOME",
        RetryState(
            make_trigger_state(node, "/return_home/start", response_timeout=170.0),
            "demo_return_attempts",
        ),
        {
            out.SUCCEEDED: "ST_ABORT_ALL_PROCESSES",
            out.RETRY: "ST_RETURN_HOME",
            out.EXHAUSTED: "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_ABORT_ALL_PROCESSES",
        cleanup,
        {
            out.SUCCEEDED: "ST_COMPLETED",
            out.ABORTED: "ST_ABORT_ALL_PROCESSES",
            "timeout": "ST_ABORT_ALL_PROCESSES",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_COMPLETED", TerminalState(out.COMPLETED), {out.COMPLETED: out.COMPLETED}
    )
    machine.add_state(
        "ST_ABORTED",
        TerminalState(out.ABORTED, abort_cleanup),
        {out.ABORTED: out.ABORTED},
    )
    machine.set_start_state("ST_STARTUP_BRINGUP")
    machine.validate()
    return machine


def main() -> None:
    """Run the streamlined follow-demo YASMIN ROS executable."""
    run_state_machine(
        "carry_my_luggage_follow_demo_yasmin",
        "CARRY_MY_LUGGAGE_FOLLOW_DEMO_YASMIN",
        build_follow_demo_state_machine,
    )


if __name__ == "__main__":
    main()
