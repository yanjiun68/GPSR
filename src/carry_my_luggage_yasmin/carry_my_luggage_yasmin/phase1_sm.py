"""Full phase-one Carry My Luggage workflow implemented with YASMIN."""

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from yasmin import StateMachine

from carry_my_luggage_yasmin import outcomes as out
from carry_my_luggage_yasmin.runtime import run_state_machine
from carry_my_luggage_yasmin.states import (
    BestEffortState,
    DelayState,
    LoadWaypointsState,
    PassBagDecisionState,
    RetryState,
    SelectBagState,
    SequenceState,
    TargetStillState,
    TerminalState,
    WaitForStartState,
    make_kill_state,
    make_managed_process_state,
    make_receptionist_state,
    make_trigger_state,
)


def build_phase1_state_machine(node: Node) -> StateMachine:
    """Build the active full phase-one transition graph."""
    # Section: Resolve installed data rather than retaining source-tree absolute paths.
    share = get_package_share_directory("carry_my_luggage_yasmin")
    waypoint_file = f"{share}/config/phase1_waypoints.yaml"

    # Section: Startup validates the grasp stack and preloads the original waypoint data.
    startup = SequenceState(
        [
            make_trigger_state(node, "/grasp/is_ready", response_timeout=10.0),
            LoadWaypointsState(waypoint_file),
        ]
    )

    # Section: Build reusable manipulation, follow, voice, return, and cleanup behaviors.
    grasp = make_trigger_state(node, "/grasp/execute")
    arm_rest = make_trigger_state(node, "/grasp/rest")
    return_home = make_trigger_state(
        node, "/breadcrumb_teach_repeat/finish_teach", response_timeout=170.0
    )
    follow_operator = SequenceState(
        [
            make_managed_process_state(node, "/bringup/person_follow", 25.0),
            TargetStillState(node),
            BestEffortState(make_kill_state(node, "person_follow")),
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

    # Section: The graph preserves the active SMACC2 success, retry, and abort edges.
    machine = StateMachine([out.COMPLETED, out.ABORTED], handle_sigint=True)
    machine.add_state(
        "ST_STARTUP_BRINGUP",
        RetryState(startup, "phase1_startup_attempts"),
        {
            out.SUCCEEDED: "ST_WAIT_FOR_BUTTON_PRESS",
            out.RETRY: "ST_STARTUP_BRINGUP",
            out.EXHAUSTED: "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_WAIT_FOR_BUTTON_PRESS",
        WaitForStartState(node),
        {
            out.SUCCEEDED: "ST_BRINGUP_BAG",
            out.FAILED: "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_BRINGUP_BAG",
        RetryState(
            make_managed_process_state(node, "/bringup/bag_point_selection"),
            "phase1_bag_bringup_attempts",
        ),
        {
            out.SUCCEEDED: "ST_SELECT_BAG",
            out.RETRY: "ST_BRINGUP_BAG",
            out.EXHAUSTED: "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_SELECT_BAG",
        SelectBagState(node),
        {
            out.SUCCEEDED: "ST_SHUTDOWN_BAG",
            out.POSE_FAILED: "ST_SELECT_BAG",
            out.BAG_FAILED: "ST_BRINGUP_BAG",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_SHUTDOWN_BAG",
        RetryState(make_kill_state(node, "bag_point_selection"), "phase1_bag_shutdown_attempts"),
        {
            out.SUCCEEDED: "ST_EXECUTE_GRASP",
            out.RETRY: "ST_SHUTDOWN_BAG",
            out.EXHAUSTED: "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_EXECUTE_GRASP",
        RetryState(grasp, "phase1_grasp_attempts"),
        {
            out.SUCCEEDED: "ST_KILL_RX200_CAMERA",
            out.RETRY: "ST_EXECUTE_GRASP",
            # Section: Match the source graph's degraded continuation after grasp retry exhaustion.
            out.EXHAUSTED: "ST_KILL_RX200_CAMERA",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_KILL_RX200_CAMERA",
        BestEffortState(make_kill_state(node, "/camera/rx200_camera")),
        {out.SUCCEEDED: "ST_FOLLOW_OPERATOR"},
    )
    machine.add_state(
        "ST_FOLLOW_OPERATOR",
        follow_operator,
        {
            out.SUCCEEDED: "ST_ASK_PASS_BAG_CONFIRMATION",
            out.ABORTED: "ST_ABORTED",
            "timeout": "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_ASK_PASS_BAG_CONFIRMATION",
        PassBagDecisionState(make_receptionist_state(node)),
        {
            out.YES: "ST_WAIT_BEFORE_DROP",
            out.NO: "ST_FOLLOW_OPERATOR",
            out.FAILED: "ST_ABORTED",
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
        RetryState(arm_rest, "phase1_arm_rest_attempts"),
        {
            out.SUCCEEDED: "ST_RETURN_HOME",
            out.RETRY: "ST_ARM_REST",
            out.EXHAUSTED: "ST_ABORTED",
            out.CANCELED: "ST_ABORTED",
        },
    )
    machine.add_state(
        "ST_RETURN_HOME",
        return_home,
        {
            out.SUCCEEDED: "ST_ABORT_ALL_PROCESSES",
            out.ABORTED: "ST_ABORTED",
            "timeout": "ST_ABORTED",
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
    """Run the full phase-one YASMIN ROS executable."""
    run_state_machine(
        "carry_my_luggage_phase1_yasmin",
        "CARRY_MY_LUGGAGE_PHASE1_YASMIN",
        build_phase1_state_machine,
    )


if __name__ == "__main__":
    main()
