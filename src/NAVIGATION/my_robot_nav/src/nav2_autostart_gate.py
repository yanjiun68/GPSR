#!/usr/bin/env python3
# flake8: noqa: E501

from functools import partial

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


# Section: Keep lifecycle decisions independent from ROS callbacks for focused testing.
NAV2_READY = 'ready'
NAV2_STARTABLE = 'startable'
NAV2_UNSAFE = 'unsafe'

# Section: Define the complete Nav2-only ownership boundary in one testable value.
NAV2_MANAGED_NODE_NAMES = (
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
)


def classify_nav2_state(manager_active, node_state_ids):
    """Special function: Classify whether STARTUP is safe from observed state."""
    # Section: The manager's active result is authoritative and makes STARTUP unnecessary.
    if manager_active:
        return NAV2_READY

    # Section: Missing or unknown node state must never permit a blind STARTUP.
    if not node_state_ids or State.PRIMARY_STATE_UNKNOWN in node_state_ids:
        return NAV2_UNSAFE

    # Section: A fully active stack is ready even if the manager response was stale.
    if all(state_id == State.PRIMARY_STATE_ACTIVE for state_id in node_state_ids):
        return NAV2_READY

    # Section: STARTUP is valid only when every managed node is unconfigured.
    if all(
        state_id == State.PRIMARY_STATE_UNCONFIGURED
        for state_id in node_state_ids
    ):
        return NAV2_STARTABLE

    # Section: Inactive, finalized, transitional, and mixed states need operator recovery.
    return NAV2_UNSAFE


class StartupRequestGuard:
    """Special class: Enforce one STARTUP attempt until its result is known."""

    def __init__(self):
        """Special function: Initialize cached readiness and request ownership flags."""
        # Section: Track successful completion and asynchronous request ownership.
        self.nav2_started = False
        self.startup_request_in_flight = False
        self.startup_failed = False

    def begin(self):
        """Special function: Atomically reserve the single allowed STARTUP request."""
        # Section: Suppress requests after success, during a call, or after a rejected call.
        if any((
            self.nav2_started,
            self.startup_request_in_flight,
            self.startup_failed,
        )):
            return False

        self.startup_request_in_flight = True
        return True

    def complete(self, success):
        """Special function: Record an asynchronous STARTUP result idempotently."""
        # Section: Release the in-flight reservation before recording the terminal result.
        self.startup_request_in_flight = False
        self.nav2_started = bool(success)
        self.startup_failed = not success

    def mark_ready(self):
        """Special function: Cache externally observed Nav2 readiness."""
        # Section: An active observation supersedes a prior failed request.
        self.startup_request_in_flight = False
        self.startup_failed = False
        self.nav2_started = True


class Nav2AutostartGate(Node):
    """Special class: Start Nav2 once after prerequisites and lifecycle checks pass."""

    def __init__(self):
        """Special function: Configure prerequisite and lifecycle state observers."""
        # Section: Initialize the ROS node and declare gate configuration.
        super().__init__('nav2_autostart_gate')

        self.declare_parameter(
            'service_name',
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.declare_parameter(
            'is_active_service_name',
            '/lifecycle_manager_navigation/is_active',
        )
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('require_initial_pose', True)
        self.declare_parameter('localization_topic', '/amcl_pose')
        self.declare_parameter('require_localization', False)
        self.declare_parameter('covariance_x_threshold', 0.12)
        self.declare_parameter('covariance_y_threshold', 0.12)
        self.declare_parameter('covariance_yaw_threshold', 0.10)
        self.declare_parameter('require_transform', False)
        self.declare_parameter('transform_target_frame', 'map')
        self.declare_parameter('transform_source_frame', 'base_footprint')
        self.declare_parameter('wait_log_interval_sec', 5.0)
        self.declare_parameter('lifecycle_check_interval_sec', 2.0)
        self.declare_parameter('startup_delay', 0.5)
        self.declare_parameter(
            'managed_node_names',
            list(NAV2_MANAGED_NODE_NAMES),
        )

        # Section: Read prerequisite and lifecycle-manager parameters once.
        self.require_initial_pose = bool(self.get_parameter('require_initial_pose').value)
        self.require_localization = bool(self.get_parameter('require_localization').value)
        self.require_transform = bool(self.get_parameter('require_transform').value)
        self.transform_target_frame = self.get_parameter('transform_target_frame').value
        self.transform_source_frame = self.get_parameter('transform_source_frame').value
        self.wait_log_interval_sec = float(
            self.get_parameter('wait_log_interval_sec').value
        )
        self.lifecycle_check_interval_sec = float(
            self.get_parameter('lifecycle_check_interval_sec').value
        )
        self.covariance_x_threshold = float(
            self.get_parameter('covariance_x_threshold').value
        )
        self.covariance_y_threshold = float(
            self.get_parameter('covariance_y_threshold').value
        )
        self.covariance_yaw_threshold = float(
            self.get_parameter('covariance_yaw_threshold').value
        )
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.managed_node_names = list(
            self.get_parameter('managed_node_names').value
        )

        # Section: Initialize prerequisite, request, and diagnostic state.
        self.have_odom = False
        self.have_initial_pose = False
        self.have_localization = False
        self.request_guard = StartupRequestGuard()
        self.lifecycle_check_in_flight = False
        self.node_state_results = {}
        self.ready_since = None
        self.last_wait_log_time = None
        self.last_lifecycle_check_time = None
        self.lifecycle_check_logged = False
        self.last_diagnostic_key = None

        # Section: Create TF support only when TF is a configured prerequisite.
        self.tf_buffer = Buffer() if self.require_transform else None
        self.tf_listener = (
            TransformListener(self.tf_buffer, self)
            if self.require_transform
            else None
        )

        # Section: Create manager clients and one lifecycle-state client per managed node.
        self.manage_client = self.create_client(
            ManageLifecycleNodes,
            self.get_parameter('service_name').value,
        )
        self.is_active_client = self.create_client(
            Trigger,
            self.get_parameter('is_active_service_name').value,
        )
        self.node_state_clients = {
            node_name: self.create_client(GetState, f'/{node_name}/get_state')
            for node_name in self.managed_node_names
        }

        # Section: Preserve the existing odom, pose, localization, and timer inputs.
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('initial_pose_topic').value,
            self.initial_pose_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('localization_topic').value,
            self.localization_callback,
            10,
        )
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info('Waiting for Nav2 prerequisites.')

    def odom_callback(self, _msg: Odometry):
        """Special function: Record that wheel odometry is available."""
        # Section: Preserve the original one-way odometry prerequisite latch.
        self.have_odom = True

    def initial_pose_callback(self, _msg: PoseWithCovarianceStamped):
        """Special function: Record that an initial pose was supplied."""
        # Section: Preserve the original one-way initial-pose prerequisite latch.
        self.have_initial_pose = True

    def localization_callback(self, msg: PoseWithCovarianceStamped):
        """Special function: Track whether localization covariance is acceptable."""
        # Section: Preserve the original covariance thresholds and indices.
        covariance = msg.pose.covariance
        self.have_localization = all((
            covariance[0] <= self.covariance_x_threshold,
            covariance[7] <= self.covariance_y_threshold,
            covariance[35] <= self.covariance_yaw_threshold,
        ))

    def timer_callback(self):
        """Special function: Advance prerequisites, state inspection, and startup."""
        # Section: Terminal and asynchronous states suppress duplicate timer work.
        if any((
            self.request_guard.nav2_started,
            self.request_guard.startup_request_in_flight,
            self.lifecycle_check_in_flight,
        )):
            return

        # Section: Preserve every existing prerequisite before lifecycle inspection.
        if not self.prerequisites_satisfied():
            self.ready_since = None
            self.log_waiting('Waiting for Nav2 prerequisites.')
            return

        # Section: Preserve the settling delay after prerequisites first become valid.
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.ready_since is None:
            self.ready_since = now_sec
            self.get_logger().info('Nav2 prerequisites satisfied.')
            return

        if (now_sec - self.ready_since) < self.startup_delay:
            return

        # Section: Inspect lifecycle-manager state before every possible STARTUP.
        if (
            self.last_lifecycle_check_time is not None
            and (
                now_sec - self.last_lifecycle_check_time
            ) < self.lifecycle_check_interval_sec
        ):
            return
        self.check_nav2_state()

    def prerequisites_satisfied(self):
        """Special function: Evaluate the unchanged navigation prerequisites."""
        # Section: Require odometry and each optionally configured readiness source.
        if not self.have_odom:
            return False
        if self.require_initial_pose and not self.have_initial_pose:
            return False
        if self.require_localization and not self.have_localization:
            return False
        if self.require_transform and not self.required_transform_available():
            return False
        return True

    def check_nav2_state(self):
        """Special function: Ask the lifecycle manager whether Nav2 is active."""
        # Section: Do not overlap manager-state checks or STARTUP service calls.
        if any((
            self.lifecycle_check_in_flight,
            self.request_guard.startup_request_in_flight,
            self.request_guard.nav2_started,
        )):
            return

        # Section: Unknown manager availability is diagnostic-only and never startable.
        if not self.is_active_client.wait_for_service(timeout_sec=0.0):
            self.log_diagnostic_once(
                'manager_state_unavailable',
                'Nav2 lifecycle manager state service is unavailable; '
                'refusing blind STARTUP.',
            )
            return

        self.lifecycle_check_in_flight = True
        self.last_lifecycle_check_time = (
            self.get_clock().now().nanoseconds / 1e9
        )
        if not self.lifecycle_check_logged:
            self.lifecycle_check_logged = True
            self.get_logger().info('Checking Nav2 lifecycle manager state.')
        try:
            future = self.is_active_client.call_async(Trigger.Request())
        except Exception as exc:
            self.lifecycle_check_in_flight = False
            self.log_diagnostic_once(
                'manager_state_call_failed',
                f'Failed to query Nav2 lifecycle manager state: {exc}',
            )
            return
        future.add_done_callback(self.handle_is_active_response)

    def handle_is_active_response(self, future):
        """Special function: Handle manager activity before inspecting node states."""
        # Section: Query failure leaves lifecycle state unknown and blocks STARTUP.
        try:
            response = future.result()
        except Exception as exc:
            self.lifecycle_check_in_flight = False
            self.log_diagnostic_once(
                'manager_state_response_failed',
                f'Failed to read Nav2 lifecycle manager state: {exc}',
            )
            return

        # Section: ACTIVE is success and must never be followed by STARTUP.
        if response is not None and response.success:
            self.mark_nav2_ready(
                'Nav2 lifecycle manager is already active; startup request skipped.'
            )
            return

        # Section: A non-active manager requires managed-node state inspection.
        self.query_managed_node_states()

    def query_managed_node_states(self):
        """Special function: Query every managed node before deciding on STARTUP."""
        # Section: Empty node ownership is unknown and therefore not startable.
        if not self.node_state_clients:
            self.lifecycle_check_in_flight = False
            self.log_diagnostic_once(
                'no_managed_nodes',
                'No managed Nav2 nodes are configured; refusing blind STARTUP.',
            )
            return

        # Section: Any unavailable get_state service makes the complete state unknown.
        unavailable = [
            node_name
            for node_name, client in self.node_state_clients.items()
            if not client.wait_for_service(timeout_sec=0.0)
        ]
        if unavailable:
            self.lifecycle_check_in_flight = False
            self.log_diagnostic_once(
                f'unavailable:{",".join(unavailable)}',
                'Nav2 lifecycle state is unknown; unavailable get_state services: '
                f'{", ".join(unavailable)}. Refusing blind STARTUP.',
            )
            return

        # Section: Collect asynchronous node responses under the existing state-check lock.
        self.node_state_results = {}
        callbacks_registered = 0
        for node_name, client in self.node_state_clients.items():
            try:
                future = client.call_async(GetState.Request())
            except Exception as exc:
                self.node_state_results[node_name] = State.PRIMARY_STATE_UNKNOWN
                self.log_diagnostic_once(
                    f'node_state_call_failed:{node_name}',
                    f'Failed to query lifecycle state for {node_name}: {exc}',
                )
                continue
            future.add_done_callback(
                partial(self.handle_node_state_response, node_name)
            )
            callbacks_registered += 1

        # Section: Handle the unlikely case where every asynchronous call failed immediately.
        if callbacks_registered == 0:
            self.finish_node_state_check()

    def handle_node_state_response(self, node_name, future):
        """Special function: Collect one managed node's lifecycle state."""
        # Section: Convert failed or absent responses into an explicit unknown state.
        try:
            response = future.result()
            state_id = (
                response.current_state.id
                if response is not None
                else State.PRIMARY_STATE_UNKNOWN
            )
        except Exception as exc:
            state_id = State.PRIMARY_STATE_UNKNOWN
            self.log_diagnostic_once(
                f'node_state_response_failed:{node_name}',
                f'Failed to read lifecycle state for {node_name}: {exc}',
            )

        self.node_state_results[node_name] = state_id
        if len(self.node_state_results) == len(self.node_state_clients):
            self.finish_node_state_check()

    def finish_node_state_check(self):
        """Special function: Classify the complete managed-node state snapshot."""
        # Section: Release the state-check lock before taking the selected action.
        self.lifecycle_check_in_flight = False
        ordered_states = [
            self.node_state_results.get(
                node_name,
                State.PRIMARY_STATE_UNKNOWN,
            )
            for node_name in self.managed_node_names
        ]
        decision = classify_nav2_state(False, ordered_states)

        # Section: Fully active nodes are success and do not require STARTUP.
        if decision == NAV2_READY:
            self.mark_nav2_ready('Nav2 nodes are already active; startup request skipped.')
            return

        # Section: A rejected prior request remains blocked even after re-evaluation.
        if decision == NAV2_STARTABLE and self.request_guard.startup_failed:
            self.log_diagnostic_once(
                'startup_retry_suppressed',
                'Nav2 remains unconfigured after a failed STARTUP; automatic retry '
                'is suppressed. Restart the gate after correcting the lifecycle error.',
            )
            return

        # Section: A uniformly unconfigured stack is the only safe STARTUP source state.
        if decision == NAV2_STARTABLE:
            self.request_startup()
            return

        # Section: Report exact mixed or unknown states without changing them.
        state_text = ', '.join(
            f'{node_name}={state_id}'
            for node_name, state_id in zip(self.managed_node_names, ordered_states)
        )
        self.log_diagnostic_once(
            f'mixed:{state_text}',
            'Mixed or unknown Nav2 lifecycle state detected; refusing blind '
            f'STARTUP. States: {state_text}',
        )

    def request_startup(self):
        """Special function: Issue the single guarded lifecycle STARTUP request."""
        # Section: An unavailable management service cannot safely receive STARTUP.
        if not self.manage_client.wait_for_service(timeout_sec=0.0):
            self.log_diagnostic_once(
                'manage_service_unavailable',
                'Nav2 lifecycle management service is unavailable; STARTUP not sent.',
            )
            return

        # Section: Reserve request ownership before constructing or sending the call.
        if not self.request_guard.begin():
            if self.request_guard.startup_request_in_flight:
                self.log_diagnostic_once(
                    'startup_in_flight',
                    'Nav2 STARTUP request already in progress; skipping duplicate request.',
                )
            return

        request = ManageLifecycleNodes.Request()
        request.command = getattr(ManageLifecycleNodes.Request, 'STARTUP', 0)
        try:
            future = self.manage_client.call_async(request)
        except Exception as exc:
            self.request_guard.complete(False)
            self.log_diagnostic_once(
                'startup_call_failed',
                'Nav2 STARTUP failed; lifecycle state will be re-evaluated '
                f'before retry. Service error: {exc}',
            )
            return
        future.add_done_callback(self.handle_startup_response)
        self.get_logger().info('Requesting Nav2 STARTUP.')

    def required_transform_available(self):
        """Special function: Check the configured map-to-robot transform."""
        # Section: A disabled transform prerequisite is always satisfied.
        if self.tf_buffer is None:
            return True

        available = self.tf_buffer.can_transform(
            self.transform_target_frame,
            self.transform_source_frame,
            Time(),
        )
        if not available:
            self.log_waiting(
                f'Waiting for TF {self.transform_target_frame} -> '
                f'{self.transform_source_frame} before starting Nav2.'
            )
        return available

    def log_waiting(self, message):
        """Special function: Throttle repeated prerequisite messages."""
        # Section: Emit waiting information only at the configured interval.
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if any((
            self.last_wait_log_time is None,
            self.last_wait_log_time is not None
            and (now_sec - self.last_wait_log_time) >= self.wait_log_interval_sec,
        )):
            self.last_wait_log_time = now_sec
            self.get_logger().info(message)

    def log_diagnostic_once(self, key, message):
        """Special function: Log lifecycle diagnostics only when their state changes."""
        # Section: Suppress identical high-frequency lifecycle diagnostics.
        if key == self.last_diagnostic_key:
            return
        self.last_diagnostic_key = key
        self.get_logger().warning(message)

    def mark_nav2_ready(self, message):
        """Special function: Enter terminal READY state and stop polling."""
        # Section: Cache readiness, release state inspection, and stop timer callbacks.
        self.request_guard.mark_ready()
        self.lifecycle_check_in_flight = False
        self.get_logger().info(message)
        self.timer.cancel()

    def handle_startup_response(self, future):
        """Special function: Record STARTUP success or enter safe retry lockout."""
        # Section: Treat exceptions as failed attempts and require later state inspection.
        try:
            response = future.result()
            success = response is not None and response.success
        except Exception as exc:
            self.request_guard.complete(False)
            self.get_logger().error(
                'Nav2 STARTUP failed; lifecycle state will be re-evaluated before '
                f'retry. Service error: {exc}'
            )
            return

        # Section: Successful STARTUP is terminal and cannot repeat on later timer calls.
        self.request_guard.complete(success)
        if success:
            self.get_logger().info('Nav2 STARTUP completed successfully.')
            self.timer.cancel()
            return

        # Section: Rejected STARTUP is never blindly retried from the timer.
        self.get_logger().error(
            'Nav2 STARTUP failed; lifecycle state will be re-evaluated before '
            'retry. Automatic retry remains blocked until the gate is restarted.'
        )


def main(args=None):
    """Special function: Run the Nav2 prerequisite gate node."""
    # Section: Own ROS initialization, spinning, and orderly shutdown.
    rclpy.init(args=args)
    node = Nav2AutostartGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
