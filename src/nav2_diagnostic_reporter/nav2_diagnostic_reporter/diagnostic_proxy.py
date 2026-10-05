"""A standalone, diagnostic-first NavigateToPose action proxy for Nav2 Humble."""

# Section: Standard-library imports for concurrency, timing, and validation.
import math
import threading
import time
from typing import Optional, Tuple

# Section: ROS action and execution imports.
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import Costmap
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

# Section: Import registers PoseStamped support with tf2's Python conversions.
import tf2_geometry_msgs  # noqa: F401

# Section: Local diagnostic domain imports.
from .costmap_utils import (
    DEFAULT_LETHAL_COST,
    CostmapSnapshot,
    point_cost_status,
    snapshot_from_message,
)
from .diagnostic_engine import DiagnosticFinding, success_finding
from .error_codes import ErrorCode
from .message_catalog import user_message_for


# Section: Proxy node implementation.
class Nav2DiagnosticReporter(Node):
    """Preflight goals, proxy Nav2, and publish evidence-based diagnostics."""

    # Section: Node construction and ROS interface wiring.
    def __init__(self) -> None:
        """Create the proxy with reentrant callbacks for action and TF progress."""

        super().__init__('nav2_diagnostic_reporter')
        self._callback_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self._global_frame = self.get_parameter('global_frame').value
        self._robot_base_frame = self.get_parameter('robot_base_frame').value
        self._proxy_action_name = self.get_parameter('proxy_action_name').value
        self._nav_action_name = self.get_parameter('nav_action_name').value
        self._planner_action_name = self.get_parameter('planner_action_name').value
        self._planner_id = self.get_parameter('planner_id').value
        self._lethal_cost = int(self.get_parameter('lethal_cost').value)
        self._unknown_is_blocked = bool(self.get_parameter('unknown_is_blocked').value)
        self._costmap_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._execution_lock = threading.Lock()
        self._latest_epoch = 0
        self._goal_epochs = {}
        self._active_nav_goal = None
        self._active_nav_epoch = None
        self._last_user_message = None
        self._last_user_message_time = 0.0
        self._latest_costmap: Optional[CostmapSnapshot] = None
        self._latest_costmap_stamp = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._nav_client = ActionClient(
            self, NavigateToPose, self._nav_action_name, callback_group=self._callback_group)
        self._planner_client = ActionClient(
            self, ComputePathToPose, self._planner_action_name, callback_group=self._callback_group)
        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._costmap_subscription = self.create_subscription(
            Costmap,
            self.get_parameter('global_costmap_topic').value,
            self._on_costmap,
            costmap_qos,
            callback_group=self._callback_group,
        )
        self._error_publisher = self.create_publisher(String, '/nav2_diagnostics/error_code', 10)
        self._user_publisher = self.create_publisher(String, '/nav2_diagnostics/user_message', 10)
        self._diagnostic_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._server = ActionServer(
            self,
            NavigateToPose,
            self._proxy_action_name,
            execute_callback=self._execute_goal,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            handle_accepted_callback=self._on_accepted,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f'Diagnostic proxy ready: {self._proxy_action_name} -> {self._nav_action_name}')

    # Section: Explicit, overridable deployment parameters.
    def _declare_parameters(self) -> None:
        """Declare all names, frames, timeouts, thresholds, and reporting controls."""

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('proxy_action_name', 'navigate_to_pose_diagnostic')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('planner_action_name', 'compute_path_to_pose')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap_raw')
        self.declare_parameter('tf_timeout_sec', 1.0)
        self.declare_parameter('costmap_timeout_sec', 2.0)
        self.declare_parameter('planner_server_timeout_sec', 2.0)
        self.declare_parameter('planning_timeout_sec', 5.0)
        self.declare_parameter('nav_server_timeout_sec', 2.0)
        self.declare_parameter('navigation_timeout_sec', 0.0)
        self.declare_parameter('cancel_timeout_sec', 2.0)
        self.declare_parameter('lethal_cost', DEFAULT_LETHAL_COST)
        self.declare_parameter('unknown_is_blocked', True)
        self.declare_parameter('user_message_cooldown_sec', 2.0)

    # Section: Transient-local global-costmap cache callback.
    def _on_costmap(self, message: Costmap) -> None:
        """Store the newest complete costmap snapshot for non-blocking preflight checks."""

        try:
            snapshot = snapshot_from_message(message)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning(f'Ignoring malformed costmap: {error}')
            return
        with self._costmap_lock:
            self._latest_costmap = snapshot
            self._latest_costmap_stamp = time.monotonic()

    # Section: Action goal admission always accepts and lets diagnostics explain failures.
    def _on_goal(self, _goal_request: NavigateToPose.Goal) -> GoalResponse:
        """Accept goals so malformed requests receive a structured diagnostic result."""

        return GoalResponse.ACCEPT

    # Section: User cancellation forwarding.
    def _on_cancel(self, _goal_handle) -> CancelResponse:
        """Accept cancellation and promptly request cancellation of the active Nav2 goal."""

        self._cancel_active_navigation()
        return CancelResponse.ACCEPT

    # Section: New-goal preemption policy selected by the user.
    def _on_accepted(self, goal_handle) -> None:
        """Make the newest accepted goal supersede and cancel any current navigation."""

        with self._state_lock:
            self._latest_epoch += 1
            self._goal_epochs[id(goal_handle)] = self._latest_epoch
        self._cancel_active_navigation()
        goal_handle.execute()

    # Section: Active forwarded-goal cancellation helper.
    def _cancel_active_navigation(self) -> None:
        """Request Nav2 cancellation without blocking an action callback thread."""

        with self._state_lock:
            nav_goal = self._active_nav_goal
        if nav_goal is not None:
            try:
                nav_goal.cancel_goal_async()
            except RuntimeError as error:
                self.get_logger().warning(f'Could not request forwarded goal cancellation: {error}')

    # Section: Epoch comparison prevents an old request from starting after preemption.
    def _is_superseded(self, epoch: int) -> bool:
        """Return whether a later proxy goal has been accepted."""

        with self._state_lock:
            return epoch != self._latest_epoch

    # Section: Action-server execution entry point serialized for deterministic preemption.
    def _execute_goal(self, goal_handle) -> NavigateToPose.Result:
        """Run preflight then forward a goal, while safely handling preemption and cancellation."""

        with self._state_lock:
            epoch = self._goal_epochs.get(id(goal_handle), self._latest_epoch)
        try:
            with self._execution_lock:
                if self._is_superseded(epoch):
                    return self._finish(goal_handle, DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'superseded before preflight'), 'canceled')
                finding, transformed_goal, robot_pose = self._preflight(goal_handle.request.pose, epoch)
                if not finding.ok:
                    return self._finish(goal_handle, finding, 'canceled' if self._is_canceling(goal_handle, epoch) else 'abort')
                if self._is_canceling(goal_handle, epoch):
                    return self._finish(goal_handle, DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'canceled before forwarding'), 'canceled')
                return self._forward_navigation(goal_handle, transformed_goal, epoch)
        finally:
            with self._state_lock:
                self._goal_epochs.pop(id(goal_handle), None)

    # Section: Unified cancellation and preemption predicate.
    def _is_canceling(self, goal_handle, epoch: int) -> bool:
        """Return whether the client canceled this goal or a newer goal superseded it."""

        return goal_handle.is_cancel_requested or self._is_superseded(epoch)

    # Section: Pose validation before TF or planner calls.
    def _validate_goal_pose(self, pose: PoseStamped) -> DiagnosticFinding:
        """Validate frame, finite pose values, and a non-degenerate quaternion."""

        point = pose.pose.position
        quaternion = pose.pose.orientation
        values = (point.x, point.y, point.z, quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if not pose.header.frame_id or not all(math.isfinite(value) for value in values):
            return DiagnosticFinding(ErrorCode.INVALID_GOAL, 'missing frame or non-finite pose')
        norm = sum(value * value for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        if norm < 1.0e-12:
            return DiagnosticFinding(ErrorCode.INVALID_GOAL, 'zero-length orientation quaternion')
        return success_finding('goal pose is valid')

    # Section: TF goal conversion and robot-pose lookup.
    def _transform_and_get_robot_pose(
        self, pose: PoseStamped
    ) -> Tuple[DiagnosticFinding, Optional[PoseStamped], Optional[PoseStamped]]:
        """Transform the target into the global frame and obtain global-to-base TF."""

        tf_timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))
        try:
            transformed_goal = pose if pose.header.frame_id == self._global_frame else self._tf_buffer.transform(
                pose, self._global_frame, timeout=tf_timeout)
        except TransformException as error:
            return DiagnosticFinding(ErrorCode.TF_ERROR, str(error)), None, None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame, Time(), timeout=tf_timeout)
        except TransformException as error:
            return DiagnosticFinding(ErrorCode.ROBOT_POSE_UNAVAILABLE, str(error)), None, None
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = self._global_frame
        robot_pose.header.stamp = transform.header.stamp
        robot_pose.pose.position.x = transform.transform.translation.x
        robot_pose.pose.position.y = transform.transform.translation.y
        robot_pose.pose.position.z = transform.transform.translation.z
        robot_pose.pose.orientation = transform.transform.rotation
        return success_finding('goal and robot pose are available'), transformed_goal, robot_pose

    # Section: Cached costmap availability and age check.
    def _get_costmap(self) -> Tuple[DiagnosticFinding, Optional[CostmapSnapshot]]:
        """Return a recent global costmap, or report why it cannot be trusted."""

        with self._costmap_lock:
            snapshot = self._latest_costmap
            stamp = self._latest_costmap_stamp
        timeout = float(self.get_parameter('costmap_timeout_sec').value)
        if snapshot is None or stamp is None or time.monotonic() - stamp > timeout:
            return DiagnosticFinding(ErrorCode.COSTMAP_UNAVAILABLE, 'global costmap absent or stale'), None
        return success_finding('global costmap is available'), snapshot

    # Section: Start and goal map-boundary and blocking checks.
    def _check_costmap_points(
        self, snapshot: CostmapSnapshot, robot_pose: PoseStamped, goal_pose: PoseStamped
    ) -> DiagnosticFinding:
        """Verify start then target points are inside and safe in the global costmap."""

        start_inside, start_blocked, start_cost = point_cost_status(
            snapshot, robot_pose.pose.position.x, robot_pose.pose.position.y,
            self._lethal_cost, self._unknown_is_blocked)
        if not start_inside:
            return DiagnosticFinding(ErrorCode.START_OUTSIDE_MAP, 'robot pose outside global costmap')
        if start_blocked:
            return DiagnosticFinding(ErrorCode.START_OCCUPIED, 'robot cell is blocked', {'cost': str(start_cost)})
        goal_inside, goal_blocked, goal_cost = point_cost_status(
            snapshot, goal_pose.pose.position.x, goal_pose.pose.position.y,
            self._lethal_cost, self._unknown_is_blocked)
        if not goal_inside:
            return DiagnosticFinding(ErrorCode.GOAL_OUTSIDE_MAP, 'goal pose outside global costmap')
        if goal_blocked:
            return DiagnosticFinding(ErrorCode.GOAL_OCCUPIED, 'goal cell is blocked', {'cost': str(goal_cost)})
        return success_finding('start and goal cells are safe')

    # Section: Planner action preflight, limited to evidence available in Humble.
    def _check_planner(
        self, goal_pose: PoseStamped, robot_pose: PoseStamped, epoch: int
    ) -> DiagnosticFinding:
        """Request ComputePathToPose and classify availability, rejection, timeout, and path result."""

        if self._is_superseded(epoch):
            return DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'superseded before planner preflight')
        server_timeout = float(self.get_parameter('planner_server_timeout_sec').value)
        if not self._planner_client.wait_for_server(timeout_sec=server_timeout):
            return DiagnosticFinding(ErrorCode.PLANNER_UNAVAILABLE, 'ComputePathToPose action server unavailable')
        if self._is_superseded(epoch):
            return DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'superseded before planner request')
        request = ComputePathToPose.Goal()
        request.goal = goal_pose
        request.start = robot_pose
        request.use_start = True
        request.planner_id = self._planner_id
        sent = self._planner_client.send_goal_async(request)
        planner_goal = self._wait_for_future(sent, float(self.get_parameter('planning_timeout_sec').value), epoch)
        if planner_goal is None:
            return self._interrupted_or_timeout(epoch, ErrorCode.PLANNING_TIMEOUT, 'planner goal send timed out')
        if not planner_goal.accepted:
            return DiagnosticFinding(ErrorCode.PLANNER_GOAL_REJECTED, 'planner rejected diagnostic request')
        result_future = planner_goal.get_result_async()
        result_wrapper = self._wait_for_future(
            result_future, float(self.get_parameter('planning_timeout_sec').value), epoch)
        if result_wrapper is None:
            planner_goal.cancel_goal_async()
            return self._interrupted_or_timeout(epoch, ErrorCode.PLANNING_TIMEOUT, 'planner result timed out')
        if result_wrapper.status != GoalStatus.STATUS_SUCCEEDED:
            return DiagnosticFinding(ErrorCode.PLANNING_FAILED, 'planner returned non-success status', {'status': str(result_wrapper.status)})
        if not result_wrapper.result.path.poses:
            return DiagnosticFinding(ErrorCode.NO_VALID_PATH, 'planner returned an empty path')
        return success_finding('planner returned a non-empty path')

    # Section: Ordered diagnostic preflight pipeline.
    def _preflight(
        self, requested_goal: PoseStamped, epoch: int
    ) -> Tuple[DiagnosticFinding, Optional[PoseStamped], Optional[PoseStamped]]:
        """Run validation, TF, costmap, and planner checks in the documented order."""

        finding = self._validate_goal_pose(requested_goal)
        if not finding.ok:
            return finding, None, None
        if self._is_superseded(epoch):
            return DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'superseded after validation'), None, None
        finding, transformed_goal, robot_pose = self._transform_and_get_robot_pose(requested_goal)
        if not finding.ok:
            return finding, None, None
        if self._is_superseded(epoch):
            return DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'superseded after TF lookup'), None, None
        finding, costmap = self._get_costmap()
        if not finding.ok:
            return finding, None, None
        finding = self._check_costmap_points(costmap, robot_pose, transformed_goal)
        if not finding.ok:
            return finding, None, None
        finding = self._check_planner(transformed_goal, robot_pose, epoch)
        if not finding.ok:
            return finding, None, None
        return success_finding(), transformed_goal, robot_pose

    # Section: Asynchronous action future wait that leaves other executor threads free.
    def _wait_for_future(self, future, timeout_sec: float, epoch: Optional[int] = None):
        """Poll a ROS future until it completes, times out, or a newer goal supersedes it."""

        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and not future.done():
            if epoch is not None and self._is_superseded(epoch):
                return None
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.02)
        if future.done():
            try:
                return future.result()
            except Exception as error:  # ROS futures surface transport errors here.
                self.get_logger().warning(f'Action future failed: {error}')
        return None

    # Section: Distinguish a user-selected preemption from a real operation timeout.
    def _interrupted_or_timeout(self, epoch: int, timeout_code: ErrorCode, detail: str) -> DiagnosticFinding:
        """Return cancellation when superseded; otherwise retain the actual timeout evidence."""

        if self._is_superseded(epoch):
            return DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'superseded during diagnostic preflight')
        return DiagnosticFinding(timeout_code, detail)

    # Section: Forwarding stage after all required preflight gates have passed.
    def _forward_navigation(self, goal_handle, goal_pose: PoseStamped, epoch: int) -> NavigateToPose.Result:
        """Forward the transformed goal to Nav2 and relay feedback and terminal state."""

        nav_timeout = float(self.get_parameter('nav_server_timeout_sec').value)
        if not self._nav_client.wait_for_server(timeout_sec=nav_timeout):
            return self._finish(goal_handle, DiagnosticFinding(ErrorCode.NAV_SERVER_UNAVAILABLE, 'NavigateToPose action server unavailable'), 'abort')
        if self._is_canceling(goal_handle, epoch):
            return self._finish(goal_handle, DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'canceled before Nav2 request'), 'canceled')
        request = NavigateToPose.Goal()
        request.pose = goal_pose
        request.behavior_tree = goal_handle.request.behavior_tree
        sent = self._nav_client.send_goal_async(request, feedback_callback=lambda feedback: self._relay_feedback(goal_handle, feedback))
        sent.add_done_callback(lambda future: self._cancel_late_nav_goal(future, epoch))
        nav_goal = self._wait_for_future(sent, nav_timeout, epoch)
        if nav_goal is None:
            return self._finish(goal_handle, self._interrupted_or_timeout(epoch, ErrorCode.NAV_SERVER_UNAVAILABLE, 'Nav2 goal send timed out'), 'canceled' if self._is_canceling(goal_handle, epoch) else 'abort')
        if not nav_goal.accepted:
            return self._finish(goal_handle, DiagnosticFinding(ErrorCode.GOAL_REJECTED, 'Nav2 rejected forwarded goal'), 'abort')
        with self._state_lock:
            self._active_nav_goal = nav_goal
            self._active_nav_epoch = epoch
        if self._is_canceling(goal_handle, epoch):
            nav_goal.cancel_goal_async()
            self._clear_active_navigation(epoch)
            return self._finish(goal_handle, DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'canceled immediately after forwarding'), 'canceled')
        result_future = nav_goal.get_result_async()
        execution_timeout = float(self.get_parameter('navigation_timeout_sec').value)
        deadline = None if execution_timeout <= 0.0 else time.monotonic() + execution_timeout
        while rclpy.ok() and not result_future.done():
            if self._is_canceling(goal_handle, epoch):
                nav_goal.cancel_goal_async()
                self._wait_for_future(result_future, float(self.get_parameter('cancel_timeout_sec').value))
                self._clear_active_navigation(epoch)
                return self._finish(goal_handle, DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'forwarded navigation canceled'), 'canceled')
            if deadline is not None and time.monotonic() >= deadline:
                nav_goal.cancel_goal_async()
                self._clear_active_navigation(epoch)
                return self._finish(goal_handle, DiagnosticFinding(ErrorCode.NAVIGATION_TIMEOUT, 'configured execution timeout reached'), 'abort')
            time.sleep(0.05)
        self._clear_active_navigation(epoch)
        if not result_future.done():
            return self._finish(goal_handle, DiagnosticFinding(ErrorCode.UNKNOWN_NAVIGATION_FAILURE, 'ROS shutdown before Nav2 result'), 'abort')
        try:
            result_wrapper = result_future.result()
        except Exception as error:
            return self._finish(goal_handle, DiagnosticFinding(ErrorCode.UNKNOWN_NAVIGATION_FAILURE, str(error)), 'abort')
        if result_wrapper.status == GoalStatus.STATUS_SUCCEEDED:
            return self._finish(goal_handle, success_finding('Nav2 reached goal'), 'succeed')
        if result_wrapper.status == GoalStatus.STATUS_CANCELED:
            return self._finish(goal_handle, DiagnosticFinding(ErrorCode.NAVIGATION_CANCELED, 'Nav2 returned canceled'), 'canceled')
        if result_wrapper.status == GoalStatus.STATUS_ABORTED:
            # Humble provides no rich Nav2 abort code, so re-run checks before asserting a cause.
            finding, _, _ = self._preflight(goal_pose, epoch)
            if finding.ok:
                finding = DiagnosticFinding(ErrorCode.EXECUTION_FAILED, 'Nav2 aborted after a passing planner preflight')
            return self._finish(goal_handle, finding, 'abort')
        return self._finish(goal_handle, DiagnosticFinding(ErrorCode.UNKNOWN_NAVIGATION_FAILURE, 'unexpected Nav2 terminal status', {'status': str(result_wrapper.status)}), 'abort')

    # Section: Relaying compatible NavigateToPose feedback through the proxy action.
    def _relay_feedback(self, proxy_goal_handle, feedback_message) -> None:
        """Forward Nav2 feedback unchanged because both actions share the same type."""

        if not proxy_goal_handle.is_cancel_requested:
            proxy_goal_handle.publish_feedback(feedback_message.feedback)

    # Section: Late Nav2 acceptance must still be canceled after a preemption race.
    def _cancel_late_nav_goal(self, future, epoch: int) -> None:
        """Cancel a Nav2 goal accepted after the corresponding proxy goal was superseded."""

        if not self._is_superseded(epoch):
            return
        try:
            nav_goal = future.result()
            if nav_goal is not None and nav_goal.accepted:
                nav_goal.cancel_goal_async()
        except Exception as error:  # A failed send needs no cancellation.
            self.get_logger().debug(f'Late Nav2 send did not require cancellation: {error}')

    # Section: Active-goal state cleanup guarded by the epoch identity.
    def _clear_active_navigation(self, epoch: int) -> None:
        """Clear the forwarded-goal reference only if it belongs to this execution."""

        with self._state_lock:
            if self._active_nav_epoch == epoch:
                self._active_nav_goal = None
                self._active_nav_epoch = None

    # Section: Terminal proxy result plus the three required diagnostic outputs.
    def _finish(self, goal_handle, finding: DiagnosticFinding, terminal_state: str) -> NavigateToPose.Result:
        """Publish the diagnostic finding and place the proxy goal in its terminal state."""

        self._publish_diagnostic(finding)
        if terminal_state == 'succeed':
            goal_handle.succeed()
        elif terminal_state == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return NavigateToPose.Result()

    # Section: Structured diagnostic publication with optional user-message cooldown.
    def _publish_diagnostic(self, finding: DiagnosticFinding) -> None:
        """Publish stable code, safe user text, and technical DiagnosticArray evidence."""

        code_text = finding.code.value
        self._error_publisher.publish(String(data=code_text))
        user_message = user_message_for(finding.code)
        now = time.monotonic()
        cooldown = float(self.get_parameter('user_message_cooldown_sec').value)
        if user_message != self._last_user_message or now - self._last_user_message_time >= cooldown:
            self._user_publisher.publish(String(data=user_message))
            self._last_user_message = user_message
            self._last_user_message_time = now
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if finding.ok else DiagnosticStatus.ERROR
        status.name = 'nav2_diagnostic_reporter/preflight'
        status.message = code_text
        status.hardware_id = 'nav2'
        status.values = [
            KeyValue(key='error_code', value=code_text),
            KeyValue(key='detail', value=finding.detail),
            KeyValue(key='global_frame', value=self._global_frame),
            KeyValue(key='robot_base_frame', value=self._robot_base_frame),
        ]
        for key, value in (finding.values or {}).items():
            status.values.append(KeyValue(key=str(key), value=str(value)))
        report = DiagnosticArray()
        report.header.stamp = self.get_clock().now().to_msg()
        report.status = [status]
        self._diagnostic_publisher.publish(report)


# Section: Multi-threaded process entry point required for reentrant action progress.
def main(args=None) -> None:
    """Run the reporter in a multi-threaded executor so proxy and Nav2 actions cannot deadlock."""

    rclpy.init(args=args)
    node = Nav2DiagnosticReporter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
