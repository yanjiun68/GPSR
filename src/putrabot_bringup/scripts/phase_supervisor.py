#!/usr/bin/env python3

"""Section: Supervisor-owned launch processes with bounded lifecycle control."""

import errno
import os
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Callable, Optional

from ament_index_python.packages import PackageNotFoundError, get_package_prefix

import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
    get_action_server_names_and_types_by_node,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from putrabot_interfaces.action import ManagedProcess
from putrabot_interfaces.srv import KillManagedProcess


@dataclass(frozen=True)
class ManagedSpec:
    """Special function: Declare one supervisor-owned logical workload."""

    name: str
    action_name: str
    command_factory: Callable[[], list[str]]
    startup_sec: float = 0.0
    wait_for_exit: bool = False
    availability_check: Optional[Callable[[], tuple[bool, str]]] = None
    trigger_callback: Optional[Callable[[object], ManagedProcess.Result]] = None
    startup_timeout_sec: float = 15.0
    required_actions: tuple[str, ...] = ()
    required_services: tuple[str, ...] = ()
    required_topics: tuple[str, ...] = ()
    owned_actions: tuple[str, ...] = ()
    owned_services: tuple[str, ...] = ()
    resources: frozenset[str] = field(default_factory=frozenset)


@dataclass
class ManagedHandle:
    """Special function: Retain immutable process identity from creation time."""

    spec: ManagedSpec
    generation: str
    process: subprocess.Popen
    root_pid: int
    pgid: int
    pidfd: Optional[int]
    state: str
    start_time: float
    resources: frozenset[str]


class PhaseSupervisor(Node):
    """Special function: Own managed launch processes and their exclusive resources."""

    @staticmethod
    def _installed_package_executable(package_name: str, executable: str) -> tuple[bool, str]:
        """Special function: Validate an installed executable before launching it."""
        try:
            prefix = get_package_prefix(package_name)
        except PackageNotFoundError:
            return False, f'package "{package_name}" is not installed'
        executable_path = os.path.join(prefix, 'lib', package_name, executable)
        if not os.path.exists(executable_path):
            return False, f'executable "{executable}" not found in {os.path.dirname(executable_path)}'
        return True, executable_path

    def __init__(self):
        """Special function: Register managed targets and lifecycle-control interfaces."""
        super().__init__('phase_supervisor')

        # Section: Concurrent ROS callbacks share guarded ownership records.
        self.cb_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._start_lock = threading.Lock()
        self._handles: dict[str, ManagedHandle] = {}
        self._resource_owners: dict[str, tuple[str, str]] = {}
        self._action_servers = []
        self._services = []

        # Section: Existing bringup parameters remain unchanged.
        self.declare_parameter('rx200_use_saved_tf', False)
        self.declare_parameter('rx200_auto_grasp', False)
        self.declare_parameter('rx200_show_yolo_viewer', False)
        self.declare_parameter('rx200_use_candidate_notifier', False)
        self.declare_parameter('rx200_candidate_notifier_interval_ms', 200)
        self.declare_parameter(
            'rx200_xs_port',
            '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2XP6Z8-if00-port0',
        )
        self.declare_parameter('rx200_lift_port', 'auto')

        # Section: Managed-process timeout and COUNT_OBJECTS dependency parameters.
        self.declare_parameter('managed_sigint_timeout_sec', 5.0)
        self.declare_parameter('managed_sigterm_timeout_sec', 3.0)
        self.declare_parameter('managed_sigkill_timeout_sec', 2.0)
        self.declare_parameter('managed_graph_timeout_sec', 5.0)
        self.declare_parameter('gpsr_count_objects_startup_timeout_sec', 15.0)
        self.declare_parameter('gpsr_count_objects_rgb_topic', '/camera/image_raw')
        self.declare_parameter('gpsr_count_objects_show_gui', True)

        # Section: Read parameters once so each managed specification is deterministic.
        rx200_use_saved_tf = bool(self.get_parameter('rx200_use_saved_tf').value)
        rx200_auto_grasp = bool(self.get_parameter('rx200_auto_grasp').value)
        rx200_show_yolo_viewer = bool(self.get_parameter('rx200_show_yolo_viewer').value)
        rx200_use_candidate_notifier = bool(self.get_parameter('rx200_use_candidate_notifier').value)
        rx200_candidate_notifier_interval_ms = int(
            self.get_parameter('rx200_candidate_notifier_interval_ms').value
        )
        rx200_xs_port = str(self.get_parameter('rx200_xs_port').value)
        rx200_lift_port = str(self.get_parameter('rx200_lift_port').value)
        count_objects_startup_timeout_sec = float(
            self.get_parameter('gpsr_count_objects_startup_timeout_sec').value
        )
        count_objects_rgb_topic = str(
            self.get_parameter('gpsr_count_objects_rgb_topic').value
        )
        count_objects_show_gui = bool(
            self.get_parameter('gpsr_count_objects_show_gui').value
        )

        # Section: Registry maps logical targets to fixed, non-shell commands.
        self._specs = {
            'person_follow': ManagedSpec(
                name='person_follow',
                action_name='/bringup/person_follow',
                command_factory=lambda: [
                    'ros2',
                    'launch',
                    'person_follower_master',
                    'person_follow.launch.py',
                    'use_external_drivers:=true',
                ],
                startup_sec=2.0,
            ),
            'rx200_grasp': ManagedSpec(
                name='rx200_grasp',
                action_name='/bringup/rx200_grasp',
                command_factory=lambda: [
                    'ros2',
                    'launch',
                    'rx200_handbag_grasp',
                    'all_in_one.launch.py',
                    'use_camera:=false',
                    f'use_saved_tf:={str(rx200_use_saved_tf).lower()}',
                    f'auto_grasp:={str(rx200_auto_grasp).lower()}',
                    f'show_yolo_viewer:={str(rx200_show_yolo_viewer).lower()}',
                    f'use_candidate_notifier:={str(rx200_use_candidate_notifier).lower()}',
                    f'candidate_notifier_interval_ms:={rx200_candidate_notifier_interval_ms}',
                    f'xs_port:={rx200_xs_port}',
                    f'lift_port:={rx200_lift_port}',
                ],
                startup_sec=3.0,
            ),
            'bag_point_selection': ManagedSpec(
                name='bag_point_selection',
                action_name='/bringup/bag_point_selection',
                command_factory=lambda: [
                    'ros2',
                    'launch',
                    'bag_point_selection',
                    'bag_side_motion.launch.py',
                ],
                startup_sec=2.0,
            ),
            'gpsr_count_objects': ManagedSpec(
                name='gpsr_count_objects',
                action_name='/bringup/gpsr_count_objects',
                command_factory=lambda: [
                    'ros2',
                    'launch',
                    'vision',
                    'gpsr_count_objects_skill.launch.py',
                    f'rgb_topic:={count_objects_rgb_topic}',
                    f'show_gui:={str(count_objects_show_gui).lower()}',
                ],
                availability_check=lambda: self._installed_package_executable(
                    'vision', 'gpsr_object_detection_node.py'
                ),
                startup_timeout_sec=count_objects_startup_timeout_sec,
                required_actions=('/gpsr/count_objects',),
                required_services=('/perception/count_objects',),
                required_topics=(count_objects_rgb_topic,),
                owned_actions=('/gpsr/count_objects',),
                owned_services=('/perception/count_objects',),
                resources=frozenset({'yolo_gpu_pipeline'}),
            ),
        }

        # Section: One ManagedProcess endpoint is created for each fixed registry entry.
        for spec in self._specs.values():
            self._action_servers.append(
                ActionServer(
                    self,
                    ManagedProcess,
                    spec.action_name,
                    execute_callback=lambda goal_handle, spec=spec: self._execute(spec, goal_handle),
                    goal_callback=lambda goal_request, spec=spec: self._goal_callback(spec, goal_request),
                    cancel_callback=lambda goal_handle, spec=spec: self._cancel_callback(spec, goal_handle),
                    callback_group=self.cb_group,
                )
            )

        # Section: Preserve the existing stop service while restricting it to owned targets.
        self._services.append(
            self.create_service(
                KillManagedProcess,
                '/kill',
                self._handle_kill_service,
                callback_group=self.cb_group,
            )
        )

        # Section: Startup logs expose only fixed logical targets and commands.
        self.get_logger().info('[PhaseSupervisor] Ready')
        for spec in self._specs.values():
            self.get_logger().info(f'  {spec.action_name} -> {" ".join(spec.command_factory())}')
        self.get_logger().info(
            '  /kill -> putrabot_interfaces/srv/KillManagedProcess '
            '{target: <managed-target>, generation: <generation>}'
        )

    def destroy_node(self):
        """Special function: Gracefully stop every workload still owned by this supervisor."""
        self._stop_all_processes(reason='node shutdown')
        return super().destroy_node()

    def _goal_callback(self, spec: ManagedSpec, goal_request: ManagedProcess.Goal):
        """Special function: Accept fixed-registry starts for execution-time validation."""
        del spec, goal_request
        return GoalResponse.ACCEPT

    def _cancel_callback(self, spec: ManagedSpec, goal_handle):
        """Special function: Accept cancellation so partial startup is torn down."""
        del spec, goal_handle
        return CancelResponse.ACCEPT

    @staticmethod
    def _open_pidfd(root_pid: int) -> Optional[int]:
        """Special function: Capture optional Linux root-process identity protection."""
        if not hasattr(os, 'pidfd_open'):
            return None
        try:
            return os.pidfd_open(root_pid)
        except (OSError, AttributeError):
            return None

    @staticmethod
    def _close_pidfd(handle: ManagedHandle) -> None:
        """Special function: Close the optional root pidfd after ownership ends."""
        if handle.pidfd is None:
            return
        try:
            os.close(handle.pidfd)
        except OSError:
            pass
        handle.pidfd = None

    @staticmethod
    def _process_group_exists(pgid: int) -> bool:
        """Special function: Test the captured process group without process discovery."""
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        except OSError as exc:
            return exc.errno != errno.ESRCH

    def _wait_for_group_exit(self, handle: ManagedHandle, timeout_sec: float) -> bool:
        """Special function: Reap the root and poll until the entire owned group is absent."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            handle.process.poll()
            if not self._process_group_exists(handle.pgid):
                return True
            time.sleep(0.1)
        handle.process.poll()
        return not self._process_group_exists(handle.pgid)

    def _send_group_signal(self, handle: ManagedHandle, signum: int, stage: str) -> None:
        """Special function: Signal only the process group captured when the workload started."""
        self.get_logger().info(
            f'[PhaseSupervisor] {stage} target={handle.spec.name} generation={handle.generation} '
            f'pid={handle.root_pid} pgid={handle.pgid}'
        )
        try:
            os.killpg(handle.pgid, signum)
        except ProcessLookupError:
            pass

    def _terminate_process_group(self, handle: ManagedHandle) -> bool:
        """Special function: Perform bounded SIGINT, SIGTERM, then SIGKILL escalation."""
        if not self._process_group_exists(handle.pgid):
            handle.process.poll()
            return True

        # Section: Request normal ROS shutdown first.
        self._send_group_signal(handle, signal.SIGINT, 'SIGINT')
        if self._wait_for_group_exit(
            handle, float(self.get_parameter('managed_sigint_timeout_sec').value)
        ):
            return True

        # Section: Escalate only when the graceful deadline expires.
        self._send_group_signal(handle, signal.SIGTERM, 'SIGTERM fallback')
        if self._wait_for_group_exit(
            handle, float(self.get_parameter('managed_sigterm_timeout_sec').value)
        ):
            return True

        # Section: Force termination only as the final bounded fallback.
        self._send_group_signal(handle, signal.SIGKILL, 'SIGKILL fallback')
        return self._wait_for_group_exit(
            handle, float(self.get_parameter('managed_sigkill_timeout_sec').value)
        )

    def _action_names(self) -> set[str]:
        """Special function: Snapshot currently discovered ROS action-server names."""
        action_names: set[str] = set()
        for node_name, node_namespace in self.get_node_names_and_namespaces():
            try:
                discovered = get_action_server_names_and_types_by_node(
                    self,
                    node_name,
                    node_namespace,
                )
            except RuntimeError:
                continue
            action_names.update(name for name, _types in discovered)
        return action_names

    def _service_names(self) -> set[str]:
        """Special function: Snapshot currently discovered ROS service names."""
        return {name for name, _types in self.get_service_names_and_types()}

    def _readiness_status(self, spec: ManagedSpec) -> tuple[bool, str]:
        """Special function: Check all declared ROS interfaces and topic dependencies."""
        action_names = self._action_names()
        service_names = self._service_names()
        missing_actions = [name for name in spec.required_actions if name not in action_names]
        missing_services = [name for name in spec.required_services if name not in service_names]
        missing_topics = [name for name in spec.required_topics if self.count_publishers(name) < 1]
        missing = []
        if missing_actions:
            missing.append(f'actions={missing_actions}')
        if missing_services:
            missing.append(f'services={missing_services}')
        if missing_topics:
            missing.append(f'topic_publishers={missing_topics}')
        return not missing, ', '.join(missing) if missing else 'all required interfaces ready'

    def _owned_interfaces_gone(self, spec: ManagedSpec) -> tuple[bool, str]:
        """Special function: Check that only skill-owned ROS endpoints have disappeared."""
        action_names = self._action_names()
        service_names = self._service_names()
        remaining_actions = [name for name in spec.owned_actions if name in action_names]
        remaining_services = [name for name in spec.owned_services if name in service_names]
        remaining = []
        if remaining_actions:
            remaining.append(f'actions={remaining_actions}')
        if remaining_services:
            remaining.append(f'services={remaining_services}')
        return not remaining, ', '.join(remaining) if remaining else 'owned interfaces absent'

    def _wait_for_readiness(
        self,
        handle: ManagedHandle,
        cancel_requested: Optional[Callable[[], bool]],
    ) -> tuple[bool, str, str]:
        """Special function: Wait for declared readiness while monitoring process and cancellation."""
        spec = handle.spec
        if not (spec.required_actions or spec.required_services or spec.required_topics):
            deadline = time.monotonic() + spec.startup_sec
            while time.monotonic() < deadline:
                if cancel_requested is not None and cancel_requested():
                    return False, 'startup canceled', 'canceled'
                if handle.process.poll() is not None:
                    return False, f'process exited early with code {handle.process.returncode}', 'abort'
                time.sleep(0.1)
            return True, 'legacy startup delay completed', 'ready'

        self.get_logger().info(
            f'[PhaseSupervisor] waiting for readiness target={spec.name} '
            f'generation={handle.generation}'
        )
        deadline = time.monotonic() + spec.startup_timeout_sec
        last_detail = 'readiness not checked'
        while time.monotonic() < deadline:
            if cancel_requested is not None and cancel_requested():
                return False, 'startup canceled', 'canceled'
            if handle.process.poll() is not None:
                return False, f'process exited early with code {handle.process.returncode}', 'abort'
            ready, last_detail = self._readiness_status(spec)
            if ready:
                return True, last_detail, 'ready'
            time.sleep(0.1)
        return False, f'readiness timeout: {last_detail}', 'abort'

    def _wait_for_graph_teardown(self, handle: ManagedHandle) -> tuple[bool, str]:
        """Special function: Poll ROS discovery until owned endpoints are absent."""
        deadline = time.monotonic() + float(
            self.get_parameter('managed_graph_timeout_sec').value
        )
        last_detail = 'graph teardown not checked'
        while time.monotonic() < deadline:
            gone, last_detail = self._owned_interfaces_gone(handle.spec)
            if gone:
                return True, last_detail
            time.sleep(0.1)
        return self._owned_interfaces_gone(handle.spec)

    def _resource_conflict(self, spec: ManagedSpec) -> Optional[str]:
        """Special function: Return the first conflicting logical resource owner."""
        for resource in sorted(spec.resources):
            owner = self._resource_owners.get(resource)
            if owner is not None:
                return f'resource conflict: {resource} owned by {owner[0]} generation={owner[1]}'
        return None

    def _spawn_process(self, spec: ManagedSpec) -> tuple[Optional[ManagedHandle], str]:
        """Special function: Atomically acquire resources and capture process identity."""
        with self._lock:
            if spec.name in self._handles:
                return None, f'{spec.name} already has an owned generation'
            conflict = self._resource_conflict(spec)
            if conflict is not None:
                self.get_logger().error(f'[PhaseSupervisor] {conflict}')
                return None, conflict

            generation = str(uuid.uuid4())
            self.get_logger().info(
                f'[PhaseSupervisor] generation created target={spec.name} generation={generation}'
            )
            try:
                process = subprocess.Popen(
                    spec.command_factory(),
                    cwd=os.getcwd(),
                    shell=False,
                    start_new_session=True,
                )
                root_pid = process.pid
                pgid = os.getpgid(root_pid)
            except Exception as exc:
                return None, f'failed to spawn {spec.name}: {exc}'

            handle = ManagedHandle(
                spec=spec,
                generation=generation,
                process=process,
                root_pid=root_pid,
                pgid=pgid,
                pidfd=self._open_pidfd(root_pid),
                state='STARTING',
                start_time=time.monotonic(),
                resources=spec.resources,
            )
            self._handles[spec.name] = handle
            for resource in spec.resources:
                self._resource_owners[resource] = (spec.name, generation)
                self.get_logger().info(
                    f'[PhaseSupervisor] resource acquired target={spec.name} '
                    f'generation={generation} resource={resource}'
                )
            self.get_logger().info(
                f'[PhaseSupervisor] START requested target={spec.name} generation={generation} '
                f'pid={root_pid} pgid={pgid} pidfd={handle.pidfd}'
            )
            return handle, ''

    def _release_handle(self, handle: ManagedHandle) -> None:
        """Special function: Release ownership only after all teardown barriers pass."""
        with self._lock:
            current = self._handles.get(handle.spec.name)
            if current is not handle:
                return
            for resource in handle.resources:
                owner = self._resource_owners.get(resource)
                if owner == (handle.spec.name, handle.generation):
                    self._resource_owners.pop(resource, None)
                    self.get_logger().info(
                        f'[PhaseSupervisor] resource released target={handle.spec.name} '
                        f'generation={handle.generation} resource={resource}'
                    )
            self._handles.pop(handle.spec.name, None)
            self._close_pidfd(handle)

    def _stop_process(
        self,
        name: str,
        reason: str,
        expected_generation: str = '',
    ) -> tuple[bool, str]:
        """Special function: Stop one exact owned generation and verify complete teardown."""
        with self._lock:
            handle = self._handles.get(name)
            if handle is None:
                return True, f'{name} is already stopped'
            if name == 'gpsr_count_objects' and not expected_generation:
                message = f'generation is required to stop {name}'
                self.get_logger().error(f'[PhaseSupervisor] {message}')
                return False, message
            if expected_generation and expected_generation != handle.generation:
                message = (
                    f'generation mismatch for {name}: requested={expected_generation} '
                    f'current={handle.generation}'
                )
                self.get_logger().error(f'[PhaseSupervisor] {message}')
                return False, message
            handle.state = 'STOPPING'

        self.get_logger().info(
            f'[PhaseSupervisor] skill STOP requested target={name} generation={handle.generation} '
            f'pid={handle.root_pid} pgid={handle.pgid} reason={reason}'
        )
        if not self._terminate_process_group(handle):
            handle.state = 'FAILED_TEARDOWN'
            message = (
                f'FAILED_TEARDOWN target={name} generation={handle.generation}: '
                f'process group {handle.pgid} still exists'
            )
            self.get_logger().error(f'[PhaseSupervisor] {message}')
            return False, message

        self.get_logger().info(
            f'[PhaseSupervisor] process teardown confirmed target={name} '
            f'generation={handle.generation} exit_code={handle.process.poll()}'
        )
        graph_gone, graph_detail = self._wait_for_graph_teardown(handle)
        if not graph_gone:
            handle.state = 'FAILED_TEARDOWN'
            message = (
                f'FAILED_TEARDOWN target={name} generation={handle.generation}: '
                f'ROS graph still contains {graph_detail}'
            )
            self.get_logger().error(f'[PhaseSupervisor] {message}')
            return False, message

        self.get_logger().info(
            f'[PhaseSupervisor] ROS graph teardown confirmed target={name} '
            f'generation={handle.generation}'
        )
        self._release_handle(handle)
        return True, f'stopped managed launch {name} generation={handle.generation}'

    def _stop_all_processes(self, reason: str) -> None:
        """Special function: Stop only workloads represented by supervisor-owned handles."""
        with self._lock:
            owned = [(name, handle.generation) for name, handle in self._handles.items()]
        for name, generation in owned:
            success, message = self._stop_process(name, reason, generation)
            if not success:
                self.get_logger().error(f'[PhaseSupervisor] shutdown cleanup failed: {message}')

    def _resolve_managed_target(self, target: str) -> Optional[str]:
        """Special function: Resolve only fixed logical names or their registered action names."""
        normalized = target.strip()
        normalized_without_slash = normalized.lstrip('/')
        for name, spec in self._specs.items():
            if normalized in (name, spec.action_name):
                return name
            if normalized_without_slash == spec.action_name.lstrip('/'):
                return name
            if normalized_without_slash == spec.action_name.rsplit('/', 1)[-1]:
                return name
        return None

    def _handle_kill_service(self, request, response):
        """Special function: Stop a logical target with optional generation protection."""
        target = request.target.strip()
        generation = str(getattr(request, 'generation', '')).strip()
        managed_name = self._resolve_managed_target(target)
        if managed_name is None:
            response.success = False
            response.message = (
                f'unknown managed target "{target}"; arbitrary PID/node-name discovery is disabled'
            )
            return response
        response.success, response.message = self._stop_process(
            managed_name,
            reason=f'/kill requested target "{target}"',
            expected_generation=generation,
        )
        return response

    @staticmethod
    def _result(
        success: bool,
        already_running: bool,
        message: str,
        generation: str = '',
    ) -> ManagedProcess.Result:
        """Special function: Build a ManagedProcess result with generation identity."""
        result = ManagedProcess.Result()
        result.success = bool(success)
        result.already_running = bool(already_running)
        result.message = message
        if hasattr(result, 'generation'):
            result.generation = generation
        return result

    @staticmethod
    def _feedback(goal_handle, state: str, detail: str) -> None:
        """Special function: Publish concise managed-process state feedback."""
        feedback = ManagedProcess.Feedback()
        feedback.state = state
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def _run_managed_process(
        self,
        spec: ManagedSpec,
        force_restart: bool,
        cancel_requested: Optional[Callable[[], bool]] = None,
        feedback_callback: Optional[Callable[[str, str], None]] = None,
    ) -> tuple[bool, bool, str, str, str]:
        """Special function: Start one target and return only after its readiness barrier."""
        with self._start_lock:
            if spec.availability_check is not None:
                available, detail = spec.availability_check()
                if not available:
                    message = f'{spec.name} unavailable: {detail}'
                    self.get_logger().error(f'[PhaseSupervisor] FAILED_STARTUP {message}')
                    return False, False, message, 'abort', ''

            with self._lock:
                existing = self._handles.get(spec.name)
            if existing is not None:
                if existing.state == 'READY' and not force_restart:
                    return (
                        True,
                        True,
                        f'{spec.name} is already ready',
                        'succeeded',
                        existing.generation,
                    )
                if existing.state == 'STARTING' and not force_restart:
                    return (
                        False,
                        True,
                        f'duplicate start rejected while {spec.name} is STARTING',
                        'abort',
                        existing.generation,
                    )
                if existing.state == 'FAILED_TEARDOWN':
                    return (
                        False,
                        True,
                        f'{spec.name} is blocked by FAILED_TEARDOWN generation={existing.generation}',
                        'abort',
                        existing.generation,
                    )
                stopped, stop_message = self._stop_process(
                    spec.name,
                    reason='force restart' if force_restart else 'clean stale generation',
                    expected_generation=existing.generation,
                )
                if not stopped:
                    return False, True, stop_message, 'abort', existing.generation

            handle, spawn_error = self._spawn_process(spec)
            if handle is None:
                self.get_logger().error(f'[PhaseSupervisor] FAILED_STARTUP {spawn_error}')
                return False, False, spawn_error, 'abort', ''

            if feedback_callback is not None:
                feedback_callback('starting', f'launching {spec.name} generation={handle.generation}')
            ready, detail, terminal_state = self._wait_for_readiness(handle, cancel_requested)
            if not ready:
                stopped, stop_message = self._stop_process(
                    spec.name,
                    reason=f'FAILED_STARTUP: {detail}',
                    expected_generation=handle.generation,
                )
                message = f'FAILED_STARTUP {spec.name} generation={handle.generation}: {detail}'
                if not stopped:
                    message = f'{message}; cleanup failed: {stop_message}'
                self.get_logger().error(f'[PhaseSupervisor] {message}')
                return False, False, message, terminal_state, handle.generation

            handle.state = 'READY'
            self.get_logger().info(
                f'[PhaseSupervisor] READY target={spec.name} generation={handle.generation} '
                f'pid={handle.root_pid} pgid={handle.pgid}'
            )
            if feedback_callback is not None:
                feedback_callback('ready', f'{spec.name} generation={handle.generation} is ready')

            if not spec.wait_for_exit:
                return True, False, f'{spec.name} ready', 'succeeded', handle.generation

            while handle.process.poll() is None:
                if cancel_requested is not None and cancel_requested():
                    stopped, stop_message = self._stop_process(
                        spec.name, 'goal canceled', handle.generation
                    )
                    return False, False, stop_message, 'canceled', handle.generation
                time.sleep(0.1)
            exit_code = handle.process.returncode
            stopped, stop_message = self._stop_process(
                spec.name, 'managed process exited', handle.generation
            )
            if exit_code == 0 and stopped:
                return True, False, f'{spec.name} completed successfully', 'succeeded', handle.generation
            return False, False, stop_message, 'abort', handle.generation

    def _execute(self, spec: ManagedSpec, goal_handle):
        """Special function: Execute one fixed ManagedProcess start request."""
        if spec.trigger_callback is not None:
            return spec.trigger_callback(goal_handle)
        success, already_running, message, terminal_state, generation = self._run_managed_process(
            spec,
            force_restart=bool(goal_handle.request.force_restart),
            cancel_requested=lambda: goal_handle.is_cancel_requested,
            feedback_callback=lambda state, detail: self._feedback(goal_handle, state, detail),
        )
        if success:
            goal_handle.succeed()
            return self._result(True, already_running, message, generation)
        if terminal_state == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return self._result(False, already_running, message, generation)


def main(args=None):
    """Special function: Spin the persistent supervisor and clean owned workloads on exit."""
    rclpy.init(args=args)
    node = PhaseSupervisor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    shutdown_requested = threading.Event()

    # Section: Signal handling initiates bounded teardown of owned workloads only.
    def _request_shutdown(signum, _frame):
        """Special function: Convert process signals into managed supervisor shutdown."""
        if shutdown_requested.is_set():
            return
        shutdown_requested.set()
        node.get_logger().info(f'[PhaseSupervisor] received signal {signum}, shutting down')

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)
    try:
        # Section: A bounded spin loop observes custom signal-handler shutdown reliably.
        while rclpy.ok() and not shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        shutdown_requested.set()
    finally:
        node._stop_all_processes(reason='main shutdown')
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
