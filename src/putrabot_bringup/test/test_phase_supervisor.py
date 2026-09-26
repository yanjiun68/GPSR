"""Section: Managed process ownership and safety regression tests."""

import importlib.util
import signal
import threading
from pathlib import Path
from types import SimpleNamespace


# Section: Load the installed-script-style module directly from the source tree.
MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "phase_supervisor.py"
MODULE_SPEC = importlib.util.spec_from_file_location("phase_supervisor_under_test", MODULE_PATH)
phase_supervisor = importlib.util.module_from_spec(MODULE_SPEC)
assert MODULE_SPEC.loader is not None
MODULE_SPEC.loader.exec_module(phase_supervisor)
ManagedHandle = phase_supervisor.ManagedHandle
ManagedSpec = phase_supervisor.ManagedSpec
PhaseSupervisor = phase_supervisor.PhaseSupervisor


class FakeLogger:
    """Special function: Capture supervisor logs without requiring a ROS node."""

    def __init__(self) -> None:
        """Special function: Initialize an empty log record."""
        self.messages: list[tuple[str, str]] = []

    def info(self, message: str) -> None:
        """Special function: Capture informational messages."""
        self.messages.append(("info", message))

    def warn(self, message: str) -> None:
        """Special function: Capture warning messages."""
        self.messages.append(("warn", message))

    def error(self, message: str) -> None:
        """Special function: Capture error messages."""
        self.messages.append(("error", message))


class FakeProcess:
    """Special function: Minimal Popen-compatible process used by ownership tests."""

    def __init__(self, pid: int = 1234, returncode=None) -> None:
        """Special function: Store deterministic root PID and return code."""
        self.pid = pid
        self.returncode = returncode

    def poll(self):
        """Special function: Return the configured process state."""
        return self.returncode


def make_spec(resources=frozenset({"yolo_gpu_pipeline"})) -> ManagedSpec:
    """Special function: Build a deterministic COUNT_OBJECTS-like managed specification."""
    return ManagedSpec(
        name="gpsr_count_objects",
        action_name="/bringup/gpsr_count_objects",
        command_factory=lambda: ["ros2", "launch", "vision", "gpsr_count_objects_skill.launch.py"],
        owned_actions=("/gpsr/count_objects",),
        owned_services=("/perception/count_objects",),
        resources=resources,
    )


def make_handle(generation: str = "generation-a") -> ManagedHandle:
    """Special function: Build a deterministic supervisor-owned handle."""
    return ManagedHandle(
        spec=make_spec(),
        generation=generation,
        process=FakeProcess(),
        root_pid=1234,
        pgid=1234,
        pidfd=None,
        state="READY",
        start_time=1.0,
        resources=frozenset({"yolo_gpu_pipeline"}),
    )


def test_stale_generation_stop_never_signals_current_workload() -> None:
    """Special function: STOP(A) cannot affect a currently owned generation B."""
    handle = make_handle("generation-b")
    signals: list[str] = []
    harness = SimpleNamespace(
        _lock=threading.RLock(),
        _handles={"gpsr_count_objects": handle},
        get_logger=lambda: FakeLogger(),
        _terminate_process_group=lambda _handle: signals.append("signal") or True,
    )
    success, message = PhaseSupervisor._stop_process(
        harness,
        "gpsr_count_objects",
        "late stop",
        "generation-a",
    )
    assert success is False
    assert "generation mismatch" in message
    assert signals == []
    assert harness._handles["gpsr_count_objects"] is handle


def test_count_objects_stop_requires_generation() -> None:
    """Special function: An unversioned stop cannot signal the active COUNT_OBJECTS group."""
    handle = make_handle("generation-current")
    signals: list[str] = []
    harness = SimpleNamespace(
        _lock=threading.RLock(),
        _handles={"gpsr_count_objects": handle},
        get_logger=lambda: FakeLogger(),
        _terminate_process_group=lambda _handle: signals.append("signal") or True,
    )
    success, message = PhaseSupervisor._stop_process(
        harness,
        "gpsr_count_objects",
        "unsafe unversioned stop",
        "",
    )
    assert success is False
    assert "generation is required" in message
    assert signals == []


def test_resource_conflict_reports_current_owner() -> None:
    """Special function: Exclusive YOLO ownership blocks another managed specification."""
    harness = SimpleNamespace(
        _resource_owners={"yolo_gpu_pipeline": ("gpsr_count_objects", "generation-a")}
    )
    conflict = PhaseSupervisor._resource_conflict(harness, make_spec())
    assert "yolo_gpu_pipeline" in conflict
    assert "generation-a" in conflict


def test_signal_escalation_uses_sigint_then_sigterm_then_sigkill() -> None:
    """Special function: Teardown escalates only after each bounded wait fails."""
    handle = make_handle()
    signals: list[int] = []
    waits = iter([False, False, True])
    harness = SimpleNamespace(
        _process_group_exists=lambda _pgid: True,
        _send_group_signal=lambda _handle, signum, _stage: signals.append(signum),
        _wait_for_group_exit=lambda _handle, _timeout: next(waits),
        get_parameter=lambda _name: SimpleNamespace(value=0.1),
    )
    assert PhaseSupervisor._terminate_process_group(harness, handle) is True
    assert signals == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]


def test_failed_graph_teardown_retains_handle_and_resource() -> None:
    """Special function: Graph uncertainty fails closed and retains resource ownership."""
    handle = make_handle()
    logger = FakeLogger()
    harness = SimpleNamespace(
        _lock=threading.RLock(),
        _handles={"gpsr_count_objects": handle},
        _resource_owners={"yolo_gpu_pipeline": ("gpsr_count_objects", handle.generation)},
        get_logger=lambda: logger,
        _terminate_process_group=lambda _handle: True,
        _wait_for_graph_teardown=lambda _handle: (False, "actions=['/gpsr/count_objects']"),
    )
    success, message = PhaseSupervisor._stop_process(
        harness,
        "gpsr_count_objects",
        "test",
        handle.generation,
    )
    assert success is False
    assert "FAILED_TEARDOWN" in message
    assert handle.state == "FAILED_TEARDOWN"
    assert harness._handles["gpsr_count_objects"] is handle
    assert harness._resource_owners["yolo_gpu_pipeline"][1] == handle.generation


def test_spawn_captures_owned_pid_pgid_pidfd_and_new_session(monkeypatch) -> None:
    """Special function: Spawn records identity immediately without shell or discovery."""
    popen_calls: list[dict] = []
    process = FakeProcess(pid=4321)

    def fake_popen(command, **kwargs):
        """Special function: Capture the exact safe Popen invocation."""
        popen_calls.append({"command": command, **kwargs})
        return process

    monkeypatch.setattr(phase_supervisor.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(phase_supervisor.os, "getpgid", lambda pid: pid)
    logger = FakeLogger()
    harness = SimpleNamespace(
        _lock=threading.RLock(),
        _handles={},
        _resource_owners={},
        _resource_conflict=lambda _spec: None,
        _open_pidfd=lambda _pid: 99,
        get_logger=lambda: logger,
    )
    handle, error = PhaseSupervisor._spawn_process(harness, make_spec())
    assert error == ""
    assert handle is not None
    assert handle.root_pid == 4321
    assert handle.pgid == 4321
    assert handle.pidfd == 99
    assert handle.generation
    assert popen_calls[0]["shell"] is False
    assert popen_calls[0]["start_new_session"] is True
    assert harness._resource_owners["yolo_gpu_pipeline"][1] == handle.generation


def test_readiness_waits_for_all_interfaces() -> None:
    """Special function: Readiness does not succeed on Popen alone."""
    spec = ManagedSpec(
        name="gpsr_count_objects",
        action_name="/bringup/gpsr_count_objects",
        command_factory=lambda: [],
        startup_timeout_sec=1.0,
        required_actions=("/gpsr/count_objects",),
    )
    handle = make_handle()
    handle.spec = spec
    checks = iter([(False, "missing"), (False, "missing"), (True, "ready")])
    harness = SimpleNamespace(
        get_logger=lambda: FakeLogger(),
        _readiness_status=lambda _spec: next(checks),
    )
    ready, detail, state = PhaseSupervisor._wait_for_readiness(harness, handle, None)
    assert ready is True
    assert detail == "ready"
    assert state == "ready"


def test_readiness_failure_requests_partial_workload_cleanup() -> None:
    """Special function: A failed startup always invokes generation-matched cleanup."""
    spec = make_spec()
    handle = make_handle()
    stopped: list[tuple[str, str]] = []
    harness = SimpleNamespace(
        _start_lock=threading.Lock(),
        _lock=threading.RLock(),
        _handles={},
        get_logger=lambda: FakeLogger(),
        _spawn_process=lambda _spec: (handle, ""),
        _wait_for_readiness=lambda _handle, _cancel: (False, "interfaces missing", "abort"),
        _stop_process=lambda name, reason, expected_generation: stopped.append(
            (name, expected_generation)
        )
        or (True, f"stopped: {reason}"),
    )
    result = PhaseSupervisor._run_managed_process(harness, spec, False)
    assert result[0] is False
    assert "FAILED_STARTUP" in result[2]
    assert stopped == [("gpsr_count_objects", handle.generation)]
