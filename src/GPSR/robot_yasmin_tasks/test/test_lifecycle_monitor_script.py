"""Section: Lifecycle monitor stable-idle semantics tests."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path


def _fake_ros2(tmp_path: Path) -> Path:
    """Special function: Create a deterministic lifecycle CLI used only by this test."""
    executable = tmp_path / "ros2"
    executable.write_text(
        "#!/usr/bin/env bash\n"
        "# Section: Return two discovered lifecycle nodes.\n"
        "if [[ \"$1 $2\" == \"lifecycle nodes\" ]]; then\n"
        "  printf '/active_node\\n/idle_node\\n'\n"
        "  exit 0\n"
        "fi\n"
        "# Section: Return one ACTIVE and one healthy UNCONFIGURED state.\n"
        "if [[ \"$1 $2 $3\" == \"lifecycle get /active_node\" ]]; then\n"
        "  printf 'active [3]\\n'\n"
        "  exit 0\n"
        "fi\n"
        "printf 'unconfigured [1]\\n'\n",
        encoding="utf-8",
    )
    executable.chmod(0o755)
    return executable


def _run_monitor(tmp_path: Path, *arguments: str) -> subprocess.CompletedProcess[str]:
    """Special function: Execute the real monitor against the fake ROS lifecycle CLI."""
    _fake_ros2(tmp_path)
    repository = Path(__file__).resolve().parents[4]
    script = repository / "scripts" / "lifecycle" / "show_all_lifecycle_states.sh"
    environment = dict(os.environ)
    environment["PATH"] = f"{tmp_path}:{environment['PATH']}"
    return subprocess.run(
        ["bash", str(script), *arguments],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def test_idle_unconfigured_nodes_are_healthy_by_default(tmp_path: Path) -> None:
    """Special function: Transient idle nodes must not make the runtime look broken."""
    result = _run_monitor(tmp_path)
    assert result.returncode == 0
    assert "HEALTHY (ACTIVE OR IDLE)" in result.stdout


def test_require_active_mode_retains_strict_manual_check(tmp_path: Path) -> None:
    """Special function: Operators can still require every discovered node to be ACTIVE."""
    result = _run_monitor(tmp_path, "--require-active")
    assert result.returncode == 1
    assert "NOT ALL NODES ACTIVE" in result.stdout
