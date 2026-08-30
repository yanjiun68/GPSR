"""Section: Deterministic GPSR scheduler facade."""

# Section: Public deterministic scheduling facade.

from __future__ import annotations

from gpsr_task_scheduler.config import SchedulerConfig
from gpsr_task_scheduler.models import SchedulerResult
from gpsr_task_scheduler.parser import parse_command
from gpsr_task_scheduler.planner import plan_task


def schedule_command(command_id: str, instruction: str, config: SchedulerConfig) -> SchedulerResult:
    """Special function: Parse and plan a GPSR command without LLM or model inference."""
    request, issues = parse_command(command_id, instruction, config)
    if issues or request is None:
        return SchedulerResult(command_id, request, [], issues)
    return plan_task(request, config)
