"""Section: Deterministic GPSR scheduler facade."""

# Section: Public deterministic scheduling facade.

from __future__ import annotations

from gpsr_task_scheduler.config import SchedulerConfig
from gpsr_task_scheduler.models import SchedulerResult
from gpsr_task_scheduler.semantic_compiler import compile_command


def schedule_command(
    command_id: str,
    instruction: str,
    config: SchedulerConfig,
    normalized_instruction: str | None = None,
) -> SchedulerResult:
    """Special function: Canonicalize, validate, and plan without LLM execution authority."""
    return compile_command(
        command_id,
        instruction,
        config,
        normalized_text=normalized_instruction,
    )
