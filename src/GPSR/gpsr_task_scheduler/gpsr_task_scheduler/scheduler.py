"""Section: Deterministic GPSR scheduler facade."""

# Section: Public deterministic scheduling facade.

from __future__ import annotations

from gpsr_task_scheduler.config import SchedulerConfig
from gpsr_task_scheduler.clarification import assess_command
from gpsr_task_scheduler.models import CommandContext, SchedulerResult, ValidationIssue
from gpsr_task_scheduler.semantic_compiler import _semantic_drift_issues, compile_command


def schedule_command(
    command_id: str,
    instruction: str,
    config: SchedulerConfig,
    normalized_instruction: str | None = None,
) -> SchedulerResult:
    """Special function: Canonicalize, validate, and plan without LLM execution authority."""
    # Section: The shared facade refuses incomplete semantics before any queue is built.
    candidate = normalized_instruction if normalized_instruction is not None else instruction
    # Section: Critical normalization drift is rejected before considering clarification.
    if normalized_instruction is not None and _semantic_drift_issues(str(instruction), str(candidate), config):
        return compile_command(command_id, instruction, config, normalized_text=normalized_instruction)
    # Section: A normalizer cannot supply an action or entity absent from raw speech.
    raw_completeness = assess_command(instruction, config)
    completeness = raw_completeness if not raw_completeness.complete else assess_command(candidate, config)
    if not completeness.complete:
        context = CommandContext(command_id, str(instruction or ""), str(candidate or ""))
        context.validation = {
            "valid": False,
            "status": completeness.status,
            "missing_fields": list(completeness.missing_fields),
            "clarification_prompt": completeness.prompt,
            "partial": {
                "intent": completeness.intent,
                "target": completeness.target,
                "location": completeness.location,
            },
            "errors": [{"code": "CLARIFICATION_REQUIRED", "message": completeness.prompt}],
            "warnings": [],
        }
        return SchedulerResult(
            command_id,
            None,
            [],
            [ValidationIssue("CLARIFICATION_REQUIRED", completeness.prompt)],
            command_context=context,
        )
    return compile_command(
        command_id,
        instruction,
        config,
        normalized_text=normalized_instruction,
    )
