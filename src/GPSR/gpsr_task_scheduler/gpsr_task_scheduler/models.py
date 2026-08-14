"""Section: Strongly typed scheduler data models."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class ValidationIssue:
    """Special function: Represent one deterministic parser or planner validation issue."""

    code: str
    message: str


@dataclass(slots=True)
class TaskRequest:
    """Special function: Represent command semantics independent of original phrase order."""

    command_id: str
    task_type: str
    task_location: str | None
    report_location: str | None
    return_location: str | None
    target: dict[str, Any] = field(default_factory=dict)
    filters: list[dict[str, Any]] = field(default_factory=list)
    recipient: dict[str, Any] | None = None
    output_template: str | None = None
    return_after_completion: bool = False


@dataclass(slots=True)
class SkillStep:
    """Special function: Represent one deterministic reusable skill invocation."""

    step_id: str
    skill_name: str
    arguments: dict[str, Any] = field(default_factory=dict)
    dependencies: list[str] = field(default_factory=list)
    timeout: float = 0.0
    retry_count: int = 0
    result_variable: str | None = None


@dataclass(slots=True)
class SchedulerResult:
    """Special function: Return either a complete ordered plan or validation issues."""

    command_id: str
    task_request: TaskRequest | None
    steps: list[SkillStep] = field(default_factory=list)
    issues: list[ValidationIssue] = field(default_factory=list)

    @property
    def valid(self) -> bool:
        """Special function: Report whether the scheduler produced an executable plan."""
        return not self.issues and self.task_request is not None
