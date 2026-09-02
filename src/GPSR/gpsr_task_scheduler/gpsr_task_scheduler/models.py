"""Section: Strongly typed scheduler data models."""

# Section: Typed scheduler requests, plans, return targets, and results.

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class ValidationIssue:
    """Special function: Represent one deterministic parser or planner validation issue."""

    code: str
    message: str


@dataclass(slots=True)
class SemanticStep:
    """Special function: Represent one ordered intent before primitive expansion."""

    step_id: str
    intent: str
    clause: str
    arguments: dict[str, Any] = field(default_factory=dict)
    attributes: dict[str, Any] = field(default_factory=dict)
    relationship: str = "then"

    def to_dict(self) -> dict[str, Any]:
        """Special function: Serialize one semantic step for logs and dry runs."""
        return {
            "id": self.step_id,
            "intent": self.intent,
            "clause": self.clause,
            "arguments": dict(self.arguments),
            "attributes": dict(self.attributes),
            "relationship": self.relationship,
        }


@dataclass(slots=True)
class CommandContext:
    """Special function: Preserve every deterministic command-processing stage."""

    command_id: str
    raw_text: str
    normalized_text: str
    clauses: list[dict[str, Any]] = field(default_factory=list)
    entities: dict[str, Any] = field(default_factory=dict)
    references: dict[str, Any] = field(default_factory=dict)
    semantic_steps: list[SemanticStep] = field(default_factory=list)
    action_queue: list[dict[str, Any]] = field(default_factory=list)
    validation: dict[str, Any] = field(
        default_factory=lambda: {"valid": False, "errors": [], "warnings": []}
    )

    def to_dict(self) -> dict[str, Any]:
        """Special function: Serialize the complete context without losing raw text."""
        return {
            "command_id": self.command_id,
            "raw_text": self.raw_text,
            "normalized_text": self.normalized_text,
            "clauses": list(self.clauses),
            "entities": dict(self.entities),
            "references": dict(self.references),
            "semantic_steps": [step.to_dict() for step in self.semantic_steps],
            "action_queue": list(self.action_queue),
            "validation": dict(self.validation),
        }


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
    person_selector: dict[str, Any] = field(default_factory=dict)
    requested_person_fields: list[str] = field(default_factory=list)
    recipient: dict[str, Any] | None = None
    output_template: str | None = None
    return_after_completion: bool = False
    report_after_return: bool = False


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


@dataclass(frozen=True, slots=True)
class ReturnTarget:
    """Special function: Freeze one command's raw-pose or named return destination."""

    source: str
    target_location: str = ""
    target_pose: dict[str, Any] | None = None


@dataclass(slots=True)
class SchedulerResult:
    """Special function: Return either a complete ordered plan or validation issues."""

    command_id: str
    task_request: TaskRequest | None
    steps: list[SkillStep] = field(default_factory=list)
    issues: list[ValidationIssue] = field(default_factory=list)
    post_return_steps: list[SkillStep] = field(default_factory=list)
    command_context: CommandContext | None = None

    @property
    def valid(self) -> bool:
        """Special function: Report whether the scheduler produced an executable plan."""
        return not self.issues and self.task_request is not None
