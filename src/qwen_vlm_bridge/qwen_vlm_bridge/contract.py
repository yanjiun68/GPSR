"""Pure input and output contracts shared by the Qwen CLI and ROS node."""

from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any, Iterable
import uuid


# Section: Enumerate every semantic skill currently defined by the installed Qwen GPSR prompt.
DEFAULT_ALLOWED_SKILLS = (
    "MOVE_TO",
    "FIND_PERSON",
    "VERIFY_PERSON",
    "FIND_OBJECT",
    "PICK",
    "HANDOVER",
    "PLACE",
    "OBSERVE",
    "FIND_EXTREME_OBJECT",
    "COUNT_OBJECTS",
    "COUNT_PEOPLE",
    "SPEAK_RESULT",
    "DESCRIBE_PERSON",
    "FOLLOW_PERSON",
    "GUIDE_PERSON",
    "ANSWER_QUESTION",
    "FIXED_INFORMATION",
    "SPEAK",
)


@dataclass(frozen=True)
class PlannerRequest:
    """Special function: Preserve source and safety metadata for one inference request."""

    request_id: str
    source: str
    instruction: str
    dry_run: bool = True
    submit_requested: bool = False


class ContractError(ValueError):
    """Special function: Identify input or model-output contract violations."""


def parse_input(
    payload: str,
    source: str,
    *,
    allow_envelope: bool,
    max_instruction_chars: int,
) -> PlannerRequest:
    """Special function: Convert plain text or a debug JSON envelope into a bounded request."""
    # Section: Remove transport whitespace before deciding whether the request is usable.
    text = str(payload or "").strip()
    if not text:
        raise ContractError("instruction is empty")

    # Section: Treat JSON as an envelope only on explicitly enabled debug-style interfaces.
    envelope: dict[str, Any] | None = None
    if allow_envelope and text.startswith("{"):
        try:
            decoded = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ContractError(f"invalid JSON debug envelope: {exc.msg}") from exc
        if not isinstance(decoded, dict):
            raise ContractError("debug envelope must be a JSON object")
        envelope = decoded

    # Section: Preserve convenient plain-text publishing while supporting correlated debug requests.
    if envelope is None:
        instruction = text
        request_id = f"{source}_{uuid.uuid4().hex}"
        dry_run = True
        submit_requested = False
    else:
        instruction = str(envelope.get("instruction") or "").strip()
        request_id = str(envelope.get("request_id") or f"{source}_{uuid.uuid4().hex}").strip()
        dry_run = bool(envelope.get("dry_run", True))
        submit_requested = bool(envelope.get("submit", False))

    # Section: Reject malformed identifiers and unbounded prompts before allocating model work.
    if not request_id:
        raise ContractError("request_id is empty")
    if not instruction:
        raise ContractError("instruction is empty")
    if len(instruction) > max(1, int(max_instruction_chars)):
        raise ContractError(
            f"instruction exceeds {max(1, int(max_instruction_chars))} characters"
        )

    # Section: Return an immutable request so worker-thread ownership is unambiguous.
    return PlannerRequest(
        request_id=request_id,
        source=source,
        instruction=instruction,
        dry_run=dry_run,
        submit_requested=submit_requested,
    )


def parse_model_json(raw_response: str) -> dict[str, Any]:
    """Special function: Parse strict JSON without silently accepting Markdown wrappers."""
    # Section: Empty output and fenced Markdown violate the prompt's machine-readable contract.
    text = str(raw_response or "").strip()
    if not text:
        raise ContractError("model returned an empty response")
    if text.startswith("```"):
        raise ContractError("model returned Markdown instead of JSON")
    try:
        decoded = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ContractError(f"model returned invalid JSON: {exc.msg}") from exc
    if not isinstance(decoded, dict):
        raise ContractError("model response must be a JSON object")
    return decoded


def validate_plan(
    plan: dict[str, Any],
    allowed_skills: Iterable[str] = DEFAULT_ALLOWED_SKILLS,
) -> dict[str, Any]:
    """Special function: Validate the semantic plan before publishing it to another ROS node."""
    # Section: Normalize the configured allow-list once for exact, case-sensitive skill admission.
    allowed = {str(skill).strip() for skill in allowed_skills if str(skill).strip()}
    if not isinstance(plan.get("requires_clarification"), bool):
        raise ContractError("requires_clarification must be a boolean")
    actions = plan.get("actions")
    if not isinstance(actions, list):
        raise ContractError("actions must be a JSON array")

    # Section: A clarification result must contain no executable actions and must name missing slots.
    if plan["requires_clarification"]:
        if actions:
            raise ContractError("clarification response must not contain actions")
        missing = plan.get("missing")
        if not isinstance(missing, list) or not missing:
            raise ContractError("clarification response must contain a non-empty missing array")
        if any(not isinstance(item, str) or not item.strip() for item in missing):
            raise ContractError("missing entries must be non-empty strings")
        return plan

    # Section: An executable response requires at least one admitted action with object parameters.
    if not actions:
        raise ContractError("executable response must contain at least one action")
    for index, action in enumerate(actions):
        if not isinstance(action, dict):
            raise ContractError(f"actions[{index}] must be a JSON object")
        skill = str(action.get("skill") or "").strip()
        if not skill:
            raise ContractError(f"actions[{index}].skill is empty")
        if allowed and skill not in allowed:
            raise ContractError(f"actions[{index}].skill is not allowed: {skill}")
        if not isinstance(action.get("params"), dict):
            raise ContractError(f"actions[{index}].params must be a JSON object")
    return plan
