"""Section: Deterministic GPSR command parser."""

from __future__ import annotations

import re
from typing import Any

from gpsr_task_scheduler.config import SchedulerConfig, normalize_symbol
from gpsr_task_scheduler.models import TaskRequest, ValidationIssue


def parse_command(command_id: str, instruction: str, config: SchedulerConfig) -> tuple[TaskRequest | None, list[ValidationIssue]]:
    """Special function: Parse constrained GPSR text into semantic task roles."""
    text = " ".join(str(instruction or "").lower().replace("-", " ").split())
    issues: list[ValidationIssue] = []
    if not text:
        return None, [ValidationIssue("empty_instruction", "Instruction is empty.")]
    if re.search(r"\b(it|there|them|him|her)\b", text) and not _contains_known_target(text, config):
        issues.append(ValidationIssue("unresolved_pronoun", "Instruction contains a pronoun without a resolvable target."))
    task_type = _detect_task_type(text)
    if not task_type:
        issues.append(ValidationIssue("unknown_task", "No supported deterministic GPSR task was recognized."))
    known_locations = _locations_in_text(text, config.known_locations)
    unknown_locations = _unknown_location_mentions(text, config.known_locations)
    for location in unknown_locations:
        issues.append(ValidationIssue("unknown_location", f"Unknown location: {location}."))
    report_location = _report_location(text, known_locations)
    task_location = _task_location(text, known_locations, report_location)
    recipient = _recipient(text, config)
    target = _target(text, task_type, config)
    filters = _filters(text, config)
    if task_type and _requires_task_location(task_type) and not task_location:
        issues.append(ValidationIssue("missing_task_location", "Task requires a known task location."))
    if _requires_report(text) and not report_location and len(known_locations) > 1:
        issues.append(ValidationIssue("missing_report_location", "Instruction requests output at a destination, but no known report location was found."))
    if task_type in {"find_object", "count_objects"} and not target:
        issues.append(ValidationIssue("missing_target", "Object task requires a known object or category target."))
    if task_type == "find_person" and recipient is None and not filters:
        issues.append(ValidationIssue("missing_person_target", "Find-person task requires a known person or person filter."))
    if issues:
        return None, issues
    return TaskRequest(
        command_id=command_id,
        task_type=str(task_type),
        task_location=task_location,
        report_location=report_location,
        return_location=config.instruction_point,
        target=target,
        filters=filters,
        recipient=recipient,
        output_template=_output_template(text, task_type),
        return_after_completion=config.return_after_completion,
    ), []


def _contains_known_target(text: str, config: SchedulerConfig) -> bool:
    """Special function: Detect whether a pronoun has an explicit known target nearby."""
    symbols = config.known_objects | config.known_categories | config.known_people
    return any(re.search(rf"\b{re.escape(symbol.replace('_', ' '))}\b", text) for symbol in symbols)


def _detect_task_type(text: str) -> str:
    """Special function: Classify the supported GPSR task family."""
    if re.search(r"\b(how many|count)\b.*\b(people|persons|humans)\b", text):
        return "count_people"
    if re.search(r"\b(how many|count)\b", text):
        return "count_objects"
    if re.search(r"\b(find|locate|look for)\b.*\b(person|man|woman|someone|guest)\b", text):
        return "find_person"
    if re.search(r"\b(find|locate|look for)\b", text):
        return "find_object"
    return ""


def _locations_in_text(text: str, known_locations: set[str]) -> list[str]:
    """Special function: Return known locations in first-appearance order."""
    hits: list[tuple[int, str]] = []
    for location in known_locations:
        phrase = location.replace("_", " ")
        match = re.search(rf"\b{re.escape(phrase)}\b", text)
        if match:
            hits.append((match.start(), location))
    return [location for _, location in sorted(hits)]


def _unknown_location_mentions(text: str, known_locations: set[str]) -> list[str]:
    """Special function: Flag obvious unsupported room/location phrases."""
    unknown: list[str] = []
    stop_words = {"and", "then", "tell", "report", "say", "answer", "count", "find", "locate", "look", "go", "navigate", "with", "for"}
    for match in re.finditer(r"(?=\b(?:in|at|to|from)\s+(?:the\s+)?([a-z_]+(?:\s+[a-z_]+){0,2}))", text):
        words = []
        for word in match.group(1).split():
            if word in stop_words:
                break
            words.append(word)
        candidate = normalize_symbol(" ".join(words))
        if not candidate or candidate in {"person", "people", "object", "objects", "me"}:
            continue
        if candidate not in known_locations and not any(candidate.startswith(location) for location in known_locations):
            unknown.append(candidate)
    return unknown


def _report_location(text: str, known_locations: list[str]) -> str | None:
    """Special function: Resolve where the robot should deliver the output."""
    if len(known_locations) < 2:
        return None
    for location in known_locations:
        phrase = location.replace("_", " ")
        if re.search(rf"\b(?:go|navigate|return)\s+to\s+(?:the\s+)?{re.escape(phrase)}\b", text):
            return location
    for location in known_locations:
        phrase = location.replace("_", " ")
        if re.search(rf"\b(?:tell|report|say|answer)\b.*\b(?:in|at|to)\s+(?:the\s+)?{re.escape(phrase)}\b", text):
            return location
    return None


def _task_location(text: str, known_locations: list[str], report_location: str | None) -> str | None:
    """Special function: Resolve the location where the task must be performed."""
    for location in known_locations:
        phrase = location.replace("_", " ")
        if location == report_location:
            continue
        if re.search(rf"\b(?:in|at|from|on)\s+(?:the\s+)?{re.escape(phrase)}\b", text):
            return location
    for location in known_locations:
        if location != report_location:
            return location
    return None


def _recipient(text: str, config: SchedulerConfig) -> dict[str, Any] | None:
    """Special function: Resolve a named person who must receive the output."""
    for person in config.known_people:
        phrase = person.replace("_", " ")
        if re.search(rf"\b(?:tell|report|say|answer)\s+(?:to\s+)?{re.escape(phrase)}\b", text):
            return {"name": person}
    return None


def _target(text: str, task_type: str, config: SchedulerConfig) -> dict[str, Any]:
    """Special function: Resolve object/category target semantics."""
    if task_type not in {"find_object", "count_objects"}:
        return {}
    for object_name in config.known_objects:
        if re.search(rf"\b{re.escape(object_name.replace('_', ' '))}s?\b", text):
            return {"object_name": object_name}
    for category in config.known_categories:
        if re.search(rf"\b{re.escape(category.replace('_', ' '))}s?\b", text):
            return {"category": category}
    return {}


def _filters(text: str, config: SchedulerConfig) -> list[dict[str, Any]]:
    """Special function: Extract object features and human pose filters."""
    filters: list[dict[str, Any]] = []
    for feature, values in config.supported_object_features.items():
        for value in values:
            if re.search(rf"\b{re.escape(value.replace('_', ' '))}\b", text):
                filters.append({"feature": feature, "operator": "equals", "value": value})
    for pose in config.supported_human_poses:
        if re.search(rf"\b{re.escape(pose.replace('_', ' '))}\b", text):
            filters.append({"feature": "pose", "operator": "equals", "value": pose})
    if re.search(r"\bnot\s+(?:a\s+|the\s+)?([a-z_ ]+)", text):
        filters.append({"feature": "negation", "operator": "not", "value": True})
    return filters


def _requires_task_location(task_type: str) -> bool:
    """Special function: Declare task families that require physical navigation first."""
    # Section: FIND_OBJECT owns global or strict-location search navigation internally.
    return task_type in {"find_person", "count_objects", "count_people"}


def _requires_report(text: str) -> bool:
    """Special function: Detect whether the command asks for a separate reporting destination."""
    return bool(re.search(r"\b(?:tell|report|say|answer)\b.*\b(?:in|at|to)\b", text) or re.search(r"\b(?:go|navigate|return)\s+to\b.*\b(?:tell|report|say|answer)\b", text))


def _output_template(text: str, task_type: str) -> str | None:
    """Special function: Select deterministic output text template for task results."""
    if not _requires_report(text) and "tell" not in text and "report" not in text and "say" not in text and "answer" not in text:
        return None
    if task_type == "count_people":
        return "I counted {result.count} people."
    if task_type == "count_objects":
        return "I counted {result.count} objects."
    if task_type == "find_person":
        return "I found the requested person."
    if task_type == "find_object":
        return "I found the requested object."
    return "{result.message}"
