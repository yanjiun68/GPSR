"""Section: Deterministic GPSR completeness checks and bounded reply merging."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any

from gpsr_task_scheduler.config import SchedulerConfig
from gpsr_task_scheduler.semantic_compiler import (
    _collect_entities,
    _detect_clause_intent,
    _location_after_marker,
    canonicalize_surface_text,
    segment_clauses,
)


# Section: Voice prompts are exact, centralized GPSR dialogue contracts.
PROMPTS = {
    "MISSING_ACTION": "What would you like me to do?",
    "MISSING_TARGET": "Which object or person do you mean?",
    "MISSING_LOCATION": "Where should I go or look?",
    "MULTIPLE_MISSING": (
        "Please repeat the task and tell me what I should do, "
        "what or who it involves, and where."
    ),
}


@dataclass(frozen=True, slots=True)
class Completeness:
    """Special function: Preserve a classified command and its recognized semantic slots."""

    status: str
    missing_fields: tuple[str, ...]
    prompt: str
    intent: str
    target: str
    location: str

    @property
    def complete(self) -> bool:
        """Special function: Admit only commands with all required semantic slots."""
        return self.status == "COMPLETE"


@dataclass(slots=True)
class ClarificationSession:
    """Special function: Hold one command's partial text and correlated reply window."""

    command_id: str
    instruction: str
    attempt: int
    deadline: float
    dry_run: bool
    timeout_sec: float
    validation: dict[str, Any]


def _classification(missing: set[str]) -> tuple[str, str]:
    """Special function: Select the exact prompt for the missing semantic fields."""
    if not missing:
        return "COMPLETE", ""
    status = f"MISSING_{next(iter(missing)).upper()}" if len(missing) == 1 else "MULTIPLE_MISSING"
    return status, PROMPTS[status]


def _explicit_unknown_place(clause: str, intent: str, config: SchedulerConfig) -> bool:
    """Special function: Leave explicit unknown places to existing parser diagnostics."""
    markers = "in|at|from|on" if intent == "DELIVER_OBJECT" else "in|at|from|on|to"
    match = re.search(rf"\b(?:{markers})\s+(?:the\s+)?([a-z_]+)", clause)
    if not match:
        return False
    candidate = match.group(1)
    return candidate not in config.known_locations | config.known_people | {"me", "operator", "there", "somewhere"}


def assess_command(instruction: str, config: SchedulerConfig) -> Completeness:
    """Special function: Validate typed task requirements before queue compilation."""
    # Section: Reuse the existing canonical vocabulary and clause segmentation.
    canonical = canonicalize_surface_text(instruction, config)
    clauses = segment_clauses(canonical, config)
    if not clauses:
        return Completeness("MULTIPLE_MISSING", ("action", "target", "location"), PROMPTS["MULTIPLE_MISSING"], "", "", "")
    missing: set[str] = set()
    memory: dict[str, str] = {}
    first_intent = ""
    first_target = ""
    first_location = ""
    original = str(instruction or "")
    for clause in clauses:
        # Section: Resolve only explicitly named entities and active-clause antecedents.
        entities = _collect_entities(clause, config)
        intent = _detect_clause_intent(clause, entities)
        # Section: Preserve an explicit GO verb when a normalizer merely restores TO.
        if not intent and re.search(r"\bgo\b", clause) and entities["locations"]:
            intent = "NAVIGATE"
        # Section: An unresolved human pronoun remains a person search, not an object search.
        if intent == "FIND_OBJECT" and re.search(r"\b(?:her|him|that person)\b", clause):
            intent = "FIND_PERSON"
        if not intent and re.search(r"\bgo\s+there\b", clause):
            intent = "NAVIGATE"
        if not first_intent:
            first_intent = intent
        object_target = next(iter(entities["objects"] or entities["categories"]), "")
        person_target = next(iter(entities["people"]), "")
        location = next(iter(entities["locations"]), "")
        if not object_target and re.search(r"\b(?:it|that object)\b", clause):
            object_target = memory.get("object", "")
        if not person_target and re.search(r"\b(?:him|her|that person)\b", clause):
            person_target = memory.get("person", "")
        if not location and re.search(r"\bthere\b", clause):
            location = memory.get("location", "")
        if not first_target:
            first_target = object_target or person_target
        if not first_location:
            first_location = location
        if not intent:
            # Section: Unsupported explicit verbs retain the parser's normal rejection path.
            if re.search(r"\b(?:meet|remember|offer|describe)\b", clause):
                continue
            missing.add("action")
            if not object_target and not person_target and not location:
                missing.update(("target", "location"))
            continue
        # Section: Preserve precise parser errors for explicitly named but unknown entities.
        unknown_person_name = bool(re.search(r"\bfind\s+(?:the\s+)?[A-Z][a-z]+\b", original))
        unknown_place = not location and _explicit_unknown_place(clause, intent, config)
        if intent in {"FIND_OBJECT", "COUNT_OBJECTS", "PICK_OBJECT", "PLACE_OBJECT", "DELIVER_OBJECT"} and not object_target and not unknown_person_name:
            missing.add("target")
        if intent in {"FOLLOW_PERSON", "GUIDE_PERSON"} and not person_target:
            missing.add("target")
        if intent == "FIND_PERSON" and not (person_target or re.search(r"\b(?:someone|person|guest|man|woman)\b", clause)):
            missing.add("target")
        if intent in {"NAVIGATE", "COUNT_OBJECTS", "COUNT_PEOPLE", "GUIDE_PERSON", "PLACE_OBJECT"} and not location and not unknown_place:
            missing.add("location")
        # Section: TAKE can operate on an already held item; BRING needs a source unless a prior FIND supplied it.
        if intent == "DELIVER_OBJECT" and not re.search(r"\btake\b", clause) and not memory.get("object") and not _location_after_marker(clause, "from", config) and not unknown_place:
            missing.add("location")
        if object_target:
            memory["object"] = object_target
        if person_target:
            memory["person"] = person_target
        if location:
            memory["location"] = location
    status, prompt = _classification(missing)
    return Completeness(status, tuple(sorted(missing)), prompt, first_intent, first_target, first_location)


def merge_clarification(original: str, response: str, config: SchedulerConfig) -> str:
    """Special function: Fill only missing slots or accept a complete restatement."""
    original = str(original or "").strip()
    response = str(response or "").strip()
    if not response:
        return original
    old = assess_command(original, config)
    new = assess_command(response, config)
    # Section: A complete replacement command always supersedes the partial one.
    if new.complete:
        return response
    canonical_reply = canonicalize_surface_text(response, config)
    entities = _collect_entities(canonical_reply, config)
    # Section: A person reply cannot silently become an object target, or the reverse.
    target_group = entities["people"] if old.intent in {"FIND_PERSON", "FOLLOW_PERSON", "GUIDE_PERSON"} else entities["objects"] or entities["categories"]
    target = next(iter(target_group), "")
    location = next(iter(entities["locations"]), "")
    merged = original
    if "action" in old.missing_fields and new.intent:
        # Section: The new action may refer to the old, explicitly named target.
        merged = response
        if old.target:
            typed_target = old.target.replace("_", " ")
            reference_pattern = r"\b(?:him|her|that person)\b" if old.target in config.known_people else r"\b(?:it|that object)\b"
            merged = re.sub(reference_pattern, typed_target, merged, count=1, flags=re.IGNORECASE)
            mentioned = _collect_entities(canonicalize_surface_text(merged, config), config)
            wrong_reference = bool(re.search(r"\b(?:it|him|her|that person|that object)\b", merged, re.IGNORECASE))
            if not wrong_reference and not mentioned["objects"] and not mentioned["people"]:
                merged = f"{merged} {typed_target}"
        if old.location and "location" in assess_command(merged, config).missing_fields:
            marker = "from" if new.intent == "DELIVER_OBJECT" else "to" if new.intent in {"NAVIGATE", "PLACE_OBJECT", "GUIDE_PERSON"} else "in"
            place = f"{marker} the {old.location.replace('_', ' ')}"
            if new.intent == "DELIVER_OBJECT" and re.search(r"\bto\s+", merged, re.IGNORECASE):
                merged = re.sub(r"\bto\s+", f"{place} to ", merged.rstrip(" ."), count=1, flags=re.IGNORECASE)
            else:
                merged = f"{merged.rstrip(' .')} {place}"
        return merged
    if "target" in old.missing_fields and target:
        spoken_target = target.replace("_", " ")
        if re.search(r"\b(?:it|him|her|them|something)\b", merged, re.IGNORECASE):
            merged = re.sub(r"\b(?:it|him|her|them|something)\b", spoken_target, merged, count=1, flags=re.IGNORECASE)
        else:
            merged = re.sub(r"\b(find|deliver|pick up|place|follow|guide|count)\b", lambda match: f"{match.group(0)} {spoken_target}", merged, count=1, flags=re.IGNORECASE)
    if "location" in old.missing_fields and location:
        marker = "from" if old.intent == "DELIVER_OBJECT" else "to" if old.intent in {"NAVIGATE", "PLACE_OBJECT", "GUIDE_PERSON"} else "in"
        place = f"{marker} the {location.replace('_', ' ')}"
        if re.search(r"\bthere\b", merged, re.IGNORECASE):
            merged = re.sub(r"\b(?:to\s+|in\s+|from\s+)?there\b", place, merged, count=1, flags=re.IGNORECASE)
        else:
            merged = f"{merged.rstrip(' .')} {place}"
    return merged
