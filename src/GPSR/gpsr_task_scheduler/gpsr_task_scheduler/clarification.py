"""Section: Deterministic GPSR completeness checks and bounded reply merging."""

from __future__ import annotations

from difflib import SequenceMatcher
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


# Section: Fuzzy matching is limited to object-target task families and never auto-admits a plan.
FUZZY_OBJECT_INTENTS = frozenset({
    "FIND_OBJECT",
    "COUNT_OBJECTS",
    "PICK_OBJECT",
    "PLACE_OBJECT",
    "DELIVER_OBJECT",
})


# Section: These replies explicitly accept the one suggestion already shown to the operator.
AFFIRMATIVE_SUGGESTION_REPLIES = frozenset({
    "yes",
    "yes please",
    "yeah",
    "yep",
    "correct",
    "that is correct",
    "that's correct",
    "right",
    "confirm",
    "confirmed",
})


@dataclass(frozen=True, slots=True)
class Completeness:
    """Special function: Preserve a classified command and its recognized semantic slots."""

    status: str
    missing_fields: tuple[str, ...]
    prompt: str
    intent: str
    target: str
    location: str
    suggestion: ObjectSuggestion | None = None

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


@dataclass(frozen=True, slots=True)
class ObjectSuggestion:
    """Special function: Carry one conservative fuzzy object suggestion."""

    source: str
    suggested_target: str
    score: float

    def to_dict(self) -> dict[str, Any]:
        """Special function: Serialize a suggestion for ROS JSON and diagnostic output."""
        # Section: Keep the public payload stable and round scores for readable logs.
        return {
            "input": self.source,
            "suggested_target": self.suggested_target,
            "score": round(float(self.score), 3),
        }


def _classification(missing: set[str]) -> tuple[str, str]:
    """Special function: Select the exact prompt for the missing semantic fields."""
    if not missing:
        return "COMPLETE", ""
    status = f"MISSING_{next(iter(missing)).upper()}" if len(missing) == 1 else "MULTIPLE_MISSING"
    return status, PROMPTS[status]


def _classification_with_suggestion(
    missing: set[str],
    suggestion: ObjectSuggestion | None,
) -> tuple[str, str]:
    """Special function: Add a confirmation prompt without changing completeness rules."""
    # Section: Preserve the existing status contract when there is no safe suggestion.
    status, prompt = _classification(missing)
    if suggestion is None or "target" not in missing:
        return status, prompt
    # Section: Require an explicit confirmation or replacement before planning can continue.
    suggested_label = suggestion.suggested_target.replace("_", " ")
    suggestion_prompt = (
        f'I could not match "{suggestion.source}" to a configured object. '
        f'Did you mean "{suggested_label}"? Reply "yes" to confirm, '
        "or provide another object."
    )
    if status == "MISSING_TARGET":
        return status, suggestion_prompt
    return status, f"{suggestion_prompt} {prompt}"


def _object_target_words(clause: str, config: SchedulerConfig) -> list[str]:
    """Special function: Extract likely unknown object words from one object-action clause."""
    # Section: Start after the action verb so command words cannot become suggestions.
    action = re.search(
        r"\b(?:find|locate|look\s+for|count|pick\s+up|pick|take|place|put|deliver|bring)\b",
        str(clause or "").casefold(),
    )
    if action is None:
        return []
    # Section: Stop at the first location, destination, relation, or next-clause marker.
    remainder = str(clause or "").casefold()[action.end():]
    remainder = re.split(
        r"\b(?:in|at|from|on|to|into|onto|near|beside|while|then|after|before)\b",
        remainder,
        maxsplit=1,
    )[0]
    words = re.findall(r"[a-z0-9']+", remainder)
    # Section: Remove grammatical and already-supported visual descriptors before matching.
    descriptors = {
        "a", "an", "the", "one", "two", "three", "some", "any",
        *{
            str(value).casefold().replace("_", " ")
            for values in config.supported_object_features.values()
            for value in values
        },
    }
    return [word for word in words if word not in descriptors]


def _fuzzy_object_suggestion(
    clause: str,
    intent: str,
    config: SchedulerConfig,
) -> ObjectSuggestion | None:
    """Special function: Find one high-confidence configured object or category candidate."""
    # Section: Never suggest a visual target for person, navigation, or dialogue commands.
    if intent not in FUZZY_OBJECT_INTENTS:
        return None
    words = _object_target_words(clause, config)
    if not words:
        return None
    # Section: Build displayable candidates from the authoritative object aliases and categories.
    candidates: dict[str, str] = {}
    for alias, canonical in config.object_aliases.items():
        candidates[" ".join(str(alias).replace("_", " ").split())] = (
            " ".join(str(canonical).replace("_", " ").split())
        )
    for category in config.known_categories:
        label = " ".join(str(category).replace("_", " ").split())
        candidates.setdefault(label, label)
    if not candidates:
        return None
    # Section: Compare one-to-four-word windows so multi-word RoboCup items remain supported.
    best_by_target: dict[str, tuple[float, str]] = {}
    for start in range(len(words)):
        for width in range(1, min(4, len(words) - start) + 1):
            source = " ".join(words[start:start + width])
            for alias, target in candidates.items():
                score = SequenceMatcher(None, source, alias).ratio()
                previous = best_by_target.get(target)
                if previous is None or score > previous[0]:
                    best_by_target[target] = (score, source)
    ranked = sorted(
        [
            (score, source, target)
            for target, (score, source) in best_by_target.items()
        ],
        reverse=True,
    )
    if not ranked:
        return None
    # Section: Short words need a stronger score to avoid unsafe accidental matches.
    best_score, source, target = ranked[0]
    threshold = 0.82 if len(source.replace(" ", "")) <= 4 else 0.74
    if best_score < threshold:
        return None
    # Section: Ambiguous near-ties are rejected instead of silently choosing one target.
    second_score = ranked[1][0] if len(ranked) > 1 else 0.0
    if len(ranked) > 1 and best_score - second_score < 0.08:
        return None
    return ObjectSuggestion(source, target, best_score)


def _suggestion_data(
    suggestion: ObjectSuggestion | dict[str, Any] | None,
) -> tuple[str, str] | None:
    """Special function: Read suggestion fields from internal or serialized state."""
    # Section: Clarification callbacks receive JSON dictionaries while unit paths may use dataclasses.
    if isinstance(suggestion, ObjectSuggestion):
        return suggestion.source, suggestion.suggested_target
    if isinstance(suggestion, dict):
        source = str(suggestion.get("input") or "").strip()
        target = str(suggestion.get("suggested_target") or "").strip()
        if source and target:
            return source, target
    return None


def _is_affirmative_suggestion_reply(response: str) -> bool:
    """Special function: Recognize only concise explicit confirmations."""
    # Section: Punctuation and case are ignored, but open-ended replies are not auto-accepted.
    normalized = " ".join(re.findall(r"[a-z']+", str(response or "").casefold()))
    return normalized in AFFIRMATIVE_SUGGESTION_REPLIES


def _replace_suggested_target(
    instruction: str,
    source: str,
    target: str,
) -> str:
    """Special function: Replace the original misspelling with the confirmed canonical target."""
    # Section: Replace only the first bounded occurrence so unrelated speech is preserved.
    source_phrase = " ".join(str(source).replace("_", " ").split())
    target_phrase = " ".join(str(target).replace("_", " ").split())
    if not source_phrase or not target_phrase:
        return str(instruction or "")
    pattern = rf"(?<!\w){re.escape(source_phrase)}(?!\w)"
    return re.sub(pattern, target_phrase, str(instruction or ""), count=1, flags=re.IGNORECASE)


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
    suggestion: ObjectSuggestion | None = None
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
        # Section: Offer only a high-confidence object candidate when the target slot is missing.
        if "target" in missing and suggestion is None:
            suggestion = _fuzzy_object_suggestion(clause, intent, config)
        if object_target:
            memory["object"] = object_target
        if person_target:
            memory["person"] = person_target
        if location:
            memory["location"] = location
    status, prompt = _classification_with_suggestion(missing, suggestion)
    return Completeness(
        status,
        tuple(sorted(missing)),
        prompt,
        first_intent,
        first_target,
        first_location,
        suggestion,
    )


def merge_clarification(
    original: str,
    response: str,
    config: SchedulerConfig,
    suggestion: ObjectSuggestion | dict[str, Any] | None = None,
) -> str:
    """Special function: Fill only missing slots or accept a complete restatement."""
    original = str(original or "").strip()
    response = str(response or "").strip()
    if not response:
        return original
    old = assess_command(original, config)
    new = assess_command(response, config)
    # Section: A concise affirmative confirms only the suggestion attached to this session.
    suggestion_values = _suggestion_data(suggestion or old.suggestion)
    if suggestion_values and _is_affirmative_suggestion_reply(response):
        source, target = suggestion_values
        return _replace_suggested_target(original, source, target)
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
        # Section: A recognized replacement answers the suggestion without duplicating the target.
        if suggestion_values:
            source, _suggested_target = suggestion_values
            replaced = _replace_suggested_target(merged, source, target)
            if replaced != merged:
                return replaced
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
