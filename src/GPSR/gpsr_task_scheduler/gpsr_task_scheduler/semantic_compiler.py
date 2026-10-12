"""Section: Deterministic language canonicalization and legacy-queue compilation."""

from __future__ import annotations

import re
from collections import defaultdict
from typing import Any

from gpsr_task_scheduler.config import SchedulerConfig, normalize_symbol
from gpsr_task_scheduler.models import (
    CommandContext,
    SchedulerResult,
    SemanticStep,
    SkillStep,
    TaskRequest,
    ValidationIssue,
)
from gpsr_task_scheduler.parser import (
    _is_fixed_information_query,
    _output_template,
    _requested_person_fields,
    parse_command,
    strip_object_mentions,
)
from gpsr_task_scheduler.planner import plan_task
from robot_yasmin_tasks.gpsr.action_types import CANONICAL_PRIMITIVES
from robot_yasmin_tasks.gpsr.greetings import DEFAULT_GREETING  # Share neutral wording across single and multi-clause commands.


# Section: A bare request to answer a question needs a spoken prompt before STT starts.
ANSWER_QUESTION_PROMPT = "What is your question?"


# Section: Common visual descriptions that Qwen can verify from a requested object ROI.
OBJECT_QUERY_SHAPE_ALIASES = {
    "cylinder": ("cylinder", "cylindrical"),
    "round": ("round", "circular"),
    "square": ("square",),
    "rectangular": ("rectangle", "rectangular"),
    "flat": ("flat",),
}


# Section: Orientation wording remains separate from human pose and robot navigation semantics.
OBJECT_QUERY_ORIENTATION_ALIASES = {
    "upright": ("upright", "standing up"),
    "horizontal": ("horizontal",),
    "vertical": ("vertical",),
    "lying_down": ("lying down",),
}


# Section: Contract for an optional upstream linguistic normalizer; it never names executors.
LLM_NORMALIZATION_CONTRACT = """Normalize grammar and remove filler words only.
Preserve every person, object, location, attribute, quantity, comparison, ordering relation,
source, destination, and recipient. Do not add missing information, invent or reorder task
steps, resolve ambiguity, or emit ROS action/executor names."""


# Section: Semantic macros map to the task-family names already owned by planner.py.
INTENT_TO_TASK = {
    "NAVIGATE": "navigate",
    "FIND_OBJECT": "find_object",
    "FIND_PERSON": "find_person",
    "COUNT_OBJECTS": "count_objects",
    "COUNT_PEOPLE": "count_people",
    "PICK_OBJECT": "acquire_object",
    "PLACE_OBJECT": "place_object",
    "DELIVER_OBJECT": "deliver_object",
    "FOLLOW_PERSON": "follow_person",
    "GUIDE_PERSON": "guide_person",
    "SAY": "say",
    "ASK_AND_LISTEN": "ask_and_listen",
    "FIXED_INFORMATION": "fixed_information",
}


# Section: Required arguments validate the final queue before scheduler admission.
REQUIRED_PRIMITIVE_ARGUMENTS = {
    "NAVIGATE": (("target_location", "target_pose"),),
    "FIND_OBJECT": (("object_name", "category"),),
    "FIND_PERSON": (("target_person",),),
    "FIND_PERSON_BY_ATTRIBUTE": (("selector",),),
    "COUNT_OBJECTS": (("object_name", "category"),),
    "COUNT_PEOPLE": tuple(),
    "EXTRACT_PERSON_INFO": (("fields",),),
    "PICK_OBJECT": (("object_name",),),
    "PLACE_OBJECT": (("object_name",), ("target_location",)),
    "HANDOVER_OBJECT": (("object_name",), ("target_person",)),
    "FOLLOW_PERSON": (("target_person",),),
    "GUIDE_PERSON": (("target_person",), ("target_location",)),
    # Section: Fixed-information SAY actions carry a provider question instead of literal TTS text.
    "SAY": (("text", "question"),),
    "ASK_AND_LISTEN": (("question",),),
    "FIXED_INFORMATION": (("question",),),
}


def compile_command(
    command_id: str,
    raw_text: str,
    config: SchedulerConfig,
    normalized_text: str | None = None,
) -> SchedulerResult:
    """Special function: Compile raw and optionally LLM-normalized text deterministically."""
    raw = str(raw_text or "").strip()
    normalized = str(normalized_text if normalized_text is not None else raw).strip()
    context = CommandContext(command_id, raw, normalized)

    # Section: Reject clear critical-entity drift before interpreting normalized language.
    drift = _semantic_drift_issues(raw, normalized, config)
    if drift:
        return _invalid_result(command_id, context, drift)

    # Section: Preserve the original question before SAY aliases can rewrite words such as "tell".
    if _is_fixed_information_query(normalized.casefold()):
        context.clauses = [{"index": 1, "text": normalized, "relationship": "then"}]
        context.entities = _collect_entities(normalized, config)
        return _compile_legacy_single(command_id, normalized, config, context)

    canonical_text = canonicalize_surface_text(normalized, config)
    clauses = segment_clauses(canonical_text, config)
    relationships = _clause_relationships(canonical_text, len(clauses))
    context.clauses = [
        {"index": index, "text": clause, "relationship": relationships[index - 1]}
        for index, clause in enumerate(clauses, start=1)
    ]
    context.entities = _collect_entities(canonical_text, config)

    # Section: Preserve the proven single-task parser/planner path whenever it is sufficient.
    # Section: Established report-back and pronoun-rejection commands retain legacy behavior.
    legacy_report = bool(re.search(r"\breport back to me\b", normalized, re.IGNORECASE))
    legacy_find_pronoun = bool(re.search(r"^\s*find\s+it\b", normalized, re.IGNORECASE))
    requires_contextual_path = bool(
        (len(clauses) > 1 and not legacy_report)
        or (
            re.search(
                r"\b(it|him|her|them|there|that object|that person)\b",
                canonical_text,
                re.IGNORECASE,
            )
            and not legacy_find_pronoun
        )
        or re.search(r"\btake\b", canonical_text, re.IGNORECASE)
        # Section: A visual object relation requires the unified semantic path rather than legacy location parsing.
        or bool(_extract_image_relation(canonical_text.casefold(), config)[0])
        # Section: The generated question prompt is not a named-person command.
        or re.search(r"\banswer question\b", normalized, re.IGNORECASE)
    )
    if not requires_contextual_path:
        return _compile_legacy_single(command_id, canonical_text, config, context)

    semantic_steps, references, semantic_issues = _parse_semantic_steps(
        clauses,
        config,
        relationships,
    )
    context.semantic_steps = semantic_steps
    context.references = references
    if semantic_issues:
        return _invalid_result(command_id, context, semantic_issues)

    result = _compile_semantic_steps(command_id, semantic_steps, config, context)
    return result


def canonicalize_surface_text(text: str, config: SchedulerConfig) -> str:
    """Special function: Apply longest-match phrase aliases without choosing executors."""
    normalized = " ".join(str(text or "").replace("-", " ").split())
    # Section: Normalize connector-adjacent gerunds before phrase matching.
    gerunds = {
        "going to": "go to",
        "finding": "find",
        "locating": "find",
        "following": "follow",
        "guiding": "guide",
        "counting": "count",
    }
    for phrase, replacement in gerunds.items():
        normalized = re.sub(
            rf"\b{re.escape(phrase)}\b",
            replacement,
            normalized,
            flags=re.IGNORECASE,
        )
    replacements = {
        "NAVIGATE": "go to",
        "FIND": "find",
        "PICK_OBJECT": "pick up",
        "PLACE_OBJECT": "place",
        "DELIVER_OBJECT": "deliver",
        "FOLLOW_PERSON": "follow",
        "GUIDE_PERSON": "guide",
        "COUNT": "count",
        "SAY": "say",
        "ASK_AND_LISTEN": "ask",
    }
    # Section: Sorting by token count and length guarantees multi-word precedence.
    aliases = sorted(
        (
            (
                alias,
                f"ask {ANSWER_QUESTION_PROMPT}" if alias == "answer question"
                else "greet" if alias == "greet"
                else replacements[intent],
            )  # Keep the question-answer prompt and neutral greeting meanings explicit.
            for intent, values in config.verb_aliases.items()
            for alias in values
            if intent in replacements
        ),
        key=lambda item: (len(item[0].split()), len(item[0])),
        reverse=True,
    )
    for alias, replacement in aliases:
        normalized = re.sub(
            rf"\b{re.escape(alias)}\b",
            replacement,
            normalized,
            flags=re.IGNORECASE,
        )

    # Section: Entity aliases become the canonical symbols expected by existing skills.
    entity_aliases = {
        **config.location_aliases,
        **config.object_aliases,
    }
    for alias, canonical in sorted(
        entity_aliases.items(), key=lambda item: len(item[0]), reverse=True
    ):
        phrase = alias.replace("_", " ")
        replacement = canonical.replace("_", " ")
        normalized = re.sub(
            rf"\b{re.escape(phrase)}\b",
            replacement,
            normalized,
            flags=re.IGNORECASE,
        )
    return " ".join(normalized.split())


def segment_clauses(text: str, config: SchedulerConfig) -> list[str]:
    """Special function: Split ordered action clauses while retaining attribute conjunctions."""
    cleaned = str(text or "").casefold().strip().strip(".")
    if not cleaned:
        return []

    # Section: An explicit leading BEFORE clause reverses its two textual branches.
    before_match = re.match(r"^before\s+(.+?),\s*(.+)$", cleaned)
    if before_match:
        later, first = before_match.groups()
        return [first.strip(" ,."), later.strip(" ,.")]
    after_match = re.match(r"^after\s+(.+?),\s*(.+)$", cleaned)
    if after_match:
        first, later = after_match.groups()
        return [first.strip(" ,."), later.strip(" ,.")]

    # Section: Infix BEFORE and AFTER are reordered into their executable chronology.
    infix_before = re.match(r"^(.+?)\s+before\s+(.+)$", cleaned)
    if infix_before:
        first, later = infix_before.groups()
        return [first.strip(" ,."), later.strip(" ,.")]
    infix_after = re.match(r"^(.+?)\s+after\s+(.+)$", cleaned)
    if infix_after:
        later, first = infix_after.groups()
        return [first.strip(" ,."), later.strip(" ,.")]

    action_starts = (
        "go to|find|meet|pick up|place|deliver|follow|guide|count|say|greet|ask|take"  # Keep person search and greeting verbs as their own ordered actions.
    )
    splitter = re.compile(
        rf"\s*(?:,\s*|\b(?:and then|then|after that|finally)\b\s*|"
        rf"\band\b\s*(?=(?:{action_starts})\b)|\bwhile\b\s*)",
    )
    clauses = [part.strip(" ,.") for part in splitter.split(cleaned) if part.strip(" ,.")]
    return clauses or [cleaned]


def _clause_relationships(text: str, clause_count: int) -> list[str]:
    """Special function: Retain explicit connector metadata beside chronological order."""
    relationships = ["then"] * clause_count
    lowered = str(text).casefold()
    if clause_count > 1 and re.search(r"\bwhile\b", lowered):
        relationships[1] = "while"
    elif clause_count > 1 and re.search(r"\b(?:before|after)\b", lowered):
        relationships[1] = "after"
    if clause_count > 1 and re.search(r"\bfinally\b", lowered):
        relationships[-1] = "finally"
    return relationships


def _compile_legacy_single(
    command_id: str,
    canonical_text: str,
    config: SchedulerConfig,
    context: CommandContext,
) -> SchedulerResult:
    """Special function: Retain established single-task behavior behind the new context."""
    request, issues = parse_command(command_id, canonical_text, config)
    if issues or request is None:
        return _invalid_result(command_id, context, issues, request)
    semantic = _semantic_from_request(request, canonical_text, config, 1)
    # Section: New attributes enrich the old request without changing its established fields.
    for key, value in semantic.attributes.items():
        feature = "object_color" if key == "color" else key
        candidate = {"feature": feature, "operator": "equals", "value": value}
        if candidate not in request.filters:
            request.filters.append(candidate)
    context.semantic_steps = [semantic]
    result = plan_task(request, config)
    result.command_context = context
    return _finalize_result(result, context, config)


def _parse_semantic_steps(
    clauses: list[str],
    config: SchedulerConfig,
    relationships: list[str],
) -> tuple[list[SemanticStep], dict[str, Any], list[ValidationIssue]]:
    """Special function: Extract ordered intents, slots, and safe antecedents."""
    steps: list[SemanticStep] = []
    issues: list[ValidationIssue] = []
    memory: dict[str, str] = {"object": "", "person": "", "location": ""}
    resolutions: dict[str, Any] = {}
    for index, original_clause in enumerate(clauses, start=1):
        clause, clause_resolutions, reference_issues = _resolve_references(
            original_clause,
            memory,
        )
        resolutions[f"clause_{index}"] = clause_resolutions
        issues.extend(reference_issues)
        if reference_issues:
            continue
        step, step_issues = _semantic_from_clause(clause, config, index)
        issues.extend(step_issues)
        if step is None:
            continue
        step.relationship = relationships[index - 1]
        # Section: An explicit preceding navigation safely constrains the next local task.
        if (
            step.intent in {"FIND_OBJECT", "FIND_PERSON", "COUNT_OBJECTS", "COUNT_PEOPLE"}
            and not step.arguments.get("target_location")
            and memory.get("location")
        ):
            step.arguments["target_location"] = memory["location"]
            resolutions[f"clause_{index}"]["implicit_location"] = memory["location"]
        steps.append(step)
        _update_reference_memory(memory, step)
    return steps, resolutions, issues


def _resolve_references(
    clause: str,
    memory: dict[str, str],
) -> tuple[str, dict[str, str], list[ValidationIssue]]:
    """Special function: Resolve pronouns to the nearest type-compatible antecedent."""
    resolved = clause
    mapping: dict[str, str] = {}
    issues: list[ValidationIssue] = []
    rules = [
        (r"\b(?:it|that object)\b", "object"),
        (r"\b(?:him|her|that person)\b", "person"),
        (r"\bthere\b", "location"),
    ]
    for pattern, kind in rules:
        if not re.search(pattern, resolved):
            continue
        antecedent = memory.get(kind, "")
        if not antecedent:
            issues.append(ValidationIssue(
                "UNRESOLVED_REFERENCE",
                f"No prior {kind} safely resolves the reference in: {clause}",
            ))
            continue
        token = re.search(pattern, resolved).group(0)
        resolved = re.sub(pattern, antecedent.replace("_", " "), resolved)
        mapping[token] = antecedent

    # Section: Plural THEM is rejected unless exactly one prior typed entity is available.
    if re.search(r"\bthem\b", resolved):
        candidates = [value for key, value in memory.items() if key != "location" and value]
        if len(set(candidates)) != 1:
            issues.append(ValidationIssue(
                "UNRESOLVED_REFERENCE",
                f"Reference 'them' is ambiguous in: {clause}",
            ))
        else:
            resolved = re.sub(r"\bthem\b", candidates[0].replace("_", " "), resolved)
            mapping["them"] = candidates[0]
    # Section: ME is an explicit operator alias, not an inferred unknown person.
    if re.search(r"\bme\b", resolved):
        resolved = re.sub(r"\bme\b", "operator", resolved)
        mapping["me"] = "operator"
    return resolved, mapping, issues


def _semantic_from_clause(
    clause: str,
    config: SchedulerConfig,
    index: int,
) -> tuple[SemanticStep | None, list[ValidationIssue]]:
    """Special function: Resolve one canonical clause into a typed semantic intent."""
    clause = clause.casefold()
    entities = _collect_entities(clause, config)
    attributes = _extract_attributes(clause, config)
    intent = _detect_clause_intent(clause, entities)
    if not intent:
        return None, [ValidationIssue("UNKNOWN_VERB", f"No supported verb in clause: {clause}")]

    objects = entities["objects"]
    categories = entities["categories"]
    people = entities["people"]
    locations = entities["locations"]
    arguments: dict[str, Any] = {}
    if objects:
        arguments["object_name"] = objects[0]
    elif categories:
        arguments["category"] = categories[0]
    if people:
        arguments["target_person"] = people[-1]
    if locations:
        arguments["target_location"] = locations[-1]

    # Section: Source and destination slots remain distinct for delivery semantics.
    if intent == "DELIVER_OBJECT":
        source = _location_after_marker(clause, "from", config)
        if source:
            arguments["source_location"] = source
        if not arguments.get("target_person"):
            return None, [ValidationIssue(
                "MISSING_REQUIRED_ARGUMENT",
                f"DELIVER_OBJECT requires a known recipient: {clause}",
            )]
    if intent == "NAVIGATE" and not arguments.get("target_location"):
        return None, [ValidationIssue("UNKNOWN_LOCATION", f"Unknown navigation destination: {clause}")]
    if intent in {"FIND_OBJECT", "PICK_OBJECT", "PLACE_OBJECT"} and not (
        arguments.get("object_name") or arguments.get("category")
    ):
        return None, [ValidationIssue("UNKNOWN_OBJECT", f"Unknown object target: {clause}")]
    if intent in {"FIND_PERSON", "FOLLOW_PERSON", "GUIDE_PERSON"} and not arguments.get("target_person"):
        if re.search(r"\b(?:person|someone|guest|man|woman)\b", clause) and intent == "FIND_PERSON":
            arguments["person_selector"] = {"type": "any_person", "value": ""}
        else:
            return None, [ValidationIssue("UNKNOWN_PERSON", f"Unknown person target: {clause}")]
    # Section: Preserve requested world fields so the planner inserts extraction and reporting.
    if intent == "FIND_PERSON":
        requested_fields = _requested_person_fields(clause, "find_person")
        if requested_fields:
            arguments["requested_person_fields"] = requested_fields
            arguments["output_template"] = _output_template(
                clause,
                "find_person",
                requested_fields,
            )
    if intent == "COUNT_OBJECTS" and not (
        arguments.get("object_name") or arguments.get("category")
    ):
        return None, [ValidationIssue("UNKNOWN_OBJECT", f"Unknown count target: {clause}")]
    if intent == "SAY":
        arguments["text"] = DEFAULT_GREETING if re.search(r"\bgreet\b", clause) else re.sub(r"^say\s+", "", clause).strip()  # Greeting commands supply their own text.
    if intent == "ASK_AND_LISTEN":
        arguments["question"] = re.sub(r"^ask\s+", "", clause).strip()
    if intent == "FIXED_INFORMATION":
        # Section: Strip the resolved recipient while retaining it as a delivery target.
        target_person = str(arguments.get("target_person", ""))
        fact_request = clause
        if target_person:
            person_phrase = target_person.replace("_", " ")
            fact_request = re.sub(
                rf"^\s*(?:tell|say|report)\s+(?:to\s+)?{re.escape(person_phrase)}\s+",
                "",
                fact_request,
                count=1,
            ).strip()
        arguments["question"] = fact_request or clause

    return SemanticStep(
        step_id=f"semantic_{index}",
        intent=intent,
        clause=clause,
        arguments=arguments,
        attributes=attributes,
    ), []


def _detect_clause_intent(clause: str, entities: dict[str, list[str]]) -> str:
    """Special function: Resolve canonical and ambiguous verbs from entity context."""
    # Section: Provider-owned fact requests take precedence over literal SAY classification.
    if _is_fixed_information_query(clause):
        return "FIXED_INFORMATION"
    # Section: A dialogue request for perceived person data is a search mission, not literal speech.
    if _requested_person_fields(clause, "find_person") and re.search(
        r"\b(?:person|guest|someone|man|woman)\b", clause
    ):
        return "FIND_PERSON"
    # Section: MEET uses the same identity or generic-person search routes as FIND_PERSON.
    if re.search(r"^meet\b", clause):
        return "FIND_PERSON"
    # Section: TAKE changes meaning based on target type and explicit destination.
    if re.search(r"\btake\b", clause):
        has_destination = bool(re.search(r"\bto\b", clause))
        if entities["people"] and entities["locations"]:
            return "GUIDE_PERSON"
        if entities["objects"] or entities["categories"]:
            return "DELIVER_OBJECT" if has_destination else "PICK_OBJECT"
        return ""
    if re.search(r"\bcount\b", clause):
        return "COUNT_PEOPLE" if re.search(r"\b(?:people|persons|humans)\b", clause) else "COUNT_OBJECTS"
    if re.search(r"\bfind\b", clause):
        return "FIND_PERSON" if entities["people"] or re.search(r"\b(?:person|someone|guest|man|woman)\b", clause) else "FIND_OBJECT"
    ordered = [
        ("NAVIGATE", r"\bgo to\b"),
        ("PICK_OBJECT", r"\bpick up\b"),
        ("PLACE_OBJECT", r"\bplace\b"),
        ("DELIVER_OBJECT", r"\bdeliver\b"),
        ("FOLLOW_PERSON", r"\bfollow\b"),
        ("GUIDE_PERSON", r"\bguide\b"),
        ("ASK_AND_LISTEN", r"\bask\b"),
        ("SAY", r"\b(?:say|greet)\b"),  # GREET remains an executable SAY primitive with neutral wording.
    ]
    for intent, pattern in ordered:
        if re.search(pattern, clause):
            return intent
    return ""


def _compile_semantic_steps(
    command_id: str,
    semantic_steps: list[SemanticStep],
    config: SchedulerConfig,
    context: CommandContext,
) -> SchedulerResult:
    """Special function: Expand semantic macros through the existing planner."""
    compiled: list[SkillStep] = []
    issues: list[ValidationIssue] = []
    first_request: TaskRequest | None = None
    # Section: Producer maps preserve only genuine reusable state dependencies after
    # duplicate macro steps are pruned; array order alone preserves clause sequencing.
    found_object_producer: dict[str, str] = {}
    picked_object_producer: dict[str, str] = {}
    found_person_producer: dict[str, str] = {}

    for sequence, semantic in enumerate(semantic_steps, start=1):
        request = _request_from_semantic(command_id, semantic, config)
        first_request = first_request or request
        planned = plan_task(request, config)
        issues.extend(planned.issues)
        local_steps = list(planned.steps)
        object_name = str(semantic.arguments.get("object_name", ""))
        target_person = str(semantic.arguments.get("target_person", ""))
        dependency_replacements: dict[str, str] = {}
        clause_prerequisites: list[str] = []

        # Section: Deliver fixed information to the person already found without repeating perception.
        if semantic.intent == "FIXED_INFORMATION" and target_person in found_person_producer:
            local_steps = [step for step in local_steps if step.skill_name != "FIND_PERSON"]
            dependency_replacements["find_recipient"] = found_person_producer[target_person]

        # Section: Reuse an explicitly produced object state instead of repeating macro work.
        if semantic.intent == "PICK_OBJECT" and object_name in found_object_producer:
            local_steps = [step for step in local_steps if step.skill_name != "FIND_OBJECT"]
            dependency_replacements["find_object"] = found_object_producer[object_name]
        if semantic.intent == "DELIVER_OBJECT":
            if object_name in picked_object_producer:
                local_steps = [
                    step for step in local_steps
                    if step.skill_name not in {"FIND_OBJECT", "PICK_OBJECT"}
                ]
                dependency_replacements["pick_object"] = picked_object_producer[object_name]
            elif object_name in found_object_producer:
                local_steps = [step for step in local_steps if step.skill_name != "FIND_OBJECT"]
                dependency_replacements["find_object"] = found_object_producer[object_name]

        # Section: Follow/guide actions that reuse an earlier named-person observation
        # retain that semantic prerequisite without chaining unrelated clauses.
        if (
            semantic.intent in {"FOLLOW_PERSON", "GUIDE_PERSON"}
            and target_person in found_person_producer
        ):
            clause_prerequisites.append(found_person_producer[target_person])

        renamed = _chain_and_rename_steps(
            local_steps,
            sequence,
            dependency_replacements,
            clause_prerequisites,
        )
        compiled.extend(renamed)
        find_object_step = next(
            (step for step in renamed if step.skill_name == "FIND_OBJECT"),
            None,
        )
        pick_object_step = next(
            (step for step in renamed if step.skill_name == "PICK_OBJECT"),
            None,
        )
        find_person_step = next(
            (
                step
                for step in renamed
                if step.skill_name in {"FIND_PERSON", "FIND_PERSON_BY_ATTRIBUTE"}
            ),
            None,
        )
        if semantic.intent == "FIND_OBJECT" and object_name:
            if find_object_step is not None:
                find_object_step.result_variable = "found_object"
                found_object_producer[object_name] = find_object_step.step_id
        if semantic.intent == "PICK_OBJECT" and object_name:
            if find_object_step is not None:
                found_object_producer[object_name] = find_object_step.step_id
            if pick_object_step is not None:
                picked_object_producer[object_name] = pick_object_step.step_id
        if semantic.intent == "DELIVER_OBJECT" and object_name:
            if find_object_step is not None:
                found_object_producer[object_name] = find_object_step.step_id
            picked_object_producer.pop(object_name, None)
        if semantic.intent == "FIND_PERSON" and target_person and find_person_step is not None:
            found_person_producer[target_person] = find_person_step.step_id

    # Section: Preserve the existing one-return mission policy for a multi-step command.
    if config.return_after_completion and config.instruction_point and compiled:
        return_step = SkillStep(
            "return_to_instruction_point",
            "NAVIGATE",
            {"target_location": config.instruction_point},
            [],
            config.timeout_for("NAVIGATE"),
            config.retries_for("NAVIGATE"),
        )
        compiled.append(return_step)

    result = SchedulerResult(command_id, first_request, compiled, _dedupe_issues(issues), command_context=context)
    return _finalize_result(result, context, config)


def _chain_and_rename_steps(
    steps: list[SkillStep],
    sequence: int,
    dependency_replacements: dict[str, str] | None = None,
    clause_prerequisites: list[str] | None = None,
) -> list[SkillStep]:
    """Special function: Rename local IDs while preserving only semantic prerequisites."""
    # Section: Removed macro producers resolve to explicit earlier producers rather
    # than to the preceding clause's final step.
    replacements = dict(dependency_replacements or {})
    initial_dependencies = list(clause_prerequisites or [])
    id_map = {step.step_id: f"semantic_{sequence}_{step.step_id}" for step in steps}
    renamed: list[SkillStep] = []
    for index, step in enumerate(steps):
        unresolved_dependencies = [
            item
            for item in step.dependencies
            if item not in id_map and item not in replacements
        ]
        if unresolved_dependencies:
            raise ValueError(
                f"No retained producer exists for dependencies {unresolved_dependencies!r} "
                f"on semantic clause {sequence}."
            )
        dependencies = [
            id_map[item] if item in id_map else replacements[item]
            for item in step.dependencies
            if item in id_map or item in replacements
        ]
        if index == 0:
            dependencies.extend(
                dependency
                for dependency in initial_dependencies
                if dependency not in dependencies
            )
        renamed.append(SkillStep(
            step_id=id_map[step.step_id],
            skill_name=step.skill_name,
            arguments=dict(step.arguments),
            dependencies=dependencies,
            timeout=step.timeout,
            retry_count=step.retry_count,
            result_variable=step.result_variable,
        ))
    return renamed


def _request_from_semantic(
    command_id: str,
    semantic: SemanticStep,
    config: SchedulerConfig,
) -> TaskRequest:
    """Special function: Adapt semantic slots to the existing planner TaskRequest."""
    intent = semantic.intent
    arguments = semantic.arguments
    target: dict[str, Any] = {}
    for key in ("object_name", "category", "target_person", "text", "question"):
        if key in arguments:
            target[key] = arguments[key]
    filters = [
        {
            "feature": "object_color" if key == "color" else key,
            "operator": "equals",
            "value": value,
        }
        for key, value in semantic.attributes.items()
    ]
    selector: dict[str, Any] = {}
    if intent == "FIND_PERSON":
        selector = arguments.get("person_selector") or {
            "type": "name",
            "value": arguments.get("target_person", ""),
        }
    task_location = arguments.get("target_location")
    if intent == "DELIVER_OBJECT":
        task_location = arguments.get("source_location")
    return TaskRequest(
        command_id=command_id,
        task_type=INTENT_TO_TASK[intent],
        task_location=task_location,
        report_location=None,
        return_location=config.instruction_point,
        target=target,
        filters=filters,
        person_selector=selector,
        requested_person_fields=list(arguments.get("requested_person_fields", [])),
        output_template=arguments.get("output_template"),
        return_after_completion=False,
    )


def _semantic_from_request(
    request: TaskRequest,
    clause: str,
    config: SchedulerConfig,
    index: int,
) -> SemanticStep:
    """Special function: Expose legacy TaskRequest semantics through the new representation."""
    inverse = {task: intent for intent, task in INTENT_TO_TASK.items()}
    arguments = dict(request.target)
    if request.task_location:
        location_key = (
            "source_location"
            if request.task_type == "deliver_object"
            else "target_location"
        )
        arguments[location_key] = request.task_location
    if request.person_selector:
        arguments["person_selector"] = dict(request.person_selector)
    return SemanticStep(
        f"semantic_{index}",
        inverse.get(request.task_type, request.task_type.upper()),
        clause,
        arguments,
        _extract_attributes(clause, config),
    )


def _finalize_result(
    result: SchedulerResult,
    context: CommandContext,
    config: SchedulerConfig,
) -> SchedulerResult:
    """Special function: Validate queue primitives and complete observable context."""
    queue_issues = _validate_queue(result.steps, config)
    result.issues = _dedupe_issues([*result.issues, *queue_issues])
    context.action_queue = [
        {
            "step_id": step.step_id,
            "action_type": step.skill_name,
            "arguments": dict(step.arguments),
            "dependencies": list(step.dependencies),
        }
        for step in result.steps
    ]
    context.validation = {
        "valid": not result.issues,
        "errors": [
            {"code": issue.code, "message": issue.message}
            for issue in result.issues
        ],
        "warnings": [],
    }
    result.command_context = context
    return result


def _validate_queue(
    steps: list[SkillStep],
    config: SchedulerConfig,
) -> list[ValidationIssue]:
    """Special function: Reject unknown actions and missing executor arguments."""
    issues: list[ValidationIssue] = []
    seen_step_ids: set[str] = set()
    for step in steps:
        if not step.step_id or step.step_id in seen_step_ids:
            issues.append(ValidationIssue(
                "INVALID_ACTION_ORDER",
                f"Queue contains an empty or duplicate step id: {step.step_id!r}.",
            ))
        missing_dependencies = [
            dependency
            for dependency in step.dependencies
            if dependency not in seen_step_ids
        ]
        if missing_dependencies:
            issues.append(ValidationIssue(
                "INVALID_ACTION_ORDER",
                f"Step {step.step_id} depends on unavailable prior steps {missing_dependencies}.",
            ))
        if step.skill_name not in CANONICAL_PRIMITIVES:
            issues.append(ValidationIssue(
                "UNSUPPORTED_ACTION",
                f"Queue step {step.step_id} uses unsupported action {step.skill_name}.",
            ))
            continue
        if step.skill_name not in config.primitive_capabilities:
            issues.append(ValidationIssue(
                "EXECUTOR_NOT_FOUND",
                f"No capability registry entry exists for {step.skill_name}.",
            ))
        for alternatives in REQUIRED_PRIMITIVE_ARGUMENTS.get(step.skill_name, tuple()):
            if not any(_argument_value(step.arguments, name) for name in alternatives):
                issues.append(ValidationIssue(
                    "MISSING_REQUIRED_ARGUMENT",
                    f"{step.skill_name} step {step.step_id} requires one of {alternatives}.",
                ))
        seen_step_ids.add(step.step_id)
    return issues


def _argument_value(arguments: dict[str, Any], name: str) -> Any:
    """Special function: Read direct and compatibility-nested primitive arguments."""
    return arguments.get(name)


def _semantic_drift_issues(
    raw_text: str,
    normalized_text: str,
    config: SchedulerConfig,
) -> list[ValidationIssue]:
    """Special function: Detect changed critical entities in optional LLM output."""
    if raw_text == normalized_text:
        return []
    raw_signature = _critical_signature(raw_text, config)
    normalized_signature = _critical_signature(normalized_text, config)
    if raw_signature == normalized_signature:
        return []
    return [ValidationIssue(
        "LLM_SEMANTIC_DRIFT",
        f"Critical command fields changed: raw={raw_signature}, normalized={normalized_signature}",
    )]


def _critical_signature(text: str, config: SchedulerConfig) -> dict[str, Any]:
    """Special function: Extract lightweight entity, attribute, number, and order evidence."""
    canonical = canonicalize_surface_text(text, config)
    entities = _collect_entities(canonical, config)
    return {
        "objects": entities["objects"],
        "categories": entities["categories"],
        "people": entities["people"],
        "locations": entities["locations"],
        "numbers": re.findall(r"\b\d+(?:\.\d+)?\b", canonical),
        "attributes": _extract_attributes(canonical, config),
        "entity_order": _entity_order(canonical, config),
    }


def _collect_entities(text: str, config: SchedulerConfig) -> dict[str, list[str]]:
    """Special function: Reuse authoritative registries and preserve mention order."""
    canonical = str(text).casefold()
    groups: dict[str, list[tuple[int, str]]] = defaultdict(list)
    vocabularies = {
        "objects": {alias: config.object_aliases.get(alias, alias) for alias in config.known_objects},
        "categories": {item: item for item in config.known_categories},
        "people": {item: item for item in config.known_people},
        "locations": {item: item for item in config.known_locations},
    }
    for group, vocabulary in vocabularies.items():
        for raw_name, canonical_name in vocabulary.items():
            phrase = raw_name.replace("_", " ")
            match = re.search(rf"\b{re.escape(phrase)}s?\b", canonical)
            if match:
                groups[group].append((match.start(), canonical_name))
    return {
        group: _ordered_unique(value for _, value in sorted(groups.get(group, [])))
        for group in ("objects", "categories", "people", "locations")
    }


def _entity_order(text: str, config: SchedulerConfig) -> list[str]:
    """Special function: Retain cross-type entity ordering for drift checks."""
    hits: list[tuple[int, str]] = []
    entities = _collect_entities(text, config)
    for values in entities.values():
        for value in values:
            match = re.search(rf"\b{re.escape(value.replace('_', ' '))}\b", text)
            if match:
                hits.append((match.start(), value))
    return [value for _, value in sorted(hits)]


def _extract_attributes(text: str, config: SchedulerConfig) -> dict[str, Any]:
    """Special function: Preserve object characteristics and image-space selection relations."""
    attributes: dict[str, Any] = {}
    # Section: Do not interpret color words inside exact object names as filters.
    feature_text = strip_object_mentions(text, config)
    colors = sorted({
        value
        for values in config.supported_object_features.values()
        for value in values
    })
    for color in colors:
        if re.search(rf"\b{re.escape(color.replace('_', ' '))}\b", feature_text):
            attributes["color"] = color
            break
    # Section: Shapes normalize inflected words so Qwen receives its schema's stable values.
    for shape, aliases in OBJECT_QUERY_SHAPE_ALIASES.items():
        if any(re.search(rf"\b{re.escape(alias)}\b", feature_text) for alias in aliases):
            attributes["shape"] = shape
            break
    # Section: Object orientation is a visual constraint, not a human-pose selector.
    for orientation, aliases in OBJECT_QUERY_ORIENTATION_ALIASES.items():
        if any(re.search(rf"\b{re.escape(alias)}\b", feature_text) for alias in aliases):
            attributes["orientation"] = orientation
            break
    # Section: Superlatives select one candidate inside the unified FIND_OBJECT skill.
    for relation in ("largest", "smallest", "biggest"):
        if re.search(rf"\b{relation}\b", text):
            attributes["size_relation"] = {
                "biggest": "largest",
            }.get(relation, relation)
            break
    # Section: Spatial wording is retained as a target/reference relation, never a new skill name.
    spatial_relation, reference_object = _extract_image_relation(text, config)
    if spatial_relation and reference_object:
        attributes["spatial_relation"] = spatial_relation
        attributes["reference_object"] = reference_object
    quantity = re.search(r"\b(\d+)\b", text)
    if quantity:
        attributes["quantity"] = int(quantity.group(1))
    return attributes


def _extract_image_relation(
    text: str,
    config: SchedulerConfig,
) -> tuple[str, str]:
    """Special function: Resolve a supported image-space relation and its configured reference entity."""
    # Section: Relations remain limited to view-verifiable geometry; metric distance stays a depth task.
    patterns = (
        ("left_of", r"\b(?:on\s+|to\s+)?(?:the\s+)?left\s+of\s+(?:the\s+)?(?P<reference>.+)$"),
        ("right_of", r"\b(?:on\s+|to\s+)?(?:the\s+)?right\s+of\s+(?:the\s+)?(?P<reference>.+)$"),
        ("above", r"\b(?:above|over)\s+(?:the\s+)?(?P<reference>.+)$"),
        ("below", r"\b(?:below|under)\s+(?:the\s+)?(?P<reference>.+)$"),
        ("near", r"\b(?:near|next\s+to|beside)\s+(?:the\s+)?(?P<reference>.+)$"),
        ("far_from", r"\b(?:far\s+from|away\s+from)\s+(?:the\s+)?(?P<reference>.+)$"),
    )
    for relation, pattern in patterns:
        match = re.search(pattern, text)
        if not match:
            continue
        # Section: Reuse only configured object/category names as reference targets.
        entities = _collect_entities(match.group("reference"), config)
        reference_candidates = [*entities["objects"], *entities["categories"]]
        if reference_candidates:
            return relation, reference_candidates[0]
    return "", ""


def _location_after_marker(
    clause: str,
    marker: str,
    config: SchedulerConfig,
) -> str:
    """Special function: Resolve a canonical location following a source marker."""
    match = re.search(rf"\b{re.escape(marker)}\b(.+?)(?:\bto\b|$)", clause)
    if not match:
        return ""
    return next(iter(_collect_entities(match.group(1), config)["locations"]), "")


def _update_reference_memory(memory: dict[str, str], step: SemanticStep) -> None:
    """Special function: Record only explicit typed antecedents for later pronouns."""
    if step.arguments.get("object_name"):
        memory["object"] = str(step.arguments["object_name"])
    if step.arguments.get("target_person"):
        memory["person"] = str(step.arguments["target_person"])
    if step.arguments.get("target_location"):
        memory["location"] = str(step.arguments["target_location"])


def _ordered_unique(values) -> list[str]:
    """Special function: Deduplicate values without losing first-mention order."""
    result: list[str] = []
    for value in values:
        if value not in result:
            result.append(value)
    return result


def _dedupe_issues(issues: list[ValidationIssue]) -> list[ValidationIssue]:
    """Special function: Deduplicate structured errors while retaining stable order."""
    result: list[ValidationIssue] = []
    seen: set[tuple[str, str]] = set()
    for issue in issues:
        key = (issue.code, issue.message)
        if key not in seen:
            seen.add(key)
            result.append(issue)
    return result


def _invalid_result(
    command_id: str,
    context: CommandContext,
    issues: list[ValidationIssue],
    request: TaskRequest | None = None,
) -> SchedulerResult:
    """Special function: Return a blocked plan with inspectable structured errors."""
    context.validation = {
        "valid": False,
        "errors": [
            {"code": issue.code, "message": issue.message}
            for issue in issues
        ],
        "warnings": [],
    }
    return SchedulerResult(
        command_id,
        request,
        [],
        issues,
        command_context=context,
    )
