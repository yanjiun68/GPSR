"""Section: Deterministic GPSR command parser."""

from __future__ import annotations

import re
from typing import Any

from gpsr_task_scheduler.config import SchedulerConfig, normalize_symbol
from gpsr_task_scheduler.models import TaskRequest, ValidationIssue
from robot_yasmin_tasks.gpsr.greetings import DEFAULT_GREETING  # Use the same trait-independent greeting as direct skill execution.


def parse_command(command_id: str, instruction: str, config: SchedulerConfig) -> tuple[TaskRequest | None, list[ValidationIssue]]:
    """Special function: Parse constrained GPSR text into semantic task roles."""
    original_instruction = str(instruction or "")
    text = " ".join(original_instruction.lower().replace("-", " ").split())
    # Section: Normalize supported STT variants before deterministic vocabulary matching.
    text = _normalize_person_attribute_phrases(text)
    issues: list[ValidationIssue] = []
    if not text:
        return None, [ValidationIssue("empty_instruction", "Instruction is empty.")]
    # Section: The idiom "what time is it" contains no unresolved entity reference.
    if not _is_fixed_information_query(text) and re.search(r"\b(it|there|them|him|her)\b", text) and not _contains_known_target(text, config):
        issues.append(ValidationIssue("unresolved_pronoun", "Instruction contains a pronoun without a resolvable target."))
    task_type = _detect_task_type(text, config)
    if task_type == "accompany_ambiguous":
        issues.append(ValidationIssue("ambiguous_leader", "Accompany requires an explicit human-leads or robot-leads role."))
        task_type = ""
    unsupported = re.search(r"\b(answer|describe|remember|offer)\b", text)
    if not task_type and unsupported:
        issues.append(ValidationIssue("unsupported_task", f"Unsupported task verb: {unsupported.group(1)}."))
    if not task_type:
        if not unsupported and not any(issue.code == "ambiguous_leader" for issue in issues):
            issues.append(ValidationIssue("unknown_task", "No supported deterministic GPSR task was recognized."))
    known_locations = _locations_in_text(text, config.known_locations)
    unknown_locations = _unknown_location_mentions(text, config)
    for location in unknown_locations:
        issue_code = "unknown_location_or_surface" if re.search(rf"\bon\s+(?:the\s+)?{re.escape(location.replace('_', ' '))}\b", text) else "unknown_location"
        issues.append(ValidationIssue(issue_code, f"Unknown location or surface: {location}."))
    report_location = _report_location(text, known_locations)
    task_location = _task_location(text, known_locations, report_location)
    recipient = _recipient(text, config)
    target = _target(text, task_type, config)
    if task_type == "compare_objects":
        target["property"] = _comparison_property(text)
    filters = _filters(text, config, task_type)
    person_selector, person_issues = _person_selector(text, task_type, filters, config)
    requested_person_fields = _requested_person_fields(text, task_type)
    issues.extend(person_issues)
    action_person, action_person_issues = _action_person(text, task_type, config)
    issues.extend(action_person_issues)
    if action_person:
        target["target_person"] = action_person
    speech_payload, speech_issues = _speech_payload(text, task_type, action_person)
    issues.extend(speech_issues)
    target.update(speech_payload)
    if task_type and _requires_task_location(task_type) and not task_location:
        issues.append(ValidationIssue("missing_task_location", "Task requires a known task location."))
    if _requires_report(text) and not report_location and len(known_locations) > 1:
        issues.append(ValidationIssue("missing_report_location", "Instruction requests output at a destination, but no known report location was found."))
    if task_type in {"find_object", "count_objects"} and not target:
        unknown_named_person = re.search(r"\b(?:find|locate|meet)\s+(?:the\s+)?([A-Z][a-z]+)\b", original_instruction)
        if task_type == "find_object" and unknown_named_person:
            issues.append(ValidationIssue("unknown_person", f"Unknown person: {normalize_symbol(unknown_named_person.group(1))}."))
        else:
            issues.append(ValidationIssue("missing_target", "Object task requires a known object or category target."))
    if task_type == "find_person" and not person_selector and not person_issues:
        issues.append(ValidationIssue("missing_person_selector", "Find-person task requires exactly one name or attribute selector."))
    if task_type == "navigate" and not task_location:
        issues.append(ValidationIssue("missing_task_location", "Navigation requires a known destination."))
    if task_type == "place_object" and not task_location:
        issues.append(ValidationIssue("missing_task_location", "Place requires a known destination or surface."))
    if task_type in {"acquire_object", "place_object", "deliver_object"} and not target.get("object_name"):
        issues.append(ValidationIssue("missing_target", "Object manipulation requires a known object target."))
    if task_type == "compare_objects":
        if not target.get("property"):
            issues.append(ValidationIssue("missing_comparison_property", "Comparison requires a supported property."))
        if len(target.get("object_names", [])) < 2 and not target.get("category"):
            issues.append(ValidationIssue("missing_target", "Object comparison requires at least two known objects or a category."))
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
        person_selector=person_selector,
        requested_person_fields=requested_person_fields,
        recipient=recipient,
        output_template=_output_template(text, task_type, requested_person_fields),
        return_after_completion=config.return_after_completion,
        report_after_return=_report_after_return(text),
    ), []


def _normalize_person_attribute_phrases(text: str) -> str:
    """Special function: Convert supported arm-language variants to canonical parser phrases."""
    # Section: Longer variants are replaced first to preserve exact word boundaries.
    aliases = {
        "raising the right arm": "right hand raised",
        "raising right arm": "right hand raised",
        "right arm raised": "right hand raised",
        "raising the right hand": "right hand raised",
        "raising right hand": "right hand raised",
        "raising the left arm": "left hand raised",
        "raising left arm": "left hand raised",
        "left arm raised": "left hand raised",
        "raising the left hand": "left hand raised",
        "raising left hand": "left hand raised",
    }
    normalized = text
    for phrase, replacement in aliases.items():
        normalized = re.sub(rf"\b{re.escape(phrase)}\b", replacement, normalized)
    return " ".join(normalized.split())


def _person_selector(
    text: str,
    task_type: str,
    filters: list[dict[str, Any]],
    config: SchedulerConfig,
) -> tuple[dict[str, Any], list[ValidationIssue]]:
    """Special function: Resolve one mutually exclusive name or attribute person selector."""
    if task_type != "find_person":
        return {}, []

    # Section: A searched name must occur after the find verb and before any reporting clause.
    searched_names = []
    search_clause = re.split(r"\b(?:tell|report|say|answer)\b", text, maxsplit=1)[0]
    for person in sorted(config.known_people):
        phrase = person.replace("_", " ")
        if re.search(rf"\b(?:find|locate|look for|meet)\b.*\b{re.escape(phrase)}\b", search_clause):
            searched_names.append(person)

    # Section: Pose filters are authoritative for attribute-routed person searches.
    pose_values = [
        str(item.get("value", ""))
        for item in filters
        if item.get("feature") == "pose" and item.get("value")
    ]
    clothing_predicates: list[dict[str, str]] = []
    for item in filters:
        if item.get("feature") != "shirt_color" or not item.get("value"):
            continue
        value = str(item["value"])
        # Section: Preserve a negative shirt-color request for the server's S_err path.
        is_negative = bool(
            re.search(
                rf"\b(?:not\s+wearing|without)\s+(?:a\s+)?{re.escape(value.replace('_', ' '))}\s+shirt\b",
                text,
            )
        )
        clothing_predicates.append(
            {"type": "shirt_color", "operator": "not" if is_negative else "equals", "value": value}
        )
    wearable_predicates: list[dict[str, str]] = []
    unavailable_wearables: list[str] = []
    visible_nouns = {"glasses": "glasses", "hat": "hat", "face_mask": "mask"}
    for feature, phrases in {
            "glasses": ("wearing glasses", "with glasses", "has glasses"),
            "hat": ("wearing a hat", "wearing hat", "with a hat", "with hat"),
            "face_mask": ("wearing a mask", "wearing mask", "with a mask", "with mask"),
    }.items():
        # Section: Negative visible attributes are dispatched to the skill's S_err
        # report-and-return path instead of being silently parsed as a positive match.
        if re.search(
            rf"\b(?:not\s+wearing|without|not\s+with)\s+(?:a\s+)?{visible_nouns[feature]}\b",
            text,
        ):
            wearable_predicates.append(
                {"type": feature, "operator": "not", "value": "yes"}
            )
            continue
        if not any(re.search(rf"\b{re.escape(phrase)}\b", text) for phrase in phrases):
            continue
        # Section: Model-gated wearables are accepted only when configuration enables YES.
        enabled_values = config.supported_person_attributes.get(feature, [])
        if not {"yes", "true", "1"}.intersection(enabled_values):
            unavailable_wearables.append(feature)
            continue
        wearable_predicates.append(
            {"type": feature, "operator": "equals", "value": "yes"}
        )
    if unavailable_wearables:
        return {}, [
            ValidationIssue(
                "unavailable_person_attribute_model",
                f"No validated positive-class model is configured for {unavailable_wearables}.",
            )
        ]
    predicates: list[dict[str, str]] = []
    predicates.extend({"type": "gesture" if value in {"right_hand_raised", "left_hand_raised", "waving"} else "pose", "operator": "equals", "value": value} for value in pose_values)
    predicates.extend(clothing_predicates)
    predicates.extend(wearable_predicates)

    # Section: Reject combined or multiple selectors instead of choosing a hidden precedence.
    if searched_names and predicates:
        return {}, [ValidationIssue("conflicting_person_selectors", "Name and attribute person selectors cannot be combined.")]
    if len(searched_names) > 1:
        return {}, [ValidationIssue("conflicting_person_selectors", "Find-person command resolved more than one name.")]
    if searched_names:
        return {"type": "name", "value": searched_names[0]}, []
    if predicates:
        return {"all": predicates}, []
    if re.search(r"\b(?:person|man|woman|someone|guest)\b", text):
        return {"type": "any_person", "value": ""}, []
    return {}, []


def _requested_person_fields(text: str, task_type: str) -> list[str]:
    """Special function: Extract explicit guest-information fields from a find-person command."""
    if task_type != "find_person":
        return []
    # Section: Preserve a deterministic reporting order independent of phrase order.
    patterns = {
        "name": r"\b(?:their|his|her|the guest'?s|person'?s)\s+name\b|\bwhat(?:'s| is)\s+(?:their|his|her)\s+name\b|\bname\s+of\s+(?:the\s+)?(?:person|guest)\b",
        "age": r"\b(?:their|his|her|the guest'?s|person'?s)\s+age\b|\bhow old\b",
        "gender": r"\b(?:their|his|her|the guest'?s|person'?s)\s+gender\b",
        "favorite_drink": r"\b(?:favorite|favourite)\s+drink\b",
        "hometown": r"\b(?:hometown|where (?:they|he|she) (?:are|is) from)\b",
        "shirt_color": r"\b(?:shirt|clothes|clothing)\s+colou?r\b",
        "glasses": r"\bwhether (?:they|he|she) (?:are|is) wearing glasses\b",
        "hat": r"\bwhether (?:they|he|she) (?:are|is) wearing (?:a )?hat\b",
        "face_mask": r"\bwhether (?:they|he|she) (?:are|is) wearing (?:a )?mask\b",
        "pose": r"\b(?:their|his|her)\s+pose\b|\bpose\s+of\s+(?:the\s+)?(?:person|guest)\b",
        "gesture": r"\b(?:their|his|her)\s+gesture\b|\bgesture\s+of\s+(?:the\s+)?(?:person|guest)\b",
    }
    return [field for field, pattern in patterns.items() if re.search(pattern, text)]


def _contains_known_target(text: str, config: SchedulerConfig) -> bool:
    """Special function: Detect whether a pronoun has an explicit known target nearby."""
    symbols = config.known_objects | config.known_categories | config.known_people
    return any(re.search(rf"\b{re.escape(symbol.replace('_', ' '))}\b", text) for symbol in symbols)


def _detect_task_type(text: str, config: SchedulerConfig) -> str:
    """Special function: Classify the supported GPSR task family."""
    # Section: World-grounded dialogue must perceive the person before speaking its answer.
    if _requested_person_fields(text, "find_person") and re.search(
        r"\b(?:person|guest|someone|man|woman)\b", text
    ):
        return "find_person"
    if re.search(r"\b(?:compare|which)\b.*\b(?:biggest|largest|smallest|heaviest|lightest|thinnest)\b", text) or re.search(
        r"\b(?:biggest|largest|smallest|heaviest|lightest|thinnest)\b.*\b(?:object|item|one)\b", text
    ):
        return "compare_objects"
    if re.search(r"\b(how many|count)\b.*\b(people|persons|humans)\b", text):
        return "count_people"
    if re.search(r"\b(how many|count)\b", text):
        return "count_objects"
    if re.search(r"\b(find|locate|look for)\b.*\b(person|man|woman|someone|guest)\b", text) or re.search(
        r"^meet\b.*\b(person|man|woman|someone|guest)\b", text
    ):
        return "find_person"
    # Section: A configured person name after a search verb selects the named-person task.
    name_search_clause = re.split(r"\b(?:tell|report|say|answer)\b", text, maxsplit=1)[0]
    if any(
        re.search(rf"\b(?:find|locate|look for)\b.*\b{re.escape(person.replace('_', ' '))}\b", name_search_clause)
        or re.search(rf"^meet\b.*\b{re.escape(person.replace('_', ' '))}\b", name_search_clause)
        for person in config.known_people
    ):
        return "find_person"
    # Section: MEET is a person-search verb even when the target needs clarification.
    if re.search(r"^meet\b", text):
        return "find_person"
    if re.search(r"\b(find|locate|look for)\b", text):
        return "find_object"
    if re.search(r"\b(?:bring|deliver)\b", text):
        return "deliver_object"
    if re.search(r"\b(?:take|pick up|pick|grasp)\b", text):
        return "acquire_object"
    if re.search(r"\b(?:place|put)\b", text):
        return "place_object"
    if re.search(r"\b(?:follow)\b", text):
        return "follow_person"
    if re.search(r"\b(?:guide|lead)\b", text):
        return "guide_person"
    if re.search(r"\baccompany\b", text):
        if re.search(r"\b(?:they|person|guest|human)\s+(?:lead|leads|is leading)\b|\bhuman leads\b", text):
            return "follow_person"
        if re.search(r"\b(?:robot|you)\s+(?:lead|leads|are leading)\b|\brobot leads\b", text):
            return "guide_person"
        return "accompany_ambiguous"
    # Section: Package-owned date/time and configured robot/team facts use their query skill.
    if _is_fixed_information_query(text):
        return "fixed_information"
    if re.search(r"\b(?:ask)\b|\btalk\b.*\b(?:and listen|with response)\b", text):
        return "ask_and_listen"
    if re.search(r"\b(?:say|speak|tell|report|greet)\b", text):
        return "say"
    if re.search(r"\b(?:go|navigate|move)\s+to\b", text):
        return "navigate"
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


def _unknown_location_mentions(text: str, config: SchedulerConfig) -> list[str]:
    """Special function: Flag obvious unsupported room/location phrases."""
    known_locations = config.known_locations
    unknown: list[str] = []
    stop_words = {"and", "then", "tell", "report", "say", "answer", "count", "find", "locate", "look", "go", "navigate", "with", "for"}
    for match in re.finditer(r"(?=\b(?:in|at|to|from|on)\s+(?:the\s+)?([a-z_]+(?:\s+[a-z_]+){0,2}))", text):
        # Section: Image relations such as "to the left of the cup" are not room/surface phrases.
        relation_phrase = text[match.start():]
        if re.match(r"\b(?:to|on)\s+(?:the\s+)?(?:left|right)\s+of\b", relation_phrase):
            continue
        words = []
        for word in match.group(1).split():
            if word in stop_words:
                break
            words.append(word)
        candidate = normalize_symbol(" ".join(words))
        if not candidate or candidate in {"person", "people", "object", "objects", "me"}:
            continue
        if candidate in config.known_people or any(candidate.startswith(person) for person in config.known_people):
            continue
        # Section: Only the nearest governing verb may classify a TO phrase as person-directed.
        governing_verbs = re.findall(
            r"\b(deliver|tell|talk|say|speak|report|answer|ask|follow|guide|greet|accompany|go|navigate|move|count|find|locate|look)\b",
            text[:match.start()],
        )
        person_verbs = {
            "deliver", "tell", "talk", "say", "speak", "report", "answer",
            "ask", "follow", "guide", "greet", "accompany",
        }
        # Section: The location regex is zero-width, so inspect source text at its offset.
        if governing_verbs and governing_verbs[-1] in person_verbs and text[match.start():].lstrip().startswith("to"):
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


def _action_person(
    text: str,
    task_type: str,
    config: SchedulerConfig,
) -> tuple[str, list[ValidationIssue]]:
    """Special function: Resolve person roles for movement, delivery, and communication primitives."""
    # Section: Fixed-information delivery can name a recipient just like literal speech.
    person_tasks = {
        "follow_person",
        "guide_person",
        "deliver_object",
        "say",
        "ask_and_listen",
        "fixed_information",
    }
    if task_type not in person_tasks:
        return "", []
    for person in sorted(config.known_people):
        if re.search(rf"\b{re.escape(person.replace('_', ' '))}\b", text):
            return person, []
    if task_type == "deliver_object" and re.search(r"\b(?:me|operator)\b", text):
        return "operator", []
    # Section: A direct fact phrase after TELL is content, not an unknown person name.
    if task_type == "fixed_information":
        return "", []

    # Section: Extract a likely explicit name only to return a precise closed-vocabulary error.
    patterns = {
        "follow_person": r"\b(?:follow|accompany)\s+([a-z_]+)",
        "guide_person": r"\b(?:guide|lead|accompany)\s+([a-z_]+)",
        "deliver_object": r"\b(?:to|for)\s+([a-z_]+)",
        "say": r"\b(?:tell|greet)\s+([a-z_]+)",
        "ask_and_listen": r"\b(?:ask|talk to)\s+([a-z_]+)",
        "fixed_information": r"\b(?:tell|say|report)\s+([a-z_]+)",
    }
    match = re.search(patterns[task_type], text)
    candidate = normalize_symbol(match.group(1)) if match else ""
    ignored = {"the", "a", "an", "me", "operator", "person", "guest", "human", "everyone", "everybody"}  # Generic audiences do not require identity lookup.
    if candidate and candidate not in ignored:
        return "", [ValidationIssue("unknown_person", f"Unknown person: {candidate}.")]
    if task_type in {"follow_person", "guide_person"}:
        return "", [ValidationIssue("missing_person", "Person-motion task requires a known target person.")]
    return "", []


def _speech_payload(
    text: str,
    task_type: str,
    target_person: str,
) -> tuple[dict[str, Any], list[ValidationIssue]]:
    """Special function: Extract literal SAY or ASK_AND_LISTEN content without query inference."""
    if task_type not in {"say", "ask_and_listen", "fixed_information"}:
        return {}, []
    if task_type == "fixed_information":
        # Section: Remove only a resolved named recipient so the provider receives the fact request itself.
        fact_request = text
        if target_person:
            person_phrase = target_person.replace("_", " ")
            fact_request = re.sub(
                rf"^\s*(?:tell|say|report)\s+(?:to\s+)?{re.escape(person_phrase)}\s+",
                "",
                fact_request,
                count=1,
            ).strip()
        return {"question": fact_request or text}, []
    if task_type == "ask_and_listen":
        match = re.search(r"\bask\b\s+(.*)$", text)
        question = str(match.group(1) if match else text).strip()
        if target_person and question.startswith(target_person.replace("_", " ")):
            question = question[len(target_person.replace("_", " ")):].strip()
        if not question:
            return {}, [ValidationIssue("missing_question", "Ask-and-listen requires literal question text.")]
        return {"question": question, "target_person": target_person}, []
    if re.search(r"\b(?:the\s+)?(?:time|date|weather)\b", text):
        return {}, [ValidationIssue("unsupported_query", "Dynamic query content requires a query-resolution component.")]
    if re.search(r"\bgreet\b", text):
        return {"text": DEFAULT_GREETING, "target_person": target_person}, []  # Preserve an explicit recipient without personalizing the wording.
    match = re.search(r"\b(?:say|speak|report|tell)\b\s+(.*)$", text)
    speech = str(match.group(1) if match else "").strip()
    if target_person and speech.startswith(target_person.replace("_", " ")):
        speech = speech[len(target_person.replace("_", " ")):].strip()
    if not speech:
        return {}, [ValidationIssue("missing_speech_text", "Speech requires literal text.")]
    return {"text": speech, "target_person": target_person}, []


def _is_fixed_information_query(text: str) -> bool:
    """Special function: Recognize provider-owned questions and direct fact triggers."""
    normalized = str(text or "").strip()
    # Section: Accept question/report verbs while keeping incidental fact words inside physical tasks unchanged.
    request_form = bool(re.search(
        r"^(?:what|what's|who|when|tell(?: me)?|say|report)\b",
        normalized,
    ))
    # Section: These domains mirror runtime and static entries in fixed_information.yaml.
    owned_domain = re.search(
        r"\b(?:time|date|today|tomorrow|weekday|day of (?:the )?(?:week|month)|"
        r"team(?:'s|s)? (?:name|country|affiliation)|who are you|about yourself)\b",
        normalized,
    )
    if owned_domain is None:
        return False
    # Section: Bare noun phrases directly trigger the provider without pretending to be questions.
    direct_fact = bool(re.fullmatch(
        r"(?:the\s+)?(?:current\s+)?(?:time|date)|today(?:'s\s+date)?|"
        r"tomorrow(?:'s\s+date)?|(?:the\s+)?(?:weekday|day of (?:the )?(?:week|month))|"
        r"(?:your\s+)?team(?:'s|s)?\s+(?:name|country|affiliation)|about yourself",
        normalized,
    ))
    return request_form or direct_fact


def _target(text: str, task_type: str, config: SchedulerConfig) -> dict[str, Any]:
    """Special function: Resolve object/category target semantics."""
    if task_type == "compare_objects":
        objects = [
            config.object_aliases.get(object_name, object_name)
            for object_name in config.known_objects
            if re.search(rf"\b{re.escape(object_name.replace('_', ' '))}s?\b", text)
        ]
        if objects:
            return {"object_names": sorted(set(objects))}
        for category in config.known_categories:
            if re.search(rf"\b{re.escape(category.replace('_', ' '))}s?\b", text):
                return {"category": category}
        if re.search(r"\b(?:object|objects|item|items)\b", text):
            return {"category": "objects"}
        return {}
    if task_type not in {"find_object", "count_objects", "acquire_object", "place_object", "deliver_object"}:
        return {}
    for object_name in config.known_objects:
        if re.search(rf"\b{re.escape(object_name.replace('_', ' '))}s?\b", text):
            return {"object_name": config.object_aliases.get(object_name, object_name)}
    for category in config.known_categories:
        if re.search(rf"\b{re.escape(category.replace('_', ' '))}s?\b", text):
            return {"category": category}
    return {}


def _comparison_property(text: str) -> str:
    """Special function: Resolve the canonical property used by COMPARE_OBJECTS."""
    aliases = {
        "biggest": "biggest", "largest": "largest", "smallest": "smallest",
        "heaviest": "heaviest", "lightest": "lightest", "thinnest": "thinnest",
    }
    for phrase, canonical in aliases.items():
        if re.search(rf"\b{phrase}\b", text):
            return canonical
    return ""


def strip_object_mentions(text: str, config: SchedulerConfig) -> str:
    """Special function: Hide named objects before reading separate attributes."""
    cleaned = text
    # Section: Longest aliases win so a complete branded name hides its color word.
    for alias in sorted(config.known_objects, key=len, reverse=True):
        phrase = alias.replace("_", " ")
        cleaned = re.sub(rf"\b{re.escape(phrase)}\b", " ", cleaned)
    return cleaned


def _filters(text: str, config: SchedulerConfig, task_type: str) -> list[dict[str, Any]]:
    """Special function: Extract task-compatible object features and human pose filters."""
    filters: list[dict[str, Any]] = []
    # Section: An object's published name does not itself request a color filter.
    feature_text = strip_object_mentions(text, config)
    for feature, values in config.supported_object_features.items():
        # Section: A shirt attribute belongs to person appearance; an object color belongs to Qwen ROI filtering.
        if task_type in {"find_object", "count_objects", "acquire_object", "place_object", "deliver_object"} and feature == "shirt_color":
            continue
        if task_type in {"find_person", "count_people"} and feature == "object_color":
            continue
        for value in values:
            if re.search(rf"\b{re.escape(value.replace('_', ' '))}\b", feature_text):
                filters.append({"feature": feature, "operator": "equals", "value": value})
    for pose in config.supported_human_poses:
        if re.search(rf"\b{re.escape(pose.replace('_', ' '))}\b", text):
            filters.append({"feature": "pose", "operator": "equals", "value": pose})
    if re.search(r"\bnot\s+(?:a\s+|the\s+)?([a-z_ ]+)", text):
        filters.append({"feature": "negation", "operator": "not", "value": True})
    return filters


def _requires_task_location(task_type: str) -> bool:
    """Special function: Declare task families that require physical navigation first."""
    # Section: FIND_OBJECT and FIND_PERSON own global or strict-location navigation internally.
    return task_type in {"count_objects", "count_people", "guide_person"}


def _requires_report(text: str) -> bool:
    """Special function: Detect whether the command asks for a separate reporting destination."""
    return bool(re.search(r"\b(?:tell|report|say|answer)\b.*\b(?:in|at|to)\b", text) or re.search(r"\b(?:go|navigate|return)\s+to\b.*\b(?:tell|report|say|answer)\b", text))


def _report_after_return(text: str) -> bool:
    """Special function: Detect requests that require returning before speaking."""
    # Section: Keep the operator-relative phrase explicit instead of guessing from every report verb.
    return bool(
        re.search(
            r"\b(?:report|tell|say|answer)\s+back\s+to\s+(?:me|the\s+operator)\b",
            text,
        )
        or re.search(
            r"\breturn\s+(?:back\s+)?to\s+(?:me|the\s+operator)\b.*\b(?:report|tell|say|answer)\b",
            text,
        )
    )


def _output_template(text: str, task_type: str, requested_person_fields: list[str] | None = None) -> str | None:
    """Special function: Select deterministic output text template for task results."""
    if not _requires_report(text) and "tell" not in text and "report" not in text and "say" not in text and "answer" not in text:
        return None
    if task_type == "count_people":
        return "I counted {result.count} people."
    if task_type == "count_objects":
        return "I counted {result.count} objects."
    if task_type == "compare_objects":
        return "The {result.selected_object} is the {property}."
    if task_type == "find_person":
        fields = list(requested_person_fields or [])
        if fields == ["name"]:
            return "The guest's name is {result.name}."
        if fields == ["favorite_drink"]:
            return "The guest's favorite drink is {result.favorite_drink}."
        if fields == ["pose"]:
            return "The guest's pose is {result.pose}."
        if fields == ["gesture"]:
            return "The guest's gesture is {result.gesture}."
        if fields == ["name", "favorite_drink"]:
            return "The guest is {result.name} and likes {result.favorite_drink}."
        return "I found the requested person."
    if task_type == "find_object":
        return "I found the requested object."
    return "{result.message}"
