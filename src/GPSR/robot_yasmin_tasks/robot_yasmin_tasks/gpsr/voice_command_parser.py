"""Section: Spoken command planner for GPSR symbolic queues."""

import re

from robot_yasmin_tasks.gpsr.action_types import COUNT_PEOPLE, FIND_OBJECT, FIND_PERSON, GIVE, NAVIGATE, PICK, SAY
from robot_yasmin_tasks.gpsr.models import ActionStepData


def parse_llm_pairs(pairs: list[str]) -> list[ActionStepData]:
    """Special function: Convert local llm_interfaces/Llm pairs into GPSR steps."""
    steps: list[ActionStepData] = []
    for pair in pairs:
        if "|" not in pair:
            continue
        action, raw = pair.split("|", 1)
        args = [part.strip() for part in raw.split(",") if part.strip()]
        action = action.strip().lower()
        if action == "navigate" and args:
            steps.append(ActionStepData("NAVIGATE", target_location=args[0], timeout_sec=30.0))
        elif action == "find_obj" and args:
            steps.append(ActionStepData("FIND_OBJECT", target_object=args[0], target_location=args[1] if len(args) > 1 else "", timeout_sec=20.0))
        elif action == "find_pers" and args:
            steps.append(ActionStepData("FIND_PERSON", target_location=args[1] if len(args) > 1 else "", parameters={"description": args[0]}, timeout_sec=20.0))
        elif action in {"count_people", "count_persons", "count_humans"}:
            location = _sym(args[0]) if args else ""
            if location and location not in {"here", "current_view"}:
                steps.append(ActionStepData(NAVIGATE, target_location=location, timeout_sec=30.0))
            steps.append(ActionStepData(COUNT_PEOPLE, target_location="" if location in {"here", "current_view"} else location, timeout_sec=8.0))
        elif action in {"grasp", "pick"} and args:
            steps.append(ActionStepData("PICK", target_object=args[0], timeout_sec=20.0))
        elif action == "handover":
            steps.append(ActionStepData("GIVE", target_person=args[0] if args and args[0] in {"operator", "guest"} else "operator", timeout_sec=20.0))
        elif action in {"speak", "say"}:
            steps.append(ActionStepData("SAY", parameters={"text": raw.strip()}, timeout_sec=8.0))
    return steps


def fallback_parse_transcript(transcript: str) -> list[ActionStepData]:
    """Special function: Deterministically plan common spoken GPSR command patterns."""
    text = transcript.strip()
    lower = text.lower()
    if not text:
        return []
    count_people = re.search(r"(?:how\s+many|count)\s+(?:people|persons|humans)(?:\s+(?:are\s+)?(?:in|at)\s+(?:the\s+)?(?P<room>[\w\s]+))?", lower)
    if count_people:
        room = _sym(count_people.group("room") or "")
        if room and room not in {"here", "current_view"}:
            return [ActionStepData(NAVIGATE, target_location=room, timeout_sec=30.0), ActionStepData(COUNT_PEOPLE, target_location=room, timeout_sec=8.0)]
        return [ActionStepData(COUNT_PEOPLE, timeout_sec=8.0)]
    bring = re.search(r"(?:go to|navigate to)\s+(?:the\s+)?(?P<room>[\w\s]+?),?\s+(?:get|grab|take|pick up)\s+(?:the\s+)?(?P<object>[\w\s]+?)(?:\s+from\s+(?:the\s+)?(?P<place>[\w\s]+?))?\s+(?:and\s+)?(?:bring|give|handover)", lower)
    if bring:
        room = _sym(bring.group("room"))
        obj = _sym(bring.group("object"))
        place = _sym(bring.group("place") or "")
        return [ActionStepData(NAVIGATE, target_location=room, timeout_sec=30.0), ActionStepData(FIND_OBJECT, target_object=obj, target_location=place, timeout_sec=20.0), ActionStepData(PICK, target_object=obj, timeout_sec=20.0), ActionStepData(NAVIGATE, target_location="operator", timeout_sec=30.0), ActionStepData(GIVE, target_person="operator", timeout_sec=20.0)]
    find_person = re.search(r"find\s+(?:a\s+|the\s+)?(?P<desc>[\w\s]+?)\s+person\s+(?:in|at)\s+(?:the\s+)?(?P<room>[\w\s]+)", lower)
    if find_person:
        room = _sym(find_person.group("room"))
        desc = find_person.group("desc").strip()
        steps = [ActionStepData(NAVIGATE, target_location=room, timeout_sec=30.0), ActionStepData(FIND_PERSON, target_location=room, parameters={"description": desc}, timeout_sec=20.0)]
        if "introduce" in lower:
            steps.append(ActionStepData(SAY, parameters={"text": "Hello, I am a service robot."}, timeout_sec=8.0))
        return steps
    if lower.startswith("say "):
        return [ActionStepData(SAY, parameters={"text": text[4:].strip()}, timeout_sec=8.0)]
    nav = re.search(r"\b(?:go|navigate|move)\s+to\s+(?:the\s+)?(.+)$", lower)
    if nav:
        return [ActionStepData(NAVIGATE, target_location=_sym(nav.group(1)), timeout_sec=30.0)]
    return []


def _sym(value: str) -> str:
    """Special function: Normalize spoken symbol text."""
    return value.strip().replace(" ", "_").strip("_")
