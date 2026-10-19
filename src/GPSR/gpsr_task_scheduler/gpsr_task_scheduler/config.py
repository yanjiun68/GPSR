"""Section: Scheduler YAML configuration loading."""

# Section: Configuration contracts, defaults, normalization, and YAML loading.

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from robot_yasmin_tasks.gpsr.action_types import (
    PrimitiveCapability,
    load_primitive_registry,
)
from robot_yasmin_tasks.object_knowledge import ObjectKnowledge

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


@dataclass(slots=True)
class SchedulerConfig:
    """Special function: Store deterministic scheduler vocabulary and policy."""

    known_locations: set[str] = field(default_factory=set)
    known_objects: set[str] = field(default_factory=set)
    known_categories: set[str] = field(default_factory=set)
    object_aliases: dict[str, str] = field(default_factory=dict)
    known_people: set[str] = field(default_factory=set)
    verb_aliases: dict[str, list[str]] = field(default_factory=dict)
    location_aliases: dict[str, str] = field(default_factory=dict)
    person_aliases: dict[str, str] = field(default_factory=dict)
    supported_object_features: dict[str, list[str]] = field(default_factory=dict)
    supported_human_poses: set[str] = field(default_factory=set)
    supported_person_attributes: dict[str, list[str]] = field(default_factory=dict)
    instruction_point: str = "instruction_point"
    return_after_completion: bool = False
    return_after_failure: bool = False
    default_skill_timeout_sec: float = 30.0
    default_skill_retries: int = 0
    skill_timeouts: dict[str, float] = field(default_factory=dict)
    skill_retries: dict[str, int] = field(default_factory=dict)
    actions: dict[str, str] = field(default_factory=dict)
    primitive_capabilities: dict[str, PrimitiveCapability] = field(default_factory=dict)
    allow_simulated_primitives: bool = False

    def timeout_for(self, skill_name: str) -> float:
        """Special function: Resolve per-skill timeout with deterministic fallback."""
        return float(self.skill_timeouts.get(skill_name, self.default_skill_timeout_sec))

    def retries_for(self, skill_name: str) -> int:
        """Special function: Resolve per-skill retry count with deterministic fallback."""
        return int(self.skill_retries.get(skill_name, self.default_skill_retries))


def normalize_symbol(value: Any) -> str:
    """Special function: Normalize spoken and configured symbols for exact matching."""
    return str(value or "").strip().lower().replace("-", "_").replace(" ", "_").strip("_")


def default_config() -> SchedulerConfig:
    """Special function: Provide deterministic defaults for unit tests and missing YAML."""
    # Section: Parser vocabulary derives from the same knowledge used by FindObject matching.
    knowledge = ObjectKnowledge.from_file(ObjectKnowledge.default_path())
    language = _load_language_vocabulary()
    return SchedulerConfig(
        known_locations={"kitchen", "living_room", "bedroom", "dining_room", "entrance", "exit", "operator", "instruction_point"},
        known_objects=knowledge.known_object_names(),
        known_categories=knowledge.known_category_names(),
        object_aliases=dict(knowledge.alias_to_object),
        # Section: Keep fallback vocabulary aligned with the bundled competition generator.
        known_people={
            "operator", "john", "mary", "adel", "angel", "axel", "charlie",
            "jane", "jules", "morgan", "paris", "robin", "simone",
        },
        verb_aliases={
            str(intent).upper(): [str(alias).casefold() for alias in aliases]
            for intent, aliases in dict(language.get("verb_aliases", {})).items()
        },
        location_aliases={
            normalize_symbol(alias): normalize_symbol(canonical)
            for alias, canonical in dict(language.get("location_aliases", {})).items()
        },
        person_aliases={
            normalize_symbol(alias): normalize_symbol(canonical)
            for alias, canonical in dict(language.get("person_aliases", {})).items()
        },
        supported_object_features={"shirt_color": ["red", "blue", "green", "black", "white"], "object_color": ["red", "blue", "green", "black", "white"]},
        supported_human_poses={"right_hand_raised", "left_hand_raised", "waving", "sitting", "standing"},
        supported_person_attributes={
            "shirt_color": ["red", "orange", "yellow", "green", "blue", "purple", "white", "black", "gray"],
            "glasses": ["yes"],
        },
        skill_timeouts={"NAVIGATE": 60.0, "FIND_OBJECT": 300.0, "FIND_PERSON": 300.0, "FIND_PERSON_BY_ATTRIBUTE": 300.0, "EXTRACT_PERSON_INFO": 10.0, "COUNT_OBJECTS": 10.0, "COUNT_PEOPLE": 10.0, "PICK_OBJECT": 60.0, "PLACE_OBJECT": 60.0, "HANDOVER_OBJECT": 60.0, "FOLLOW_PERSON": 60.0, "GUIDE_PERSON": 60.0, "SAY": 8.0, "ASK_AND_LISTEN": 15.0, "FIXED_INFORMATION": 12.0},
        skill_retries={"NAVIGATE": 1, "FIND_OBJECT": 0, "FIND_PERSON": 0, "FIND_PERSON_BY_ATTRIBUTE": 0, "EXTRACT_PERSON_INFO": 0, "COUNT_OBJECTS": 0, "COUNT_PEOPLE": 0, "PICK_OBJECT": 0, "PLACE_OBJECT": 0, "HANDOVER_OBJECT": 0, "FOLLOW_PERSON": 0, "GUIDE_PERSON": 0, "SAY": 0, "ASK_AND_LISTEN": 0, "FIXED_INFORMATION": 0},
        actions={"execute_action_queue": "/execute_action_queue"},
        primitive_capabilities=load_primitive_registry(),
        allow_simulated_primitives=False,
    )


def load_config(
    path: str | Path | None = None,
    skill_registry_file: str | Path | None = None,
) -> SchedulerConfig:
    """Special function: Load scheduler YAML while preserving deterministic defaults."""
    config = default_config()
    config.primitive_capabilities = load_primitive_registry(skill_registry_file)
    if path is None or not str(path):
        return config
    if yaml is None:
        return config
    loaded_path = Path(path)
    if not loaded_path.exists():
        return config
    data = yaml.safe_load(loaded_path.read_text(encoding="utf-8")) or {}
    return SchedulerConfig(
        known_locations={normalize_symbol(item) for item in data.get("known_locations", config.known_locations)},
        # Section: Ignore duplicated scheduler vocabulary and retain authoritative object knowledge.
        known_objects=set(config.known_objects),
        known_categories=set(config.known_categories),
        object_aliases=dict(config.object_aliases),
        known_people={normalize_symbol(item) for item in data.get("known_people", config.known_people)},
        verb_aliases=dict(config.verb_aliases),
        location_aliases=dict(config.location_aliases),
        person_aliases=dict(config.person_aliases),
        supported_object_features={normalize_symbol(key): [normalize_symbol(item) for item in value] for key, value in dict(data.get("supported_object_features", config.supported_object_features)).items()},
        supported_human_poses={normalize_symbol(item) for item in data.get("supported_human_poses", config.supported_human_poses)},
        supported_person_attributes={normalize_symbol(key): [normalize_symbol(item) for item in value] for key, value in dict(data.get("supported_person_attributes", config.supported_person_attributes)).items()},
        instruction_point=normalize_symbol(data.get("instruction_point", config.instruction_point)),
        return_after_completion=bool(data.get("return_after_completion", config.return_after_completion)),
        return_after_failure=bool(data.get("return_after_failure", config.return_after_failure)),
        default_skill_timeout_sec=float(data.get("default_skill_timeout_sec", config.default_skill_timeout_sec)),
        default_skill_retries=int(data.get("default_skill_retries", config.default_skill_retries)),
        skill_timeouts={str(key).upper(): float(value) for key, value in dict(data.get("skill_timeouts", config.skill_timeouts)).items()},
        skill_retries={str(key).upper(): int(value) for key, value in dict(data.get("skill_retries", config.skill_retries)).items()},
        actions={str(key): str(value) for key, value in dict(data.get("actions", config.actions)).items()},
        primitive_capabilities=config.primitive_capabilities,
        allow_simulated_primitives=bool(data.get("allow_simulated_primitives", False)),
    )


def _load_language_vocabulary() -> dict[str, Any]:
    """Special function: Load the single natural-language alias source."""
    # Section: Resolve source and installed paths without introducing another database.
    source_path = Path(__file__).resolve().parents[1] / "config" / "language.yaml"
    try:
        from ament_index_python.packages import get_package_share_directory

        installed_path = Path(get_package_share_directory("gpsr_task_scheduler")) / "config" / "language.yaml"
    except Exception:
        installed_path = source_path
    vocabulary_path = installed_path if installed_path.is_file() else source_path
    if yaml is None or not vocabulary_path.is_file():
        raise RuntimeError(f"GPSR language vocabulary is unavailable: {vocabulary_path}")
    payload = yaml.safe_load(vocabulary_path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, dict):
        raise RuntimeError(f"GPSR language vocabulary must be a mapping: {vocabulary_path}")
    return payload
