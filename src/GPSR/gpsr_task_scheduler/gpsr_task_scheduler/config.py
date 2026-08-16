"""Section: Scheduler YAML configuration loading."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from robot_yasmin_tasks.gpsr.action_types import (
    PrimitiveCapability,
    load_primitive_registry,
)

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
    known_people: set[str] = field(default_factory=set)
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
    return SchedulerConfig(
        known_locations={"kitchen", "living_room", "bedroom", "dining_room", "entrance", "operator", "instruction_point"},
        known_objects={"apple", "banana", "bottle", "chair", "coke", "cup", "milk", "chips"},
        known_categories={"drink", "food", "object"},
        known_people={"operator", "john", "mary"},
        supported_object_features={"shirt_color": ["red", "blue", "green", "black", "white"], "object_color": ["red", "blue", "green", "black", "white"]},
        supported_human_poses={"right_hand_raised", "left_hand_raised", "waving", "sitting", "standing"},
        supported_person_attributes={
            "shirt_color": ["red", "orange", "yellow", "green", "blue", "purple", "white", "black", "gray"],
            "glasses": ["yes"],
        },
        skill_timeouts={"NAVIGATE": 60.0, "FIND_OBJECT": 300.0, "FIND_PERSON": 20.0, "FIND_PERSON_BY_ATTRIBUTE": 20.0, "EXTRACT_PERSON_INFO": 10.0, "COUNT_OBJECTS": 10.0, "COUNT_PEOPLE": 10.0, "PICK_OBJECT": 20.0, "PLACE_OBJECT": 20.0, "HANDOVER_OBJECT": 20.0, "FOLLOW_PERSON": 60.0, "GUIDE_PERSON": 60.0, "SAY": 8.0, "ASK_AND_LISTEN": 15.0},
        skill_retries={"NAVIGATE": 1, "FIND_OBJECT": 0, "FIND_PERSON": 0, "FIND_PERSON_BY_ATTRIBUTE": 0, "EXTRACT_PERSON_INFO": 0, "COUNT_OBJECTS": 0, "COUNT_PEOPLE": 0, "PICK_OBJECT": 0, "PLACE_OBJECT": 0, "HANDOVER_OBJECT": 0, "FOLLOW_PERSON": 0, "GUIDE_PERSON": 0, "SAY": 0, "ASK_AND_LISTEN": 0},
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
        known_objects={normalize_symbol(item) for item in data.get("known_objects", config.known_objects)},
        known_categories={normalize_symbol(item) for item in data.get("known_categories", config.known_categories)},
        known_people={normalize_symbol(item) for item in data.get("known_people", config.known_people)},
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
