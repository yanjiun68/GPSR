"""Section: Canonical executable primitive vocabulary and capability registry."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


# Section: ActionQueue contains only executable primitives, never surface-language intents.
NAVIGATE = "NAVIGATE"
FIND_OBJECT = "FIND_OBJECT"
FIND_PERSON = "FIND_PERSON"
FIND_PERSON_BY_ATTRIBUTE = "FIND_PERSON_BY_ATTRIBUTE"
COUNT_OBJECTS = "COUNT_OBJECTS"
COUNT_PEOPLE = "COUNT_PEOPLE"
EXTRACT_PERSON_INFO = "EXTRACT_PERSON_INFO"
PICK_OBJECT = "PICK_OBJECT"
PLACE_OBJECT = "PLACE_OBJECT"
HANDOVER_OBJECT = "HANDOVER_OBJECT"
FOLLOW_PERSON = "FOLLOW_PERSON"
GUIDE_PERSON = "GUIDE_PERSON"
SAY = "SAY"
ASK_AND_LISTEN = "ASK_AND_LISTEN"
FIXED_INFORMATION = "FIXED_INFORMATION"


# Section: One canonical set is shared by scheduler validation, lifecycle policy, and dispatch.
CANONICAL_PRIMITIVES = {
    NAVIGATE,
    FIND_OBJECT,
    FIND_PERSON,
    FIND_PERSON_BY_ATTRIBUTE,
    COUNT_OBJECTS,
    COUNT_PEOPLE,
    EXTRACT_PERSON_INFO,
    PICK_OBJECT,
    PLACE_OBJECT,
    HANDOVER_OBJECT,
    FOLLOW_PERSON,
    GUIDE_PERSON,
    SAY,
    ASK_AND_LISTEN,
    FIXED_INFORMATION,
}
ALLOWED_ACTION_TYPES = CANONICAL_PRIMITIVES


# Section: Temporary one-release aliases are accepted only at queue admission boundaries.
LEGACY_ACTION_ALIASES = {
    "PICK": PICK_OBJECT,
    "PLACE": PLACE_OBJECT,
    "GIVE": HANDOVER_OBJECT,
    "FOLLOW": FOLLOW_PERSON,
    "GUIDE": GUIDE_PERSON,
    "ASK": ASK_AND_LISTEN,
    "RETURN_TO_INSTRUCTION_POINT": NAVIGATE,
    "REPORT": SAY,
    "GREET": SAY,
    # Section: Historical feature-specific searches use the unified FIND_OBJECT contract.
    "FIND_EXTREME_OBJECT": FIND_OBJECT,
}


# Section: Only transient GPSR skill processes may be lifecycle-managed by ActionQueue.
GPSR_MANAGED_LIFECYCLE_NODES = frozenset({
    "/count_objects_perception_server",
    "/count_objects_skill_server",
    "/count_people_perception_server",
    "/count_people_skill_server",
    "/extract_person_info_skill_server",
    "/find_object_skill_server",
    "/find_person_skill_server",
    "/find_person_by_attribute_skill_server",
    "/follow_person_skill_server",
    "/guide_person_skill_server",
    "/gpsr_human_detection_node",
    "/gpsr_human_identity_node",
    "/gpsr_object_detection_node",
    "/gpsr_tracked_appearance_node",
    "/human_skeleton_detection_node",
})


# Section: Keep every GPSR lifecycle control endpoint outside Nav2's manager API.
GPSR_LIFECYCLE_SERVICE_PREFIX = "/gpsr_lifecycle"
GPSR_PREPARE_SKILL_SERVICE = (
    f"{GPSR_LIFECYCLE_SERVICE_PREFIX}/prepare_skill"
)
GPSR_CLEANUP_SKILL_SERVICE = (
    f"{GPSR_LIFECYCLE_SERVICE_PREFIX}/cleanup_skill"
)
GPSR_RECOVER_SKILL_SERVICE = (
    f"{GPSR_LIFECYCLE_SERVICE_PREFIX}/recover_skill"
)
GPSR_GET_SKILL_STATE_SERVICE = (
    f"{GPSR_LIFECYCLE_SERVICE_PREFIX}/get_skill_state"
)


@dataclass(frozen=True, slots=True)
class PrimitiveCapability:
    """Special function: Represent one registry-backed executable primitive contract."""

    name: str
    conflict_group: str
    lifecycle_nodes: tuple[str, ...]
    execution_interface: str
    interface_type: str
    implementation: str
    production_enabled: bool
    legacy_aliases: tuple[str, ...]

    @property
    def production_available(self) -> bool:
        """Special function: Require an explicitly enabled real implementation."""
        return self.implementation == "real" and self.production_enabled


def normalize_primitive_name(raw_name: Any) -> tuple[str, str]:
    """Special function: Normalize a canonical or temporary legacy action identifier."""
    supplied = str(raw_name or "").strip().upper()
    canonical = LEGACY_ACTION_ALIASES.get(supplied, supplied)
    return canonical, supplied if supplied in LEGACY_ACTION_ALIASES else ""


def default_registry_path() -> Path:
    """Special function: Resolve the installed registry with a source-tree fallback."""
    source_candidate = Path(__file__).resolve().parents[2] / "config" / "skill_registry.yaml"
    if source_candidate.exists():
        return source_candidate
    try:
        # Section: Import ament lazily so pure Python scheduler tests remain lightweight.
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("robot_yasmin_tasks")) / "config" / "skill_registry.yaml"
    except Exception:
        return source_candidate


def load_primitive_registry(registry_file: str | Path | None = None) -> dict[str, PrimitiveCapability]:
    """Special function: Load and validate the single primitive capability registry."""
    path = Path(registry_file) if registry_file else default_registry_path()
    payload = json.loads(path.read_text(encoding="utf-8"))
    raw_skills = payload.get("skills", {})
    if not isinstance(raw_skills, dict) or not raw_skills:
        raise RuntimeError(f"Primitive registry has no skills: {path}")

    # Section: Normalize every entry before comparing it with the canonical primitive set.
    registry: dict[str, PrimitiveCapability] = {}
    valid_implementations = {"real", "simulated", "missing"}
    for raw_name, raw_spec in raw_skills.items():
        name = str(raw_name).strip().upper()
        nodes = tuple(str(item).strip().rstrip("/") for item in raw_spec.get("lifecycle_nodes", []))
        implementation = str(raw_spec.get("implementation", "missing")).strip().lower()
        if not name or any(not node.startswith("/") for node in nodes):
            raise RuntimeError(f"Invalid primitive registry entry: {raw_name}")
        if implementation not in valid_implementations:
            raise RuntimeError(f"Invalid implementation status for {name}: {implementation}")
        registry[name] = PrimitiveCapability(
            name=name,
            conflict_group=str(raw_spec.get("conflict_group", "")).strip(),
            lifecycle_nodes=nodes,
            execution_interface=str(raw_spec.get("execution_interface", "")).strip(),
            interface_type=str(raw_spec.get("interface_type", "")).strip(),
            implementation=implementation,
            production_enabled=bool(raw_spec.get("production_enabled", False)),
            legacy_aliases=tuple(str(item).strip().upper() for item in raw_spec.get("legacy_aliases", [])),
        )

    # Section: Registry keys must exactly cover the executable primitive vocabulary.
    missing = sorted(CANONICAL_PRIMITIVES - set(registry))
    unexpected = sorted(set(registry) - CANONICAL_PRIMITIVES)
    if missing or unexpected:
        raise RuntimeError(
            f"Primitive registry/action mismatch: missing={missing}, unexpected={unexpected}"
        )

    # Section: Registry aliases must agree with the temporary compatibility translation.
    declared_aliases = {
        alias: name
        for name, spec in registry.items()
        for alias in spec.legacy_aliases
    }
    mismatched_aliases = {
        alias: (target, LEGACY_ACTION_ALIASES.get(alias))
        for alias, target in declared_aliases.items()
        if LEGACY_ACTION_ALIASES.get(alias) != target
    }
    if mismatched_aliases:
        raise RuntimeError(f"Primitive registry legacy alias mismatch: {mismatched_aliases}")

    # Section: Reject base, navigation, arm, camera, or any unapproved process fail-closed.
    configured_lifecycle_nodes = {
        node_name
        for capability in registry.values()
        for node_name in capability.lifecycle_nodes
    }
    unapproved_lifecycle_nodes = sorted(
        configured_lifecycle_nodes - GPSR_MANAGED_LIFECYCLE_NODES
    )
    if unapproved_lifecycle_nodes:
        raise RuntimeError(
            "Primitive registry contains non-GPSR lifecycle resources: "
            f"{unapproved_lifecycle_nodes}"
        )
    return registry


def unavailable_primitives(
    primitive_names: list[str] | tuple[str, ...] | set[str],
    registry: dict[str, PrimitiveCapability],
    allow_simulated: bool = False,
) -> list[str]:
    """Special function: Return sorted primitives lacking real production capability."""
    return sorted({
        name
        for raw_name in primitive_names
        for name, _legacy in [normalize_primitive_name(raw_name)]
        if name not in registry
        or not (
            registry[name].production_available
            or (allow_simulated and registry[name].implementation == "simulated")
        )
    })
