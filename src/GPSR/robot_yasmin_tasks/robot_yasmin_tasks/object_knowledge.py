"""Section: Authoritative GPSR object vocabulary and target-resolution policy."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import yaml


class ObjectKnowledgeError(ValueError):
    """Special function: Report malformed knowledge or unsupported target semantics."""


class TargetKind(str, Enum):
    """Section: Distinguish exact, category, wildcard, and invalid requests explicitly."""

    EXACT_OBJECT = "EXACT_OBJECT"
    KNOWN_CATEGORY = "KNOWN_CATEGORY"
    EXPLICIT_WILDCARD = "EXPLICIT_WILDCARD"
    INVALID_CATEGORY = "INVALID_CATEGORY"


@dataclass(frozen=True, slots=True)
class ResolvedObjectTarget:
    """Special function: Carry normalized matching and location-prior semantics."""

    kind: TargetKind
    canonical_name: str
    detector_labels: tuple[str, ...]
    probable_locations: tuple[str, ...]


def normalize_object_symbol(value: Any) -> str:
    """Special function: Normalize spoken aliases without losing word boundaries."""

    return "_".join(str(value or "").strip().casefold().replace("-", " ").split())


class ObjectKnowledge:
    """Special function: Validate and query the single GPSR object knowledge document."""

    def __init__(self, data: dict[str, Any]) -> None:
        """Special function: Validate references and build deterministic lookup indexes."""
        # Section: Normalize the three top-level collections before validating references.
        self.objects = {
            normalize_object_symbol(name): dict(payload or {})
            for name, payload in dict(data.get("objects", {})).items()
        }
        self.categories = {
            normalize_object_symbol(name): dict(payload or {})
            for name, payload in dict(data.get("categories", {})).items()
        }
        self.wildcards = {
            normalize_object_symbol(value)
            for value in list(data.get("wildcards", []))
            if normalize_object_symbol(value)
        }
        if not self.objects:
            raise ObjectKnowledgeError("Object knowledge must define at least one object.")
        if not self.wildcards:
            raise ObjectKnowledgeError("Object knowledge must define an explicit wildcard.")

        # Section: One alias maps to exactly one canonical object, including canonical names.
        self.alias_to_object: dict[str, str] = {}
        for canonical_name, payload in self.objects.items():
            aliases = [canonical_name, *list(payload.get("aliases", []))]
            for raw_alias in aliases:
                alias = normalize_object_symbol(raw_alias)
                if not alias:
                    raise ObjectKnowledgeError(
                        f"Object '{canonical_name}' contains an empty alias."
                    )
                previous = self.alias_to_object.get(alias)
                if previous is not None and previous != canonical_name:
                    raise ObjectKnowledgeError(
                        f"Duplicate object alias '{alias}' belongs to '{previous}' and "
                        f"'{canonical_name}'."
                    )
                self.alias_to_object[alias] = canonical_name

        # Section: Map singular and plural category phrases to one canonical category.
        self.alias_to_category: dict[str, str] = {}
        for canonical_name, payload in self.categories.items():
            for raw_alias in [canonical_name, *list(payload.get("aliases", []))]:
                alias = normalize_object_symbol(raw_alias)
                if not alias or alias in self.alias_to_object or alias in self.wildcards:
                    raise ObjectKnowledgeError(
                        f"Invalid category alias '{alias}' for '{canonical_name}'."
                    )
                previous = self.alias_to_category.get(alias)
                if previous is not None and previous != canonical_name:
                    raise ObjectKnowledgeError(
                        f"Duplicate category alias '{alias}' belongs to '{previous}' and "
                        f"'{canonical_name}'."
                    )
                self.alias_to_category[alias] = canonical_name

        # Section: Category membership must agree in both directions with defined objects.
        for category_name, payload in self.categories.items():
            for raw_member in list(payload.get("members", [])):
                member = normalize_object_symbol(raw_member)
                if member not in self.objects:
                    raise ObjectKnowledgeError(
                        f"Category '{category_name}' references missing object '{member}'."
                    )
        for object_name, payload in self.objects.items():
            category = normalize_object_symbol(payload.get("category", ""))
            if category and category not in self.categories:
                raise ObjectKnowledgeError(
                    f"Object '{object_name}' references unknown category '{category}'."
                )
            if category and object_name not in {
                normalize_object_symbol(item)
                for item in self.categories[category].get("members", [])
            }:
                raise ObjectKnowledgeError(
                    f"Object '{object_name}' is absent from category '{category}' members."
                )

    @classmethod
    def from_file(cls, path: str | Path) -> "ObjectKnowledge":
        """Special function: Load one YAML knowledge document from an explicit path."""
        knowledge_path = Path(path)
        if not knowledge_path.is_file():
            raise ObjectKnowledgeError(f"Object knowledge file does not exist: {knowledge_path}")
        payload = yaml.safe_load(knowledge_path.read_text(encoding="utf-8")) or {}
        if not isinstance(payload, dict):
            raise ObjectKnowledgeError("Object knowledge root must be a YAML mapping.")
        return cls(payload)

    @staticmethod
    def default_path() -> Path:
        """Special function: Resolve installed config with a source-tree fallback."""
        try:
            from ament_index_python.packages import get_package_share_directory

            installed = Path(get_package_share_directory("robot_yasmin_tasks"))
            installed_file = installed / "config" / "object_knowledge.yaml"
            # Section: A stale install may predate this knowledge file during source tests.
            if installed_file.is_file():
                return installed_file
        except Exception:
            pass
        return Path(__file__).resolve().parents[1] / "config" / "object_knowledge.yaml"

    def resolve(self, object_name: str = "", category: str = "") -> ResolvedObjectTarget:
        """Special function: Resolve target semantics and fail closed on unknown categories."""
        normalized_object = normalize_object_symbol(object_name)
        normalized_category = normalize_object_symbol(category)
        if normalized_object and normalized_category:
            raise ObjectKnowledgeError(
                "Specify either object_name or category, not both."
            )
        target = normalized_object or normalized_category
        if not target:
            raise ObjectKnowledgeError("Object target is empty.")
        if target in self.wildcards:
            return ResolvedObjectTarget(
                TargetKind.EXPLICIT_WILDCARD,
                target,
                tuple(self.detector_prompts()),
                tuple(),
            )
        canonical_object = self.alias_to_object.get(target)
        if canonical_object is not None and normalized_object:
            payload = self.objects[canonical_object]
            detector_label = normalize_object_symbol(
                payload.get("detector_label", canonical_object)
            )
            return ResolvedObjectTarget(
                TargetKind.EXACT_OBJECT,
                canonical_object,
                (detector_label,),
                tuple(
                    normalize_object_symbol(item)
                    for item in payload.get("probable_locations", [])
                ),
            )
        canonical_category = self.alias_to_category.get(target)
        if canonical_category is not None:
            members = tuple(
                normalize_object_symbol(item)
                for item in self.categories[canonical_category].get("members", [])
            )
            labels = tuple(
                normalize_object_symbol(
                    self.objects[member].get("detector_label", member)
                )
                for member in members
            )
            locations = tuple(
                normalize_object_symbol(item)
                for item in self.categories[canonical_category].get("probable_locations", [])
            )
            return ResolvedObjectTarget(
                TargetKind.KNOWN_CATEGORY,
                canonical_category,
                labels,
                locations,
            )
        # Section: An alias passed in the category field is invalid rather than reinterpreted.
        raise ObjectKnowledgeError(f"UNSUPPORTED_TARGET: '{target}' is not configured.")

    def detector_prompts(self) -> list[str]:
        """Special function: Return readable YOLOE prompts in stable object order."""
        prompts: list[str] = []
        for object_name in self.objects:
            label = normalize_object_symbol(
                self.objects[object_name].get("detector_label", object_name)
            )
            # Section: Spoken identifiers use underscores; visual prompts need natural spaces.
            prompt = label.replace("_", " ")
            if prompt not in prompts:
                prompts.append(prompt)
        return prompts

    def known_object_names(self) -> set[str]:
        """Special function: Expose canonical names and aliases to deterministic parsing."""
        return set(self.alias_to_object)

    def known_category_names(self) -> set[str]:
        """Special function: Expose valid categories plus explicit generic wildcards."""
        return set(self.alias_to_category) | set(self.wildcards)
