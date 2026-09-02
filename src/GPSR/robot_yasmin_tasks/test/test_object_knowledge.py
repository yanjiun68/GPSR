"""Section: Camera-free tests for centralized object vocabulary and fail-closed targets."""

from __future__ import annotations

import pytest

from robot_yasmin_tasks.object_knowledge import (
    ObjectKnowledge,
    ObjectKnowledgeError,
    TargetKind,
)


def test_exact_alias_category_and_wildcard_resolution() -> None:
    """Special function: Verify every supported target kind has distinct semantics."""
    knowledge = ObjectKnowledge.from_file(ObjectKnowledge.default_path())
    exact = knowledge.resolve("coca cola", "")
    category = knowledge.resolve("", "drink")
    wildcard = knowledge.resolve("any object", "")
    assert exact.kind == TargetKind.EXACT_OBJECT
    assert exact.canonical_name == "coke"
    assert exact.detector_labels == ("coke",)
    assert category.kind == TargetKind.KNOWN_CATEGORY
    assert set(category.detector_labels) == {"bottle", "cup", "coke", "milk"}
    assert wildcard.kind == TargetKind.EXPLICIT_WILDCARD
    assert "chair" in wildcard.detector_labels
    # Section: Case, hyphen, and whitespace normalize before alias lookup.
    assert knowledge.resolve("  COCA-COLA  ", "").canonical_name == "coke"


@pytest.mark.parametrize("category", ["fruit", "", "unknown category"])
def test_unknown_or_empty_category_never_becomes_wildcard(category: str) -> None:
    """Special function: Unsupported category input must fail before navigation."""
    knowledge = ObjectKnowledge.from_file(ObjectKnowledge.default_path())
    with pytest.raises(ObjectKnowledgeError):
        knowledge.resolve("", category)


def test_duplicate_alias_and_invalid_category_references_fail() -> None:
    """Special function: Malformed setup-day vocabulary is rejected deterministically."""
    with pytest.raises(ObjectKnowledgeError, match="Duplicate object alias"):
        ObjectKnowledge({
            "objects": {
                "cup": {"aliases": ["vessel"]},
                "bottle": {"aliases": ["vessel"]},
            },
            "wildcards": ["object"],
        })
    with pytest.raises(ObjectKnowledgeError, match="missing object"):
        ObjectKnowledge({
            "objects": {"cup": {"category": "drink"}},
            "categories": {"drink": {"members": ["milk"]}},
            "wildcards": ["object"],
        })


def test_location_prior_lookup_comes_from_object_knowledge() -> None:
    """Special function: Exact object search exposes its configured semantic locations."""
    knowledge = ObjectKnowledge.from_file(ObjectKnowledge.default_path())
    assert knowledge.resolve("milk carton", "").probable_locations == (
        "kitchen",
        "dining_room",
    )
