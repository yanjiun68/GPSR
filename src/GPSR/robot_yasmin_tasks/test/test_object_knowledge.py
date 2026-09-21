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
    assert set(category.detector_labels) == {
        "coke", "red_bull_can", "milk", "soju_bottle", "pepsi_can", "bottle",
    }
    assert wildcard.kind == TargetKind.EXPLICIT_WILDCARD
    assert "chair" in wildcard.detector_labels
    # Section: Case, hyphen, and whitespace normalize before alias lookup.
    assert knowledge.resolve("  COCA-COLA  ", "").canonical_name == "coke"


@pytest.mark.parametrize("category", ["vehicle", "", "unknown category"])
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


def test_incheon_2026_objects_and_category_aliases() -> None:
    """Special function: Keep the published object list available to GPSR commands."""
    knowledge = ObjectKnowledge.from_file(ObjectKnowledge.default_path())
    # Section: Every official Incheon object is present under its published identifier.
    official_names = {
        "white_shirt", "grey_shirt", "blue_shirt", "black_shirt", "hand_towel",
        "rubiks_cube", "pringles", "seaweed", "apple", "peach", "mangostane",
        "lemon", "yellow_bellpepper", "red_bellpepper", "instant_noodles",
        "cornflakes", "coke", "red_bull", "milk", "soju", "pepsi",
        "dishwasher_tab", "sponge", "toothpaste", "cup", "spoon", "plate",
        "knife", "fork", "bowl",
    }
    assert len(official_names) == 30
    assert official_names <= set(knowledge.objects)
    # Section: Old local targets remain available while official class plurals resolve.
    assert {"banana", "bottle", "chair", "chips"} <= set(knowledge.objects)
    assert knowledge.resolve("", "fruits").canonical_name == "fruit"
    assert knowledge.resolve("", "drinks").canonical_name == "drink"
    assert knowledge.resolve("", "cleaning supplies").canonical_name == "cleaning_supply"
    # Section: Detector prompts use natural phrases and preserve matching labels.
    assert "yellow bell pepper" in knowledge.detector_prompts()
    assert knowledge.resolve("yellow bell pepper").detector_labels == ("yellow_bell_pepper",)
