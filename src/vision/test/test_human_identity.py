"""Section: Deterministic tests for persistent human identity behavior."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest


from human_identity.human_identity.database import IdentityDatabase
from human_identity.human_identity.feature_extractor import FeatureExtractor
from human_identity.human_identity.feature_types import ATTRIBUTE_UNKNOWN, PersonFeatures
from human_identity.human_identity.matcher import IdentityMatcher
from human_identity.human_identity.registration_manager import (
    RegistrationManager,
    rank_bounding_boxes,
    resolve_profile_target,
)


def make_features(vector: list[float], track_id: int = 1) -> PersonFeatures:
    """Special function: Build a complete synthetic identity feature record."""
    values = np.asarray(vector, dtype=np.float32)
    return PersonFeatures(
        track_id=track_id,
        body_vector=values,
        upper_histogram=values,
        lower_histogram=values,
        object_vector=values,
        body_quality=1.0,
        upper_quality=1.0,
        lower_quality=1.0,
        object_quality=1.0,
    )


def test_person_ids_are_persistent_and_ascending(tmp_path: Path) -> None:
    """Special function: SQLite AUTOINCREMENT remains ascending after reopening the database."""
    database_path = tmp_path / "identity.db"
    database = IdentityDatabase(str(database_path))
    first_id = database.create_registration_profile()
    second_id = database.create_registration_profile()
    assert [first_id, second_id] == [1, 2]
    database.close()
    reopened = IdentityDatabase(str(database_path))
    assert reopened.create_registration_profile() == 3
    reopened.close()


def test_body_identity_can_bind_shared_canonical_uuid() -> None:
    """Special function: Vision keeps its own features while sharing semantic identity."""
    database = IdentityDatabase(":memory:")
    person_id = database.create_registration_profile("person-uuid-42")
    assert database.get_person_profile(person_id)["canonical_person_uuid"] == "person-uuid-42"
    database.bind_canonical_person(person_id, "person-uuid-43")
    assert database.get_person_profile(person_id)["canonical_person_uuid"] == "person-uuid-43"
    database.close()


def test_person_id_zero_is_reserved_for_unknown() -> None:
    """Special function: Unknown ID zero is neither stored nor accepted for profile updates."""
    database = IdentityDatabase(":memory:")
    assert database.person_exists(0) is False
    assert database.get_person_profile(0) is None
    with pytest.raises(ValueError, match="reserved"):
        database.update_person_profile(0, "Unknown", 0, "", True, False, False)
    database.close()


def test_explicit_profile_update_changes_only_selected_fields() -> None:
    """Special function: Field update flags preserve profile values that were not selected."""
    database = IdentityDatabase(":memory:")
    person_id = database.create_registration_profile()
    database.update_person_profile(person_id, "Alex", 24, "orange juice", True, True, True)
    database.update_person_profile(person_id, "Ignored", 25, "tea", False, True, False)
    assert database.get_person_profile(person_id) == {
        "person_id": person_id,
        "canonical_person_uuid": "",
        "person_name": "Alex",
        "age": 25,
        "favourite_drink": "orange juice",
    }
    database.close()


def test_nearest_profile_update_resolves_locked_registration_target() -> None:
    """Special function: Resolve nearest updates through the locked registration pair."""
    registration = RegistrationManager()
    registration.start(person_id=7, track_id=41, started_nanoseconds=100)
    person_id, track_id = resolve_profile_target(
        True,
        requested_person_id=999,
        registration=registration,
    )
    assert (person_id, track_id) == (7, 41)


def test_registration_locks_one_target_until_matching_finish() -> None:
    """Special function: Prevent starts and foreign tokens from replacing a target."""
    registration = RegistrationManager()
    context = registration.start(person_id=3, track_id=12, started_nanoseconds=100)
    with pytest.raises(RuntimeError, match="already active"):
        registration.start(person_id=4, track_id=13, started_nanoseconds=200)
    assert registration.finish("wrong-token") is False
    assert registration.resolve_active_target() == (3, 12)
    assert registration.finish(context.registration_token) is True
    with pytest.raises(RuntimeError, match="No active"):
        registration.resolve_active_target()


def test_nearest_bbox_ranking_uses_descending_visible_area() -> None:
    """Special function: Largest person box receives nearest rank one deterministically."""
    observations = [
        {"track_id": 10, "area": 100.0},
        {"track_id": 11, "area": 900.0},
        {"track_id": 12, "area": 400.0},
    ]
    ranked = rank_bounding_boxes(observations)
    assert [(item["track_id"], item["rank"]) for item in ranked] == [(11, 1), (12, 2), (10, 3)]


def test_attributes_remain_unknown_without_attribute_model() -> None:
    """Special function: Missing attribute weights never fabricate false glasses or mask values."""
    extractor = FeatureExtractor(attribute_model=None, pose_model_path=None, device="cpu")
    frame = np.full((120, 60, 3), 127, dtype=np.uint8)
    features = extractor.extract(frame, (0, 0, 60, 120), track_id=8)
    assert features.visual_attributes.wearing_glasses == ATTRIBUTE_UNKNOWN
    assert features.visual_attributes.wearing_face_mask == ATTRIBUTE_UNKNOWN
    assert features.visual_attributes.glasses_confidence == 0.0
    assert features.visual_attributes.face_mask_confidence == 0.0


def test_identity_matcher_rejects_ambiguous_candidates() -> None:
    """Special function: Two similarly strong identities produce AMBIGUOUS and person ID zero."""
    matcher = IdentityMatcher(threshold=0.72, ambiguity_margin=0.08)
    query = make_features([1.0, 0.0])
    prototypes = [(1, make_features([1.0, 0.0])), (2, make_features([0.999, 0.045]))]
    result = matcher.match(query, prototypes)
    assert result.status == "AMBIGUOUS"
    assert result.person_id == 0
    assert result.final_score >= 0.72
    assert result.final_score - result.second_score < 0.08


def test_identity_matcher_recognizes_separated_candidate_and_database_round_trip() -> None:
    """Special function: Recognize a separated prototype after SQLite storage."""
    database = IdentityDatabase(":memory:")
    first_id = database.create_registration_profile()
    second_id = database.create_registration_profile()
    database.add_prototype(first_id, make_features([1.0, 0.0]))
    database.add_prototype(second_id, make_features([-1.0, 0.0]))
    result = IdentityMatcher(threshold=0.72, ambiguity_margin=0.08).match(
        make_features([1.0, 0.0]),
        database.iter_prototypes(),
    )
    assert result.status == "RECOGNIZED"
    assert result.person_id == first_id
    database.close()
