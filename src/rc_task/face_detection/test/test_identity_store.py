"""Unit tests for lightweight face enrollment and open-set recognition policy."""

import pickle

import numpy as np
import pytest

from face_detection.identity_store import (
    IdentityStore,
    TemporalIdentityVoter,
    cosine_similarity,
    normalize_embedding,
    observation_is_fresh,
    observation_is_new,
    observation_stamp,
    recognized_identity_payload,
    unknown_match,
)


# Section: Embedding math must remain deterministic and model-dimension agnostic.
def test_embedding_normalization():
    """Special function: Verify non-zero embeddings are normalized to unit length."""
    normalized = normalize_embedding([3.0, 4.0])
    assert np.allclose(normalized, [0.6, 0.8])
    assert np.isclose(np.linalg.norm(normalized), 1.0)


def test_cosine_similarity():
    """Special function: Verify identical and orthogonal embedding scores."""
    assert cosine_similarity([1.0, 0.0], [1.0, 0.0]) == pytest.approx(1.0)
    assert cosine_similarity([1.0, 0.0], [0.0, 1.0]) == pytest.approx(0.0)
    assert cosine_similarity([], []) == -1.0


# Section: Enrollment freshness tests protect against repeatedly counting one cached frame.
def test_duplicate_timestamp_rejection():
    """Special function: Verify a previously accepted ROS stamp cannot be counted again."""
    observation = {'stamp_sec': 12, 'stamp_nanosec': 34}
    accepted_stamp = observation_stamp(observation)
    assert observation_is_new(observation, None)
    assert not observation_is_new(observation, accepted_stamp)


def test_stale_face_rejection():
    """Special function: Verify observations older than max_face_age_sec are rejected."""
    observation = {'stamp_sec': 10, 'stamp_nanosec': 0}
    assert observation_is_fresh(observation, 10_400_000_000, 0.5)
    assert not observation_is_fresh(observation, 10_600_000_000, 0.5)


# Section: Open-set decisions require both absolute similarity and identity separation.
def _three_dimensional_store(tmp_path):
    """Special function: Build a small model-safe store for matching tests."""
    return IdentityStore(
        tmp_path / 'identities.pkl',
        embedding_model='test_model',
        embedding_dimension=3,
        maximum_templates=3,
    )


def test_unknown_threshold(tmp_path):
    """Special function: Verify a low top-one score remains UNKNOWN."""
    store = _three_dimensional_store(tmp_path)
    store.register_profile('alice', 'Alice', 'guest', [1.0, 0.0, 0.0])
    result = store.recognize_profile([0.0, 1.0, 0.0], threshold=0.45, margin=0.05)
    assert not result['is_known']
    assert result['profile_id'] == ''


def test_top1_top2_margin(tmp_path):
    """Special function: Verify an ambiguous query is rejected despite a high top-one score."""
    store = _three_dimensional_store(tmp_path)
    store.register_profile('alice', 'Alice', 'guest', [1.0, 0.0, 0.0])
    store.register_profile('bob', 'Bob', 'guest', [0.99, 0.1, 0.0])
    result = store.recognize_profile([1.0, 0.05, 0.0], threshold=0.45, margin=0.05)
    assert not result['is_known']
    assert result['score_margin'] < 0.05


def test_multi_template_matching(tmp_path):
    """Special function: Verify a non-centroid template can produce the best identity match."""
    store = _three_dimensional_store(tmp_path)
    store.register_profile(
        'alice',
        'Alice',
        'guest',
        [1.0, 0.0, 0.0],
        templates=[[1.0, 0.0, 0.0], [0.8, 0.6, 0.0]],
    )
    result = store.recognize_profile([0.8, 0.6, 0.0], threshold=0.9, margin=0.05)
    assert result['is_known']
    assert result['profile_id'] == 'alice'
    assert result['match_score'] == pytest.approx(1.0)


# Section: Temporal output must not confirm one noisy frame.
def test_temporal_voting():
    """Special function: Verify three matching votes stabilize identity through one UNKNOWN frame."""
    voter = TemporalIdentityVoter(window_size=5, minimum_votes=3)
    known = {
        'profile_id': 'morgan',
        'name': 'Morgan',
        'role': 'guest',
        'match_score': 0.7,
        'top2_score': 0.2,
        'score_margin': 0.5,
        'is_known': True,
    }
    for sample in (known, known, unknown_match(), known):
        result, stable = voter.add(sample)
    assert stable
    assert result['profile_id'] == 'morgan'


# Section: Dedicated identity topic payload exposes only temporally verified identities.
def test_recognized_identity_payload_requires_stable_known_match():
    """Special function: Verify unstable matches remain recognizing on the debug topic."""
    known = {
        'profile_id': 'guest_1',
        'name': 'Morgan',
        'role': 'guest',
        'favorite_drink': 'orange juice',
        'age': 25,
        'match_score': 0.72,
        'is_known': True,
    }
    import json

    unstable = json.loads(recognized_identity_payload(known, False, True))
    stable = json.loads(recognized_identity_payload(known, True, True))
    assert unstable['state'] == 'recognizing'
    assert unstable['human_id'] == 'unknown'
    assert unstable['profile_id'] == ''
    assert unstable['name'] == ''
    assert unstable['role'] == ''
    assert unstable['favorite_drink'] == ''
    assert unstable['age'] == 0
    assert stable['state'] == 'known'
    assert stable['human_id'] == 'guest_1'
    assert stable['name'] == 'Morgan'
    assert stable['favorite_drink'] == 'orange juice'
    assert stable['age'] == 25


def test_recognized_identity_payload_reports_no_face():
    """Special function: Verify absent face evidence is explicit on the identity topic."""
    import json

    payload = json.loads(recognized_identity_payload(unknown_match(), False, False))
    assert payload['state'] == 'no_face'
    assert payload['human_id'] == 'unknown'
    assert payload['profile_id'] == ''
    assert payload['favorite_drink'] == ''
    assert payload['age'] == 0


# Section: Optional dialogue metadata persists without changing schema-v2 compatibility.
def test_identity_store_persists_and_recognizes_profile_metadata(tmp_path):
    """Special function: Verify drink and age survive reload and appear in recognition results."""
    store = _three_dimensional_store(tmp_path)
    store.register_profile(
        'alice',
        'Alice',
        'guest',
        [1.0, 0.0, 0.0],
        favorite_drink='orange juice',
        age=25,
    )
    reloaded = _three_dimensional_store(tmp_path)
    profile = reloaded.get_profile('alice')
    result = reloaded.recognize_profile([1.0, 0.0, 0.0], threshold=0.45, margin=0.05)
    assert profile['favorite_drink'] == 'orange juice'
    assert profile['age'] == 25
    assert result['favorite_drink'] == 'orange juice'
    assert result['age'] == 25


def test_legacy_schema_v2_profile_metadata_defaults(tmp_path):
    """Special function: Verify schema-v2 profiles without optional metadata still load safely."""
    store = _three_dimensional_store(tmp_path)
    store.register_profile('alice', 'Alice', 'guest', [1.0, 0.0, 0.0])
    with open(store.db_path, 'rb') as file_obj:
        database = pickle.load(file_obj)
    database['profiles']['alice'].pop('favorite_drink', None)
    database['profiles']['alice'].pop('age', None)
    with open(store.db_path, 'wb') as file_obj:
        pickle.dump(database, file_obj)
    reloaded = _three_dimensional_store(tmp_path)
    profile = reloaded.get_profile('alice')
    assert profile['favorite_drink'] == ''
    assert profile['age'] == 0


def test_find_profile_by_name_is_case_insensitive_and_returns_copy(tmp_path):
    """Special function: Name lookup resolves usable profile data without exposing mutable storage."""
    # Section: Register one valid profile with persisted face templates.
    store = _three_dimensional_store(tmp_path)
    store.register_profile('guest_alex', 'Alex', 'guest', [1.0, 0.0, 0.0])

    profile = store.find_profile_by_name('  aLeX  ')

    assert profile['profile_id'] == 'guest_alex'
    assert len(profile['templates']) == 1
    # Section: Mutating the response cannot corrupt the registry cache.
    profile['name'] = 'Changed'
    assert store.find_profile_by_name('alex')['name'] == 'Alex'


def test_find_profile_by_name_rejects_missing_or_empty_name(tmp_path):
    """Special function: Missing names never resolve to an arbitrary stored identity."""
    store = _three_dimensional_store(tmp_path)
    store.register_profile('guest_alex', 'Alex', 'guest', [1.0, 0.0, 0.0])

    assert store.find_profile_by_name('') is None
    assert store.find_profile_by_name('Taylor') is None


# Section: Atomic persistence and metadata checks protect the runtime database.
def test_database_atomic_save_load(tmp_path, monkeypatch):
    """Special function: Verify successful saves load and failed replacement preserves old data."""
    store = _three_dimensional_store(tmp_path)
    store.register_profile('alice', 'Alice', 'guest', [1.0, 0.0, 0.0])
    reloaded = _three_dimensional_store(tmp_path)
    assert reloaded.get_profile('alice')['name'] == 'Alice'

    def fail_replace(_source, _destination):
        """Special function: Simulate interruption immediately before atomic replacement."""
        raise OSError('simulated replacement failure')

    monkeypatch.setattr('face_detection.identity_store.os.replace', fail_replace)
    reloaded.database['profiles']['alice']['name'] = 'Corrupted'
    assert not reloaded.save_database()
    preserved = _three_dimensional_store(tmp_path)
    assert preserved.get_profile('alice')['name'] == 'Alice'


def test_database_model_mismatch(tmp_path):
    """Special function: Verify embeddings from a different model are never loaded for matching."""
    database_path = tmp_path / 'identities.pkl'
    raw_database = {
        'schema_version': 2,
        'embedding_backend': 'opencv_sface',
        'embedding_model': 'buffalo_l',
        'embedding_dimension': 3,
        'profiles': {},
    }
    with database_path.open('wb') as file_obj:
        pickle.dump(raw_database, file_obj)
    store = IdentityStore(
        database_path,
        embedding_model='buffalo_sc',
        embedding_dimension=3,
    )
    assert not store.compatible
    assert not store.recognize_profile([1.0, 0.0, 0.0])['is_known']


def test_database_backend_mismatch(tmp_path):
    """Special function: Verify legacy InsightFace embeddings are rejected explicitly."""
    database_path = tmp_path / 'identities.pkl'
    raw_database = {
        'schema_version': 2,
        'embedding_backend': 'insightface',
        'embedding_model': 'buffalo_sc',
        'embedding_dimension': 3,
        'profiles': {},
    }
    with database_path.open('wb') as file_obj:
        pickle.dump(raw_database, file_obj)
    store = IdentityStore(
        database_path,
        embedding_backend='opencv_sface',
        embedding_model='face_recognition_sface_2021dec.onnx',
        embedding_dimension=3,
    )
    assert not store.compatible
    assert 'backend/model=insightface' in store.incompatibility_reason
