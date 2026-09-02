#!/usr/bin/env python3
"""Lightweight, model-safe identity storage and recognition helpers."""

import copy
import json
import os
import pickle
import tempfile
from collections import Counter, deque

import numpy as np


# Section: Database defaults bind persisted embeddings to the OpenCV SFace model space.
SCHEMA_VERSION = 2
DEFAULT_EMBEDDING_BACKEND = 'opencv_sface'
DEFAULT_EMBEDDING_MODEL = 'face_recognition_sface_2021dec.onnx'
# Section: Zero means that ROS consumers must read the actual dimension from the first SFace result.
DEFAULT_EMBEDDING_DIMENSION = 0
DEFAULT_TEMPLATE_COUNT = 5


def normalize_embedding(embedding):
    """Special function: Return a float32 unit vector without changing zero vectors."""
    emb = np.asarray(embedding, dtype=np.float32).reshape(-1)
    norm = float(np.linalg.norm(emb))
    if norm <= 1e-6:
        return emb
    return emb / norm


def cosine_similarity(a, b):
    """Special function: Compute cosine similarity and reject empty or incompatible vectors."""
    vector_a = np.asarray(a, dtype=np.float32).reshape(-1)
    vector_b = np.asarray(b, dtype=np.float32).reshape(-1)
    if vector_a.size == 0 or vector_a.shape != vector_b.shape:
        return -1.0
    denominator = float(np.linalg.norm(vector_a) * np.linalg.norm(vector_b))
    if denominator <= 1e-6:
        return -1.0
    return float(np.dot(vector_a, vector_b) / denominator)


def observation_stamp(face_data):
    """Special function: Convert JSON ROS stamp fields into a unique nanosecond value."""
    if not isinstance(face_data, dict):
        return None
    try:
        seconds = int(face_data.get('stamp_sec', 0))
        nanoseconds = int(face_data.get('stamp_nanosec', 0))
    except (TypeError, ValueError):
        return None
    if seconds == 0 and nanoseconds == 0:
        return None
    return seconds * 1_000_000_000 + nanoseconds


def observation_is_fresh(face_data, now_nanoseconds, max_face_age_sec):
    """Special function: Accept only stamped observations within the configured ROS-time age."""
    stamp_nanoseconds = observation_stamp(face_data)
    if stamp_nanoseconds is None:
        return False
    age_nanoseconds = int(now_nanoseconds) - stamp_nanoseconds
    maximum_age_nanoseconds = int(max(0.0, float(max_face_age_sec)) * 1_000_000_000)
    return 0 <= age_nanoseconds <= maximum_age_nanoseconds


def observation_is_new(face_data, last_accepted_stamp):
    """Special function: Reject missing or duplicate ROS observation timestamps."""
    current_stamp = observation_stamp(face_data)
    return current_stamp is not None and current_stamp != last_accepted_stamp


def select_representative_templates(embeddings, maximum_templates=DEFAULT_TEMPLATE_COUNT):
    """Special function: Keep normalized samples closest to the enrollment centroid."""
    normalized = [normalize_embedding(item) for item in embeddings]
    normalized = [item for item in normalized if item.size > 0 and np.linalg.norm(item) > 1e-6]
    if not normalized:
        return []
    dimensions = {int(item.size) for item in normalized}
    if len(dimensions) != 1:
        raise ValueError('Enrollment embeddings must all have the same dimension.')
    centroid = normalize_embedding(np.mean(np.stack(normalized), axis=0))
    ranked = sorted(normalized, key=lambda item: cosine_similarity(item, centroid), reverse=True)
    return [item.tolist() for item in ranked[:max(1, int(maximum_templates))]]


def unknown_match(top1_score=-1.0, top2_score=-1.0):
    """Special function: Build one consistent open-set UNKNOWN match result."""
    return {
        'profile_id': '',
        'name': '',
        'role': '',
        'match_score': float(top1_score),
        'top2_score': float(top2_score),
        'score_margin': float(top1_score - top2_score),
        'is_known': False,
    }


# Special function: Serialize the dedicated real-time identity topic payload.
def recognized_identity_payload(match, is_stable, face_detected, unknown_profile_id='unknown'):
    """Return stable identity state as compact JSON for GUI and lightweight ROS consumers."""
    # Section: Publish a known identity only after the temporal voter confirms it.
    known_and_stable = bool(is_stable and match.get('is_known', False))
    if known_and_stable:
        state = 'known'
        human_id = str(match.get('profile_id', '')).strip() or str(unknown_profile_id)
        name = str(match.get('name', '')).strip()
        role = str(match.get('role', '')).strip()
        favorite_drink = str(match.get('favorite_drink', '')).strip()
        try:
            age = int(match.get('age', 0))
        except (TypeError, ValueError):
            age = 0
    else:
        # Section: Distinguish active recognition from no-face and confirmed-unknown states.
        state = 'unknown' if is_stable else ('recognizing' if face_detected else 'no_face')
        human_id = str(unknown_profile_id)
        name = ''
        role = ''
        favorite_drink = ''
        age = 0
    payload = {
        'human_id': human_id,
        'profile_id': human_id if known_and_stable else '',
        'name': name,
        'role': role,
        'favorite_drink': favorite_drink,
        'age': age,
        'state': state,
        'is_known': known_and_stable,
        'is_stable': bool(is_stable),
        'face_detected': bool(face_detected),
        'match_score': float(match.get('match_score', -1.0)),
    }
    return json.dumps(payload, separators=(',', ':'))


class TemporalIdentityVoter:
    """Special function: Confirm identity only after repeated votes in a sliding window."""

    # Section: UNKNOWN votes intentionally participate so confidence collapse clears a stale identity.
    UNKNOWN_LABEL = '__unknown__'

    def __init__(self, window_size=5, minimum_votes=3):
        """Special function: Configure the bounded identity vote history."""
        self.window_size = max(1, int(window_size))
        self.minimum_votes = max(1, min(int(minimum_votes), self.window_size))
        self.history = deque(maxlen=self.window_size)
        self.stable_match = unknown_match()
        self.is_stable = False

    def reset(self):
        """Special function: Clear temporal evidence after a target or track change."""
        self.history.clear()
        self.stable_match = unknown_match()
        self.is_stable = False

    def add(self, match):
        """Special function: Add one frame vote and return the current stable decision."""
        label = str(match.get('profile_id', '')).strip() if match.get('is_known', False) else ''
        self.history.append((label or self.UNKNOWN_LABEL, copy.deepcopy(match)))
        counts = Counter(item[0] for item in self.history)
        winning_label, winning_votes = counts.most_common(1)[0]
        if winning_votes < self.minimum_votes:
            return copy.deepcopy(self.stable_match), self.is_stable
        if winning_label == self.UNKNOWN_LABEL:
            self.stable_match = unknown_match(
                top1_score=float(match.get('match_score', -1.0)),
                top2_score=float(match.get('top2_score', -1.0)),
            )
        else:
            candidates = [item[1] for item in self.history if item[0] == winning_label]
            self.stable_match = max(candidates, key=lambda item: float(item.get('match_score', -1.0)))
        self.is_stable = True
        return copy.deepcopy(self.stable_match), True


class IdentityStore:
    """Special function: Cache a model-versioned pickle database with atomic persistence."""

    def __init__(
        self,
        db_path,
        logger=None,
        embedding_backend=DEFAULT_EMBEDDING_BACKEND,
        embedding_model=DEFAULT_EMBEDDING_MODEL,
        embedding_dimension=DEFAULT_EMBEDDING_DIMENSION,
        maximum_templates=DEFAULT_TEMPLATE_COUNT,
    ):
        """Special function: Initialize metadata and load the database once."""
        self.db_path = str(db_path)
        self.logger = logger
        self.embedding_backend = str(embedding_backend).strip()
        self.embedding_model = str(embedding_model).strip()
        self.embedding_dimension = int(embedding_dimension)
        if not self.embedding_backend:
            raise ValueError('embedding_backend must not be empty')
        if not self.embedding_model:
            raise ValueError('embedding_model must not be empty')
        if self.embedding_dimension <= 0:
            raise ValueError('embedding_dimension must be read from SFace and be greater than zero')
        self.maximum_templates = max(1, int(maximum_templates))
        self.database = self._empty_database()
        self.compatible = True
        self.incompatibility_reason = ''
        self._loaded_mtime_ns = None
        db_dir = os.path.dirname(self.db_path)
        if db_dir:
            os.makedirs(db_dir, exist_ok=True)
        self.load_database()

    def _empty_database(self):
        """Special function: Build an empty database for the active embedding model."""
        return {
            'schema_version': SCHEMA_VERSION,
            'embedding_backend': self.embedding_backend,
            'embedding_model': self.embedding_model,
            'embedding_dimension': self.embedding_dimension,
            'profiles': {},
        }

    def _log_info(self, message):
        """Special function: Forward informational storage diagnostics to ROS when available."""
        if self.logger is not None:
            self.logger.info(message)

    def _log_warning(self, message):
        """Special function: Forward storage warnings to ROS when available."""
        if self.logger is not None:
            self.logger.warning(message)

    def _log_error(self, message):
        """Special function: Forward storage errors to ROS when available."""
        if self.logger is not None:
            self.logger.error(message)

    def _mark_incompatible(self, reason):
        """Special function: Disable comparison when persisted embeddings lack matching metadata."""
        self.compatible = False
        self.incompatibility_reason = str(reason)
        self.database = self._empty_database()
        self._log_error(
            f'Identity database is incompatible: {self.incompatibility_reason} '
            f'Remove or migrate {self.db_path} before enrollment or recognition.'
        )

    def _normalize_profile(self, profile_id, record):
        """Special function: Validate and normalize one schema-v2 identity profile."""
        if not isinstance(record, dict):
            return None
        normalized_profile_id = str(record.get('profile_id', profile_id)).strip()
        if not normalized_profile_id:
            return None
        centroid = normalize_embedding(record.get('centroid_embedding', []))
        templates = [normalize_embedding(item) for item in record.get('templates', [])]
        templates = [item for item in templates if item.size == self.embedding_dimension]
        if centroid.size != self.embedding_dimension or not templates:
            self._log_warning(f"Skipping invalid identity profile '{normalized_profile_id}'.")
            return None
        return {
            'profile_id': normalized_profile_id,
            'person_uuid': str(record.get('person_uuid', normalized_profile_id)).strip(),
            'name': str(record.get('name', normalized_profile_id)).strip(),
            'role': str(record.get('role', 'guest')).strip().lower() or 'guest',
            'favorite_drink': str(record.get('favorite_drink', '')).strip(),
            'age': self._normalize_age(record.get('age', 0)),
            'embedding_backend': self.embedding_backend,
            'embedding_model': self.embedding_model,
            'embedding_dimension': self.embedding_dimension,
            'centroid_embedding': centroid.tolist(),
            'templates': [item.tolist() for item in templates[:self.maximum_templates]],
            'shirt_color': str(record.get('shirt_color', '')).strip(),
            'attributes': copy.deepcopy(record.get('attributes', {})),
        }

    @staticmethod
    def _normalize_age(age):
        """Special function: Normalize persisted age while preserving legacy records."""
        # Section: Missing, malformed, and negative legacy values safely become unspecified age.
        try:
            normalized_age = int(age)
        except (TypeError, ValueError):
            return 0
        return normalized_age if normalized_age >= 0 else 0

    def load_database(self):
        """Special function: Load and validate the pickle once without accepting legacy embeddings."""
        self.compatible = True
        self.incompatibility_reason = ''
        if not os.path.exists(self.db_path):
            self.database = self._empty_database()
            self._loaded_mtime_ns = None
            return self.database
        try:
            with open(self.db_path, 'rb') as file_obj:
                raw_database = pickle.load(file_obj)
            self._loaded_mtime_ns = os.stat(self.db_path).st_mtime_ns
        except Exception as exc:
            self._mark_incompatible(f'failed to load database ({exc})')
            return self.database
        if not isinstance(raw_database, dict) or 'profiles' not in raw_database:
            self._mark_incompatible('legacy schema has no model metadata')
            return self.database
        try:
            schema_version = int(raw_database.get('schema_version', -1))
            stored_backend = str(raw_database.get('embedding_backend', '')).strip()
            stored_model = str(raw_database.get('embedding_model', '')).strip()
            stored_dimension = int(raw_database.get('embedding_dimension', -1))
        except (TypeError, ValueError) as exc:
            self._mark_incompatible(f'invalid database metadata ({exc})')
            return self.database
        if schema_version != SCHEMA_VERSION:
            self._mark_incompatible(
                f'schema_version={schema_version}, expected={SCHEMA_VERSION}'
            )
            return self.database
        if (
            stored_backend != self.embedding_backend
            or stored_model != self.embedding_model
            or stored_dimension != self.embedding_dimension
        ):
            self._mark_incompatible(
                f'backend/model={stored_backend}/{stored_model}/{stored_dimension}, '
                f'active={self.embedding_backend}/{self.embedding_model}/{self.embedding_dimension}'
            )
            return self.database
        profiles = {}
        for profile_id, record in dict(raw_database.get('profiles', {})).items():
            normalized = self._normalize_profile(profile_id, record)
            if normalized is not None:
                profiles[normalized['profile_id']] = normalized
        self.database = self._empty_database()
        self.database['profiles'] = profiles
        return self.database

    def refresh_if_changed(self):
        """Special function: Reload only when another process atomically replaces the database."""
        try:
            current_mtime_ns = os.stat(self.db_path).st_mtime_ns
        except FileNotFoundError:
            return False
        if current_mtime_ns == self._loaded_mtime_ns:
            return False
        self.load_database()
        return True

    def save_database(self):
        """Special function: Persist with flush, fsync, and atomic replacement."""
        if not self.compatible:
            self._log_error('Refusing to overwrite an incompatible identity database.')
            return False
        db_dir = os.path.dirname(self.db_path) or '.'
        temporary_path = ''
        try:
            with tempfile.NamedTemporaryFile(
                mode='wb',
                prefix='.identity-',
                suffix='.tmp',
                dir=db_dir,
                delete=False,
            ) as file_obj:
                temporary_path = file_obj.name
                pickle.dump(self.database, file_obj, protocol=pickle.HIGHEST_PROTOCOL)
                file_obj.flush()
                os.fsync(file_obj.fileno())
            os.replace(temporary_path, self.db_path)
            self._loaded_mtime_ns = os.stat(self.db_path).st_mtime_ns
            return True
        except Exception as exc:
            self._log_error(f'Failed to save identity database atomically: {exc}')
            if temporary_path:
                try:
                    os.unlink(temporary_path)
                except OSError:
                    pass
            return False

    def register_profile(
        self,
        profile_id,
        name,
        role,
        embedding,
        shirt_color='',
        attributes=None,
        templates=None,
        favorite_drink='',
        age=0,
        person_uuid='',
    ):
        """Special function: Store a centroid plus representative normalized templates."""
        if not self.compatible:
            raise RuntimeError(self.incompatibility_reason)
        normalized_profile_id = str(profile_id).strip()
        if not normalized_profile_id:
            raise ValueError('profile_id must not be empty')
        candidate_templates = list(templates if templates is not None else [embedding])
        representative_templates = select_representative_templates(
            candidate_templates,
            self.maximum_templates,
        )
        centroid = normalize_embedding(embedding)
        if centroid.size != self.embedding_dimension:
            raise ValueError(
                f'Expected {self.embedding_dimension}-D embedding, received {centroid.size}-D.'
            )
        if not representative_templates:
            raise ValueError('At least one valid template embedding is required.')
        if any(len(item) != self.embedding_dimension for item in representative_templates):
            raise ValueError(
                f'All templates must contain {self.embedding_dimension} embedding values.'
            )
        record = {
            'profile_id': normalized_profile_id,
            'person_uuid': str(person_uuid).strip() or normalized_profile_id,
            'name': str(name).strip(),
            'role': str(role).strip().lower() or 'guest',
            'favorite_drink': str(favorite_drink).strip(),
            'age': self._normalize_age(age),
            'embedding_backend': self.embedding_backend,
            'embedding_model': self.embedding_model,
            'embedding_dimension': self.embedding_dimension,
            'centroid_embedding': centroid.tolist(),
            'templates': representative_templates,
            'shirt_color': str(shirt_color).strip(),
            'attributes': copy.deepcopy(attributes or {}),
        }
        self.database['profiles'][normalized_profile_id] = record
        if not self.save_database():
            raise OSError(f'Failed to persist identity profile to {self.db_path}')
        self._log_info(f"Registered identity profile '{normalized_profile_id}'")
        return copy.deepcopy(record)

    def get_profile(self, profile_id):
        """Special function: Return a defensive copy of one cached profile."""
        record = self.database['profiles'].get(str(profile_id).strip())
        return copy.deepcopy(record) if record is not None else None

    def find_profile_by_name(self, name):
        """Special function: Resolve one usable profile by normalized human-readable name."""
        # Section: Case-folding provides deterministic Unicode-aware name comparison.
        normalized_name = str(name).strip().casefold()
        if not normalized_name:
            return None
        # Section: Stable profile-id ordering makes duplicate-name resolution deterministic.
        for profile_id in sorted(self.database['profiles']):
            record = self.database['profiles'][profile_id]
            if str(record.get('name', '')).strip().casefold() == normalized_name:
                return copy.deepcopy(record)
        return None

    def recognize_profile(self, embedding, threshold=0.45, margin=0.05):
        """Special function: Apply template matching plus threshold and top-two margin rejection."""
        if not self.compatible or not self.database['profiles']:
            return unknown_match()
        query = normalize_embedding(embedding)
        if query.size != self.embedding_dimension:
            self._log_error(
                f'Rejecting query embedding dimension {query.size}; '
                f'expected {self.embedding_dimension}.'
            )
            return unknown_match()
        ranked = []
        for record in self.database['profiles'].values():
            reference_embeddings = [record['centroid_embedding'], *record['templates']]
            score = max(cosine_similarity(query, reference) for reference in reference_embeddings)
            ranked.append((float(score), record))
        ranked.sort(key=lambda item: item[0], reverse=True)
        top1_score, top1_record = ranked[0]
        top2_score = ranked[1][0] if len(ranked) > 1 else -1.0
        if top1_score < float(threshold) or (top1_score - top2_score) < float(margin):
            return unknown_match(top1_score, top2_score)
        return {
            'profile_id': str(top1_record.get('profile_id', '')).strip(),
            'person_uuid': str(top1_record.get('person_uuid', '')).strip(),
            'name': str(top1_record.get('name', '')).strip(),
            'role': str(top1_record.get('role', '')).strip(),
            'favorite_drink': str(top1_record.get('favorite_drink', '')).strip(),
            'age': self._normalize_age(top1_record.get('age', 0)),
            'match_score': float(top1_score),
            'top2_score': float(top2_score),
            'score_margin': float(top1_score - top2_score),
            'is_known': True,
        }

    def reset_database(self):
        """Special function: Replace all identities with a fresh active-model database."""
        self.compatible = True
        self.incompatibility_reason = ''
        self.database = self._empty_database()
        if not self.save_database():
            raise OSError(f'Failed to reset identity database at {self.db_path}')
        self._log_info('Identity database reset.')
