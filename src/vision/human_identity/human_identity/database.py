"""Section: SQLite persistence for profiles and visual prototypes."""

from __future__ import annotations

import io
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from threading import RLock
from typing import Iterator

import numpy as np

from .feature_types import PersonFeatures, VisualAttributes


class IdentityDatabase:
    """Section: Own the persistent person ID sequence and prototype records."""

    def __init__(self, database_path: str) -> None:
        """Special function: Open the database and create its schema transactionally."""
        self.database_path = database_path
        self._lock = RLock()
        if database_path != ":memory:":
            Path(database_path).expanduser().parent.mkdir(parents=True, exist_ok=True)
        self._connection = sqlite3.connect(database_path, check_same_thread=False)
        self._connection.row_factory = sqlite3.Row
        self._connection.execute("PRAGMA foreign_keys = ON")
        self._create_schema()

    def _create_schema(self) -> None:
        """Special function: Create profile and prototype tables when absent."""
        with self._lock, self._connection:
            self._connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS people (
                    person_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    canonical_person_uuid TEXT NOT NULL DEFAULT '',
                    person_name TEXT NOT NULL DEFAULT '',
                    age INTEGER,
                    favourite_drink TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS prototypes (
                    prototype_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    person_id INTEGER NOT NULL,
                    feature_blob BLOB NOT NULL,
                    created_at TEXT NOT NULL,
                    FOREIGN KEY(person_id)
                        REFERENCES people(person_id)
                        ON DELETE CASCADE
                );
                """
            )
            # Section: Existing SQLite databases gain the mapping column without data loss.
            columns = {
                str(row[1])
                for row in self._connection.execute("PRAGMA table_info(people)").fetchall()
            }
            if "canonical_person_uuid" not in columns:
                self._connection.execute(
                    "ALTER TABLE people ADD COLUMN canonical_person_uuid TEXT NOT NULL DEFAULT ''"
                )

    @staticmethod
    def _timestamp() -> str:
        """Special function: Produce a stable UTC timestamp for database records."""
        return datetime.now(timezone.utc).isoformat()

    def create_registration_profile(self, canonical_person_uuid: str = "") -> int:
        """Special function: Allocate the next SQLite AUTOINCREMENT person ID."""
        timestamp = self._timestamp()
        with self._lock, self._connection:
            cursor = self._connection.execute(
                "INSERT INTO people (canonical_person_uuid, created_at, updated_at) "
                "VALUES (?, ?, ?)",
                (str(canonical_person_uuid).strip(), timestamp, timestamp),
            )
            return int(cursor.lastrowid)

    def get_person_profile(self, person_id: int) -> dict | None:
        """Special function: Fetch a profile while preserving ID zero as unknown."""
        if person_id <= 0:
            return None
        with self._lock:
            row = self._connection.execute(
                "SELECT person_id, canonical_person_uuid, person_name, age, favourite_drink "
                "FROM people WHERE person_id = ?",
                (person_id,),
            ).fetchone()
        if row is None:
            return None
        return {
            "person_id": int(row["person_id"]),
            "canonical_person_uuid": str(row["canonical_person_uuid"]),
            "person_name": str(row["person_name"]),
            "age": int(row["age"] or 0),
            "favourite_drink": str(row["favourite_drink"]),
        }

    def bind_canonical_person(self, person_id: int, person_uuid: str) -> None:
        """Special function: Bind a body-identity record to a shared semantic person UUID."""
        normalized_uuid = str(person_uuid).strip()
        if person_id <= 0 or not normalized_uuid:
            raise ValueError("person_id and person_uuid must be non-empty")
        if not self.person_exists(person_id):
            raise KeyError(f"Person {person_id} does not exist")
        with self._lock, self._connection:
            self._connection.execute(
                "UPDATE people SET canonical_person_uuid = ?, updated_at = ? "
                "WHERE person_id = ?",
                (normalized_uuid, self._timestamp(), person_id),
            )

    def update_person_profile(
        self,
        person_id: int,
        person_name: str,
        age: int,
        favourite_drink: str,
        update_name: bool,
        update_age: bool,
        update_favourite_drink: bool,
    ) -> None:
        """Special function: Update only explicitly selected profile fields."""
        if person_id <= 0:
            raise ValueError("person_id=0 is reserved for unknown people")
        if not self.person_exists(person_id):
            raise KeyError(f"Person {person_id} does not exist")
        assignments: list[str] = []
        values: list[object] = []
        if update_name:
            assignments.append("person_name = ?")
            values.append(person_name)
        if update_age:
            assignments.append("age = ?")
            values.append(age)
        if update_favourite_drink:
            assignments.append("favourite_drink = ?")
            values.append(favourite_drink)
        if not assignments:
            return
        assignments.append("updated_at = ?")
        values.append(self._timestamp())
        values.append(person_id)
        with self._lock, self._connection:
            self._connection.execute(
                f"UPDATE people SET {', '.join(assignments)} WHERE person_id = ?",
                values,
            )

    def add_prototype(self, person_id: int, prototype: PersonFeatures) -> int:
        """Special function: Persist one structured visual prototype for an existing person."""
        if not self.person_exists(person_id):
            raise KeyError(f"Person {person_id} does not exist")
        feature_blob = self._serialize_prototype(prototype)
        with self._lock, self._connection:
            cursor = self._connection.execute(
                "INSERT INTO prototypes (person_id, feature_blob, created_at) VALUES (?, ?, ?)",
                (person_id, feature_blob, self._timestamp()),
            )
            return int(cursor.lastrowid)

    def iter_prototypes(self) -> Iterator[tuple[int, PersonFeatures]]:
        """Special function: Yield detached prototype records for identity matching."""
        with self._lock:
            rows = self._connection.execute(
                "SELECT person_id, feature_blob FROM prototypes ORDER BY prototype_id"
            ).fetchall()
        for row in rows:
            yield int(row["person_id"]), self._deserialize_prototype(row["feature_blob"])

    def person_exists(self, person_id: int) -> bool:
        """Special function: Check positive profile IDs without treating unknown as a record."""
        if person_id <= 0:
            return False
        with self._lock:
            row = self._connection.execute(
                "SELECT 1 FROM people WHERE person_id = ?",
                (person_id,),
            ).fetchone()
        return row is not None

    def close(self) -> None:
        """Special function: Close the SQLite connection explicitly."""
        with self._lock:
            self._connection.close()

    @staticmethod
    def _serialize_prototype(prototype: PersonFeatures) -> bytes:
        """Special function: Encode prototype arrays and attributes without pickle."""
        visual = prototype.visual_attributes
        stream = io.BytesIO()
        np.savez_compressed(
            stream,
            track_id=np.asarray([prototype.track_id], dtype=np.int32),
            body_vector=np.asarray(prototype.body_vector, dtype=np.float32),
            upper_histogram=np.asarray(prototype.upper_histogram, dtype=np.float32),
            lower_histogram=np.asarray(prototype.lower_histogram, dtype=np.float32),
            object_vector=np.asarray(prototype.object_vector, dtype=np.float32),
            qualities=np.asarray(
                [
                    prototype.body_quality,
                    prototype.upper_quality,
                    prototype.lower_quality,
                    prototype.object_quality,
                ],
                dtype=np.float32,
            ),
            attribute_states=np.asarray(
                [visual.wearing_glasses, visual.wearing_face_mask],
                dtype=np.int8,
            ),
            attribute_confidences=np.asarray(
                [
                    visual.glasses_confidence,
                    visual.shirt_colour_confidence,
                    visual.face_mask_confidence,
                    visual.hair_colour_confidence,
                    visual.overall_quality,
                ],
                dtype=np.float32,
            ),
            attribute_labels=np.asarray([visual.shirt_colour, visual.hair_colour], dtype="U64"),
        )
        return stream.getvalue()

    @staticmethod
    def _deserialize_prototype(feature_blob: bytes) -> PersonFeatures:
        """Special function: Decode a trusted local prototype using non-pickle NumPy arrays."""
        with np.load(io.BytesIO(feature_blob), allow_pickle=False) as values:
            qualities = values["qualities"]
            states = values["attribute_states"]
            confidences = values["attribute_confidences"]
            labels = values["attribute_labels"]
            visual = VisualAttributes(
                wearing_glasses=int(states[0]),
                glasses_confidence=float(confidences[0]),
                shirt_colour=str(labels[0]),
                shirt_colour_confidence=float(confidences[1]),
                wearing_face_mask=int(states[1]),
                face_mask_confidence=float(confidences[2]),
                hair_colour=str(labels[1]),
                hair_colour_confidence=float(confidences[3]),
                overall_quality=float(confidences[4]),
            )
            return PersonFeatures(
                track_id=int(values["track_id"][0]),
                body_vector=values["body_vector"].astype(np.float32),
                upper_histogram=values["upper_histogram"].astype(np.float32),
                lower_histogram=values["lower_histogram"].astype(np.float32),
                object_vector=values["object_vector"].astype(np.float32),
                body_quality=float(qualities[0]),
                upper_quality=float(qualities[1]),
                lower_quality=float(qualities[2]),
                object_quality=float(qualities[3]),
                visual_attributes=visual,
            )
