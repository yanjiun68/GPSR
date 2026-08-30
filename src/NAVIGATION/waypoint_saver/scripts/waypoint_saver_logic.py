#!/usr/bin/env python3
"""Section: Hardware-independent validation and YAML persistence for waypoint saving."""

# Section: Import standard-library support for safe copies, atomic files, and paths.
from __future__ import annotations

from copy import deepcopy
import os
from pathlib import Path
import tempfile
from typing import Callable, Mapping

# Section: Import YAML serialization without importing ROS runtime modules.
from yaml import safe_dump, safe_load


class WaypointValidationError(ValueError):
    """Special class: Report deterministic request, schema, or collision rejection."""


def resolve_workspace_root(
    package_share: str | Path,
    environment: Mapping[str, str] | None = None,
) -> Path:
    """Special function: Resolve the runtime workspace root with an environment override."""
    # Section: Read only the supplied environment during tests and the process environment at runtime.
    active_environment = os.environ if environment is None else environment
    override = str(active_environment.get("PUTRABOT_WORKSPACE_ROOT", "")).strip()
    if override:
        override_path = Path(override).expanduser()
        if not override_path.is_absolute():
            raise WaypointValidationError("PUTRABOT_WORKSPACE_ROOT must be an absolute path")
        return override_path.resolve(strict=False)

    # Section: Require the standard colcon install/<package>/share/<package> layout.
    share_path = Path(package_share).expanduser().resolve(strict=False)
    if (
        share_path.parent.name != "share"
        or share_path.parent.parent.name != share_path.name
        or share_path.parent.parent.parent.name != "install"
    ):
        raise WaypointValidationError(
            "Cannot derive workspace root from package share path; set PUTRABOT_WORKSPACE_ROOT"
        )
    return share_path.parent.parent.parent.parent


def resolve_yaml_path(raw_path: str) -> Path:
    """Special function: Validate and normalize the caller-selected waypoint YAML path."""
    # Section: Preserve the existing absolute-path contract and YAML-only extension policy.
    normalized_text = str(raw_path).strip()
    if not normalized_text:
        raise WaypointValidationError("yaml_path must not be empty")
    expanded_path = Path(normalized_text).expanduser()
    if not expanded_path.is_absolute():
        raise WaypointValidationError("yaml_path must be an absolute path")
    if expanded_path.suffix.lower() not in {".yaml", ".yml"}:
        raise WaypointValidationError("yaml_path must end in .yaml or .yml")
    return expanded_path.resolve(strict=False)


def validate_transform_age(
    now_sec: float,
    transform_stamp_sec: float,
    max_age_sec: float,
) -> float:
    """Special function: Validate a nonzero TF timestamp and return its nonnegative age."""
    # Section: Reject invalid configuration before comparing fabricated or live timestamps.
    now_value = float(now_sec)
    stamp_value = float(transform_stamp_sec)
    maximum_age = float(max_age_sec)
    if maximum_age < 0.0:
        raise WaypointValidationError("max_transform_age_sec must be non-negative")
    if stamp_value <= 0.0:
        raise WaypointValidationError("Transform timestamp is zero or uninitialized")

    # Section: Reject every future timestamp instead of silently accepting clock disagreement.
    age_sec = now_value - stamp_value
    if age_sec < 0.0:
        raise WaypointValidationError(
            f"Transform timestamp is {abs(age_sec):.3f}s in the future"
        )

    # Section: Accept the exact limit deterministically because freshness is age <= maximum age.
    if age_sec > maximum_age:
        raise WaypointValidationError(
            f"Transform is stale: age {age_sec:.3f}s exceeds {maximum_age:.3f}s"
        )
    return age_sec


def load_waypoint_data(path: Path) -> dict:
    """Special function: Load or initialize the repository-compatible waypoint-list schema."""
    # Section: Treat a missing file as a valid empty waypoint database.
    if not path.exists():
        return {"waypoints": []}
    if not path.is_file():
        raise WaypointValidationError(f"yaml_path is not a regular file: {path}")

    # Section: Parse YAML and require one top-level waypoint list.
    data = safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise WaypointValidationError("Waypoint YAML root must be a mapping")
    waypoints = data.setdefault("waypoints", [])
    if not isinstance(waypoints, list):
        raise WaypointValidationError("Waypoint YAML 'waypoints' field must be a list")
    return data


def _normalized_existing_waypoint(entry: object, index: int) -> tuple[str, int]:
    """Special function: Return a validated existing waypoint name and nonnegative integer ID."""
    # Section: Fail closed on malformed entries because overwrite selection must be deterministic.
    if not isinstance(entry, dict):
        raise WaypointValidationError(f"Waypoint entry {index} must be a mapping")
    name = str(entry.get("name", "")).strip()
    if not name:
        raise WaypointValidationError(f"Waypoint entry {index} has an empty name")
    raw_id = entry.get("id")
    if isinstance(raw_id, bool):
        raise WaypointValidationError(f"Waypoint entry {index} has an invalid id")
    try:
        waypoint_id = int(raw_id)
    except (TypeError, ValueError) as exc:
        raise WaypointValidationError(
            f"Waypoint entry {index} has a non-integer id"
        ) from exc
    if str(raw_id).strip() != str(waypoint_id) or waypoint_id < 0:
        raise WaypointValidationError(f"Waypoint entry {index} has an invalid id")
    return name, waypoint_id


def select_waypoint_slot(
    waypoints: list,
    name: str,
    requested_id: int,
    overwrite: bool,
) -> tuple[int, int | None]:
    """Special function: Select one append/replace slot under the collision-safe overwrite policy."""
    # Section: Validate the new identity before examining existing records.
    requested_name = str(name).strip()
    if not requested_name:
        raise WaypointValidationError("Waypoint name must not be empty")
    if int(requested_id) < -1:
        raise WaypointValidationError("Waypoint id must be -1 or non-negative")

    # Section: Index every existing identity and reject corrupted duplicate names or IDs.
    names_to_indexes: dict[str, list[int]] = {}
    ids_to_indexes: dict[int, list[int]] = {}
    existing_ids: set[int] = set()
    for index, entry in enumerate(waypoints):
        existing_name, existing_id = _normalized_existing_waypoint(entry, index)
        names_to_indexes.setdefault(existing_name, []).append(index)
        ids_to_indexes.setdefault(existing_id, []).append(index)
        existing_ids.add(existing_id)
    duplicate_names = sorted(name for name, indexes in names_to_indexes.items() if len(indexes) > 1)
    duplicate_ids = sorted(waypoint_id for waypoint_id, indexes in ids_to_indexes.items() if len(indexes) > 1)
    if duplicate_names:
        raise WaypointValidationError(
            f"Waypoint YAML contains duplicate names: {', '.join(duplicate_names)}"
        )
    if duplicate_ids:
        raise WaypointValidationError(
            "Waypoint YAML contains duplicate ids: "
            + ", ".join(str(waypoint_id) for waypoint_id in duplicate_ids)
        )

    # Section: Automatic ID overwrite preserves the unique existing name's ID.
    name_index = names_to_indexes.get(requested_name, [None])[0]
    if int(requested_id) == -1:
        if name_index is not None:
            if not overwrite:
                raise WaypointValidationError(
                    f"Waypoint name '{requested_name}' already exists; set overwrite=true to replace it"
                )
            preserved_id = int(waypoints[name_index]["id"])
            return preserved_id, name_index
        return max(existing_ids, default=-1) + 1, None

    # Section: Resolve explicit ID/name matches without ever deleting two distinct entries.
    assigned_id = int(requested_id)
    id_index = ids_to_indexes.get(assigned_id, [None])[0]
    if not overwrite:
        if name_index is not None or id_index is not None:
            raise WaypointValidationError(
                f"Waypoint name '{requested_name}' or id {assigned_id} already exists; "
                "set overwrite=true to replace it"
            )
        return assigned_id, None
    if name_index is not None and id_index is not None and name_index != id_index:
        raise WaypointValidationError(
            f"Ambiguous overwrite: name '{requested_name}' and id {assigned_id} "
            "identify different waypoints"
        )
    replacement_index = name_index if name_index is not None else id_index
    return assigned_id, replacement_index


def update_waypoint_data(
    data: dict,
    name: str,
    requested_id: int,
    overwrite: bool,
    entry_builder: Callable[[int, str], dict],
) -> tuple[dict, int]:
    """Special function: Return a copied YAML document containing one deterministic update."""
    # Section: Select the update without mutating the caller's document on rejection.
    original_waypoints = data.get("waypoints", [])
    if not isinstance(original_waypoints, list):
        raise WaypointValidationError("Waypoint YAML 'waypoints' field must be a list")
    waypoint_id, replacement_index = select_waypoint_slot(
        original_waypoints,
        name,
        requested_id,
        overwrite,
    )

    # Section: Apply exactly one replacement or append to a deep copy.
    updated_data = deepcopy(data)
    updated_waypoints = updated_data["waypoints"]
    new_entry = entry_builder(waypoint_id, str(name).strip())
    if replacement_index is None:
        updated_waypoints.append(new_entry)
    else:
        updated_waypoints[replacement_index] = new_entry
    return updated_data, waypoint_id


def atomic_write_waypoint_data(
    path: Path,
    data: dict,
    replace_function: Callable[[str | Path, str | Path], None] | None = None,
) -> None:
    """Special function: Flush a same-directory temporary YAML before atomic replacement."""
    # Section: Create missing runtime waypoint directories and serialize parseable YAML.
    path.parent.mkdir(parents=True, exist_ok=True)
    serialized = safe_dump(data, sort_keys=False, allow_unicode=True)
    temporary_path: Path | None = None
    active_replace = os.replace if replace_function is None else replace_function

    # Section: Preserve the temporary-file, flush, fsync, and os.replace strategy.
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            stream.write(serialized)
            stream.flush()
            os.fsync(stream.fileno())
        active_replace(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
