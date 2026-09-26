"""Section: GPSR service-owned YASMIN guest-memory policy tests."""

# Section: Test fixtures, deterministic fakes, and regression scenarios.

import json

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_ai_interfaces.msg import GuestAppearance
from robot_ai_interfaces.msg import FoundEntityRecord, GpsrGuestRecord

from robot_yasmin_tasks.ros_servers.gpsr_guest_memory_node import GpsrGuestMemoryStore


def make_store() -> GpsrGuestMemoryStore:
    """Special function: Build deterministic session memory without a ROS node."""
    return GpsrGuestMemoryStore(lambda: Time(sec=10), session_id="session_a")


def test_anonymous_identity_is_session_and_track_scoped() -> None:
    """Special function: Anonymous records cannot collide across tracking generations."""
    store = make_store()
    record = GpsrGuestRecord()
    record.tracking_session_id = "tracker_a"
    record.track_id = 7
    saved = store.save(record)
    assert saved.memory_id == "anonymous:tracker_a:7"
    assert store.get().memory_id == saved.memory_id


def test_profile_identity_merges_stronger_appearance_and_revision() -> None:
    """Special function: Repeated profile saves update one record conservatively."""
    store = make_store()
    first = GpsrGuestRecord()
    first.profile.profile_id = "guest_1"
    first.profile.name = "Alex"
    first.profile.appearance.glasses = GuestAppearance.YES
    first.profile.appearance.glasses_confidence = 0.8
    saved_first = store.save(first)
    second = GpsrGuestRecord()
    second.profile.profile_id = "guest_1"
    second.profile.favorite_drink = "tea"
    second.profile.appearance.glasses = GuestAppearance.UNKNOWN
    saved_second = store.save(second)
    assert saved_second.memory_id == "profile:guest_1"
    assert saved_second.revision == saved_first.revision + 1
    assert saved_second.profile.name == "Alex"
    assert saved_second.profile.favorite_drink == "tea"
    assert saved_second.profile.appearance.glasses == GuestAppearance.YES


def test_clear_retains_session_but_removes_records() -> None:
    """Special function: Explicit clear resets records without changing session identity."""
    store = make_store()
    store.save(GpsrGuestRecord())
    store.clear()
    assert store.session_id == "session_a"
    assert store.list_records() == []
    assert store.get() is None


def test_guest_memory_is_bounded_and_caps_face_embeddings() -> None:
    """Special function: Existing guest storage also stays within fixed RAM limits."""
    store = GpsrGuestMemoryStore(
        lambda: Time(sec=10),
        session_id="session_a",
        max_guest_records=2,
    )
    for profile_id in ("guest_1", "guest_2", "guest_3"):
        record = GpsrGuestRecord()
        record.profile.profile_id = profile_id
        record.profile.face_embedding = [0.1] * 1500
        saved = store.save(record)
        assert len(saved.profile.face_embedding) == 1024
    assert store.get("profile:guest_1") is None
    assert len(store.list_records()) == 2


def test_unknown_update_does_not_erase_known_guest_info() -> None:
    """Special function: UNKNOWN spellings preserve earlier known JSON fields."""
    store = make_store()
    first = GpsrGuestRecord()
    first.tracking_session_id = "tracking"
    first.track_id = 3
    first.guest_info_json = '{"name": "Alex", "pose": "standing"}'
    stored = store.save(first)
    update = GpsrGuestRecord()
    update.memory_id = stored.memory_id
    update.guest_info_json = '{"name": "UNKNOWN", "pose": "unknown"}'
    merged = store.save(update)
    assert json.loads(merged.guest_info_json)["name"] == "Alex"
    assert json.loads(merged.guest_info_json)["pose"] == "standing"


def test_weaker_and_older_observation_does_not_replace_evidence() -> None:
    """Special function: Preserve strong appearance and fresh pose across delayed retries."""
    store = make_store()
    first = GpsrGuestRecord()
    first.tracking_session_id = "tracking"
    first.track_id = 4
    first.person_ref_json = json.dumps({
        "stamp": {"sec": 20, "nanosec": 0},
        "appearance": {"shirt_color": "red", "shirt_color_confidence": 0.9},
        "pose": "standing",
        "pose_confidence": 0.8,
    })
    first.guest_info_json = json.dumps({
        "shirt_color": "red",
        "shirt_color_confidence": 0.9,
        "pose": "standing",
    })
    stored = store.save(first)
    delayed = GpsrGuestRecord()
    delayed.memory_id = stored.memory_id
    delayed.person_ref_json = json.dumps({
        "stamp": {"sec": 19, "nanosec": 0},
        "appearance": {"shirt_color": "blue", "shirt_color_confidence": 0.4},
        "pose": "sitting",
        "pose_confidence": 0.7,
    })
    delayed.guest_info_json = json.dumps({
        "shirt_color": "blue",
        "shirt_color_confidence": 0.4,
        "pose": "sitting",
    })
    merged = store.save(delayed)
    person_ref = json.loads(merged.person_ref_json)
    guest_info = json.loads(merged.guest_info_json)
    assert person_ref["appearance"]["shirt_color"] == "red"
    assert person_ref["pose"] == "standing"
    assert guest_info["shirt_color"] == "red"
    assert guest_info["pose"] == "standing"


def test_find_history_is_bounded_and_evicts_oldest_record() -> None:
    """Special function: Compact history never exceeds its configured RAM record limit."""
    store = GpsrGuestMemoryStore(
        lambda: Time(sec=10),
        session_id="session_a",
        max_find_records=2,
    )
    for target in ("name:apple", "name:banana", "name:orange"):
        record = FoundEntityRecord()
        record.entity_type = "object"
        record.target_key = target
        store.save_found(record)
    assert store.lookup_found("object", "name:apple") is None
    assert [record.target_key for record in store.list_found_records()] == [
        "name:banana",
        "name:orange",
    ]


def test_find_history_uses_latest_amcl_pose_for_person_observation() -> None:
    """Special function: Person finds store the latest map pose without a second pose node."""
    store = make_store()
    amcl = PoseWithCovarianceStamped()
    amcl.header.frame_id = "map"
    amcl.pose.pose.position.x = 1.25
    amcl.pose.pose.orientation.w = 1.0
    store.update_observation_pose(amcl)
    record = FoundEntityRecord()
    record.entity_type = "person"
    record.target_key = "name:alex"
    saved = store.save_found(record)
    assert saved.observation_pose.header.frame_id == "map"
    assert saved.observation_pose.pose.position.x == 1.25


def test_clear_releases_find_records_and_cached_pose() -> None:
    """Special function: Explicit shutdown clear leaves no retained record or AMCL value."""
    store = make_store()
    amcl = PoseWithCovarianceStamped()
    amcl.header.frame_id = "map"
    store.update_observation_pose(amcl)
    first = FoundEntityRecord()
    first.entity_type = "person"
    first.target_key = "name:alex"
    store.save_found(first)
    store.clear()
    second = FoundEntityRecord()
    second.entity_type = "person"
    second.target_key = "name:sam"
    saved = store.save_found(second)
    assert store.lookup_found("person", "name:alex") is None
    assert saved.observation_pose.header.frame_id == ""
