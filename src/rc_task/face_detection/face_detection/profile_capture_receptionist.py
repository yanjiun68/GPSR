#!/usr/bin/env python3
import copy
import json
import time
from collections import deque
from uuid import NAMESPACE_URL, uuid5

import numpy as np
import rclpy
from face_detection.face_backend import EMBEDDING_BACKEND
from face_detection.identity_store import (
    DEFAULT_EMBEDDING_BACKEND,
    DEFAULT_EMBEDDING_DIMENSION,
    DEFAULT_EMBEDDING_MODEL,
    IdentityStore,
    cosine_similarity,
    normalize_embedding,
    observation_is_fresh,
    observation_is_new,
    observation_stamp,
)
from face_detection.paths import default_guest_db_path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_ai_interfaces.action import CaptureNamedProfile
from robot_ai_interfaces.msg import CurrentGuestStatus, GuestAppearance, PersonProfile
from robot_ai_interfaces.srv import (
    CaptureAppearance,
    CommitCurrentGuest,
    SetCurrentGuestProfile,
    SetHostProfile,
)
from std_msgs.msg import String


def bbox_center_and_area(bbox):
    """Special function: Convert a face box into stability-comparison geometry."""
    x1, y1, x2, y2 = [float(value) for value in bbox]
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    return center_x, center_y, area


def bbox_is_stable(bbox, last_center_area, center_tol, area_ratio_tol):
    """Special function: Reject gross face motion or scale change during enrollment."""
    center_x, center_y, area = bbox_center_and_area(bbox)
    if last_center_area is None:
        return True, (center_x, center_y, area)

    last_center_x, last_center_y, last_area = last_center_area
    center_distance = ((center_x - last_center_x) ** 2 + (center_y - last_center_y) ** 2) ** 0.5

    if last_area <= 1e-6:
        area_ratio_diff = 0.0
    else:
        area_ratio_diff = abs(area - last_area) / last_area

    stable = center_distance <= center_tol and area_ratio_diff <= area_ratio_tol
    return stable, (center_x, center_y, area)


def canonical_person_uuid(profile_id, requested_uuid=""):
    """Special function: Reuse a requested UUID or derive a stable profile-scoped UUID."""
    explicit = str(requested_uuid or "").strip()
    return explicit or str(uuid5(NAMESPACE_URL, f"putrabot-person:{str(profile_id).casefold()}"))


def select_locked_face(face_payload, tracking_session_id, track_id):
    """Special function: Select exactly one face associated with the requested canonical track."""
    if not isinstance(face_payload, dict):
        return None
    requested_session = str(tracking_session_id).strip()
    requested_track = int(track_id)
    matches = []
    for face in list(face_payload.get("faces", [])):
        if not isinstance(face, dict):
            continue
        if (
            str(face.get("tracking_session_id", "")).strip() == requested_session
            and int(face.get("track_id", 0) or 0) == requested_track
            and bool(face.get("has_track_id", False))
        ):
            candidate = copy.deepcopy(face)
            candidate["stamp_sec"] = int(face_payload.get("stamp_sec", 0))
            candidate["stamp_nanosec"] = int(face_payload.get("stamp_nanosec", 0))
            candidate["detected"] = True
            matches.append(candidate)
    # Section: Multiple faces for one track are ambiguous and therefore rejected.
    return matches[0] if len(matches) == 1 else None


def locked_track_timed_out(tracking_session_id, last_seen, now, timeout_sec):
    """Special function: Decide when an active exact-track enrollment must abort."""
    return bool(str(tracking_session_id).strip()) and (
        float(now) - float(last_seen) >= float(timeout_sec)
    )


class ProfileCaptureNode(Node):
    """Special function: Enroll named profiles from distinct, fresh, consistent face frames."""

    def __init__(self):
        """Special function: Configure enrollment policy and compatible persistent storage."""
        super().__init__("profile_capture_node")

        self.declare_parameter("face_topic", "/perception/largest_face")
        self.declare_parameter("appearance_topic", "/appearance/current_estimate")
        self.declare_parameter("guest_profile_topic", "/dialogue/current_guest_profile")
        self.declare_parameter("host_profile_topic", "/dialogue/current_host_profile")
        self.declare_parameter("status_topic", "/appearance/current_capture_status")
        self.declare_parameter("snapshot_service_name", "/appearance/capture_profile_snapshot")
        self.declare_parameter("snapshot_json_topic", "/appearance/captured_profile_json")
        self.declare_parameter("action_name", "/appearance/capture_named_profile")
        self.declare_parameter("db_path", default_guest_db_path())
        # Section: Multi-frame enrollment uses bounded fresh observations and a hard timeout.
        self.declare_parameter("min_enrollment_samples", 5)
        self.declare_parameter("max_enrollment_samples", 10)
        self.declare_parameter("min_stable_samples", 0)
        self.declare_parameter("max_face_age_sec", 0.5)
        self.declare_parameter("profile_capture_timeout_sec", 8.0)
        self.declare_parameter("det_score_threshold", 0.8)
        self.declare_parameter("embedding_backend", DEFAULT_EMBEDDING_BACKEND)
        self.declare_parameter("embedding_model", DEFAULT_EMBEDDING_MODEL)
        self.declare_parameter("embedding_dimension", DEFAULT_EMBEDDING_DIMENSION)
        self.declare_parameter("embedding_similarity_threshold", 0.55)
        self.declare_parameter("center_tolerance_px", 40.0)
        self.declare_parameter("area_ratio_tolerance", 0.35)
        self.declare_parameter("feedback_period_sec", 0.5)
        self.declare_parameter("track_loss_timeout_sec", 2.0)
 # get parameter and convert into suitable format , e.g. str , float . 
        self.face_topic = str(self.get_parameter("face_topic").value)
        self.appearance_topic = str(self.get_parameter("appearance_topic").value)
        self.guest_profile_topic = str(self.get_parameter("guest_profile_topic").value)
        self.host_profile_topic = str(self.get_parameter("host_profile_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.snapshot_service_name = str(self.get_parameter("snapshot_service_name").value)
        self.snapshot_json_topic = str(self.get_parameter("snapshot_json_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.db_path = str(self.get_parameter("db_path").value)
        configured_minimum = int(self.get_parameter("min_enrollment_samples").value)
        legacy_minimum = int(self.get_parameter("min_stable_samples").value)
        self.min_enrollment_samples = max(1, legacy_minimum or configured_minimum)
        self.max_enrollment_samples = max(
            self.min_enrollment_samples,
            int(self.get_parameter("max_enrollment_samples").value),
        )
        self.max_face_age_sec = max(0.0, float(self.get_parameter("max_face_age_sec").value))
        self.profile_capture_timeout_sec = max(
            0.1,
            float(self.get_parameter("profile_capture_timeout_sec").value),
        )
        self.det_score_threshold = float(self.get_parameter("det_score_threshold").value)
        self.embedding_backend = str(self.get_parameter("embedding_backend").value).strip()
        self.embedding_model = str(self.get_parameter("embedding_model").value).strip()
        self.embedding_dimension = int(self.get_parameter("embedding_dimension").value)
        self.embedding_similarity_threshold = float(
            self.get_parameter("embedding_similarity_threshold").value
        )
        self.center_tolerance_px = float(self.get_parameter("center_tolerance_px").value)
        self.area_ratio_tolerance = float(self.get_parameter("area_ratio_tolerance").value)
        self.feedback_period_sec = max(
            0.05, float(self.get_parameter("feedback_period_sec").value)
        )
        self.track_loss_timeout_sec = max(
            0.1, float(self.get_parameter("track_loss_timeout_sec").value)
        )

        self.latest_face = None
        self.latest_appearance = None
        self.active_goal_handle = None
        self.target_profile_id = ""
        self.target_role = "guest"
        self.target_name = ""
        self.target_favorite_drink = ""
        self.target_age = 0
        self.target_person_uuid = ""
        self.target_tracking_session_id = ""
        self.target_track_id = 0
        self.last_target_seen_monotonic = 0.0
        self.last_center_area = None
        self.last_accepted_face_stamp = None
        self.last_selected_track_id = None
        self.last_target_seen_monotonic = time.monotonic()
        self.embedding_samples = deque(maxlen=self.max_enrollment_samples)
        self.appearance_snapshot = self._make_unknown_appearance()
        # Section: SFace feature length is discovered from actual model output, never assumed.
        self.identity_store = None
        if self.embedding_dimension > 0:
            self._ensure_identity_store(self.embedding_dimension)

        # Section: Reentrant callbacks let fresh face messages arrive while an action waits.
        self.callback_group = ReentrantCallbackGroup()
        self.face_sub = self.create_subscription(
            String,
            self.face_topic,
            self.face_callback,
            10,
            callback_group=self.callback_group,
        )
        self.appearance_sub = self.create_subscription(
            GuestAppearance,
            self.appearance_topic,
            self.appearance_callback,
            10,
            callback_group=self.callback_group,
        )
        self.guest_profile_pub = self.create_publisher(PersonProfile, self.guest_profile_topic, 10)
        self.host_profile_pub = self.create_publisher(PersonProfile, self.host_profile_topic, 10)
        self.status_pub = self.create_publisher(CurrentGuestStatus, self.status_topic, 10)
        self.snapshot_json_pub = self.create_publisher(String, self.snapshot_json_topic, 10)
        self.set_current_guest_client = self.create_client(
            SetCurrentGuestProfile,
            "/guest_memory/set_current_guest_profile",
        )
        self.set_host_profile_client = self.create_client(
            SetHostProfile,
            "/guest_memory/set_host_profile",
        )
        self.commit_current_guest_client = self.create_client(
            CommitCurrentGuest,
            "/guest_memory/commit_current_guest",
        )
        self.snapshot_service = self.create_service(
            CaptureAppearance,
            self.snapshot_service_name,
            self.capture_snapshot_cb,
        )

        self.action_server = ActionServer(
            self,
            CaptureNamedProfile,
            self.action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            handle_accepted_callback=self.handle_accepted_cb,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "Profile capture action ready. "
            f"action_name={self.action_name}, face_topic={self.face_topic}, "
            f"appearance_topic={self.appearance_topic}, status_topic={self.status_topic}, "
            f"snapshot_service_name={self.snapshot_service_name}, "
            f"snapshot_json_topic={self.snapshot_json_topic}, db_path={self.db_path}"
        )
        self._publish_status("idle", 0.0, 0.0, "")

    def _make_unknown_appearance(self):
        appearance = GuestAppearance()
        appearance.glasses = GuestAppearance.UNKNOWN
        appearance.hat = GuestAppearance.UNKNOWN
        appearance.face_mask = GuestAppearance.UNKNOWN
        appearance.shirt_color = ""
        appearance.glasses_confidence = 0.0
        appearance.hat_confidence = 0.0
        appearance.face_mask_confidence = 0.0
        appearance.shirt_color_confidence = 0.0
        appearance.stamp = self.get_clock().now().to_msg()
        return appearance

    def _reset_capture_state(self):
        """Special function: Reset all evidence so samples never leak between goals."""
        self.last_center_area = None
        self.last_accepted_face_stamp = None
        self.last_selected_track_id = None
        self.embedding_samples.clear()
        self.appearance_snapshot = self._make_unknown_appearance()

    def _ensure_identity_store(self, embedding_dimension):
        """Special function: Create the model-safe store after reading SFace feature length."""
        actual_dimension = int(embedding_dimension)
        if actual_dimension <= 0:
            raise ValueError('SFace embedding_dimension must be greater than zero.')
        if self.embedding_dimension > 0 and self.embedding_dimension != actual_dimension:
            raise ValueError(
                f'Configured embedding dimension {self.embedding_dimension} does not match '
                f'SFace output {actual_dimension}.'
            )
        if self.identity_store is None:
            self.embedding_dimension = actual_dimension
            self.identity_store = IdentityStore(
                self.db_path,
                logger=self.get_logger(),
                embedding_backend=self.embedding_backend,
                embedding_model=self.embedding_model,
                embedding_dimension=self.embedding_dimension,
            )
        return self.identity_store

    def face_callback(self, msg):
        """Special function: Cache the latest serialized face observation."""
        try:
            self.latest_face = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Failed to parse face message JSON: {exc}")
            self.latest_face = None

    def appearance_callback(self, msg):
        self.latest_appearance = copy.deepcopy(msg)

    def goal_cb(self, goal_request):
        if self.active_goal_handle is not None and self.active_goal_handle.is_active:
            self.get_logger().warn("Profile capture already active; rejecting overlapping goal.")
            return GoalResponse.REJECT

        profile_id = goal_request.profile_id.strip()
        if not profile_id:
            self.get_logger().warn("Rejecting profile capture goal without profile_id.")
            return GoalResponse.REJECT

        role = goal_request.role.strip().lower() or "guest"
        if role not in ("guest", "host"):
            self.get_logger().warn(f"Rejecting profile capture goal with unsupported role '{role}'.")
            return GoalResponse.REJECT
        session_id = str(getattr(goal_request, "tracking_session_id", "")).strip()
        track_id = int(getattr(goal_request, "track_id", 0) or 0)
        if bool(session_id) != bool(track_id > 0):
            self.get_logger().warn(
                "Rejecting profile capture goal with an incomplete session/track lock."
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Accepted profile capture goal for profile_id={profile_id}, role={role}, name={goal_request.name.strip() or 'unknown'}."
        )
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle):
        self.get_logger().info("Profile capture cancel requested.")
        return CancelResponse.ACCEPT

    def handle_accepted_cb(self, goal_handle):
        """Special function: Capture identity and metadata from the action goal."""
        # Section: getattr preserves compatibility until the external action gains metadata fields.
        self.active_goal_handle = goal_handle
        self.target_profile_id = goal_handle.request.profile_id.strip()
        self.target_role = goal_handle.request.role.strip().lower() or "guest"
        self.target_name = goal_handle.request.name.strip()
        self.target_person_uuid = canonical_person_uuid(
            self.target_profile_id,
            getattr(goal_handle.request, "person_uuid", ""),
        )
        self.target_tracking_session_id = str(
            getattr(goal_handle.request, "tracking_session_id", "")
        ).strip()
        self.target_track_id = int(getattr(goal_handle.request, "track_id", 0) or 0)
        # Section: Give the requested track one configured grace period to reappear.
        self.last_target_seen_monotonic = time.monotonic()
        self.target_favorite_drink = str(
            getattr(goal_handle.request, "favorite_drink", "")
        ).strip()
        try:
            self.target_age = int(getattr(goal_handle.request, "age", 0))
        except (TypeError, ValueError):
            self.target_age = 0
        self._reset_capture_state()
        self._publish_status("capturing", 0.0, 0.0, self.target_profile_id)
        goal_handle.execute()

    def _merge_binary_attribute(self, current_value, current_confidence, incoming_value, incoming_confidence):
        if incoming_value == GuestAppearance.UNKNOWN:
            return current_value, current_confidence
        if current_value == GuestAppearance.UNKNOWN or incoming_confidence >= current_confidence:
            return incoming_value, incoming_confidence
        return current_value, current_confidence

    def _merge_appearance(self, msg):
        (
            self.appearance_snapshot.glasses,
            self.appearance_snapshot.glasses_confidence,
        ) = self._merge_binary_attribute(
            self.appearance_snapshot.glasses,
            self.appearance_snapshot.glasses_confidence,
            msg.glasses,
            msg.glasses_confidence,
        )
        self.appearance_snapshot.hat, self.appearance_snapshot.hat_confidence = (
            self._merge_binary_attribute(
                self.appearance_snapshot.hat,
                self.appearance_snapshot.hat_confidence,
                msg.hat,
                msg.hat_confidence,
            )
        )
        (
            self.appearance_snapshot.face_mask,
            self.appearance_snapshot.face_mask_confidence,
        ) = self._merge_binary_attribute(
            self.appearance_snapshot.face_mask,
            self.appearance_snapshot.face_mask_confidence,
            msg.face_mask,
            msg.face_mask_confidence,
        )

        if (
            msg.shirt_color
            and msg.shirt_color_confidence >= self.appearance_snapshot.shirt_color_confidence
        ):
            self.appearance_snapshot.shirt_color = msg.shirt_color
            self.appearance_snapshot.shirt_color_confidence = msg.shirt_color_confidence

        self.appearance_snapshot.stamp = msg.stamp

    def _process_latest_observations(self):
        """Special function: Accept one new, fresh, high-quality enrollment observation."""
        if self.latest_face is None or self.latest_appearance is None:
            return False
        face_data = self.latest_face
        if self.target_tracking_session_id and self.target_track_id > 0:
            face_data = select_locked_face(
                self.latest_face,
                self.target_tracking_session_id,
                self.target_track_id,
            )
            if face_data is None:
                return False
            self.last_target_seen_monotonic = time.monotonic()
        if not bool(face_data.get("detected", False)):
            return False

        current_stamp = observation_stamp(face_data)
        if not observation_is_new(face_data, self.last_accepted_face_stamp):
            return False
        if not observation_is_fresh(
            face_data,
            self.get_clock().now().nanoseconds,
            self.max_face_age_sec,
        ):
            return False
        if float(face_data.get("det_score", 0.0)) < self.det_score_threshold:
            return False
        incoming_backend = str(face_data.get("embedding_backend", '')).strip()
        incoming_model = str(face_data.get("embedding_model", '')).strip()
        if incoming_backend != self.embedding_backend or incoming_backend != EMBEDDING_BACKEND:
            self.get_logger().error(
                f"Rejected enrollment backend '{incoming_backend}'; expected '{self.embedding_backend}'."
            )
            return False
        if incoming_model != self.embedding_model:
            self.get_logger().error(
                f"Rejected enrollment model '{incoming_model}'; expected '{self.embedding_model}'."
            )
            return False

        bbox = face_data.get("bbox", [])
        embedding = face_data.get("embedding", [])
        incoming_dimension = int(face_data.get('embedding_dimension', len(embedding)))
        if len(bbox) != 4 or len(embedding) != incoming_dimension or incoming_dimension <= 0:
            return False
        try:
            self._ensure_identity_store(incoming_dimension)
        except ValueError as exc:
            self.get_logger().error(f'Rejected enrollment embedding metadata: {exc}')
            return False

        # Section: A track change invalidates all accumulated enrollment evidence.
        has_track_id = bool(face_data.get('has_track_id', False))
        selected_track_id = int(face_data.get('track_id', 0)) if has_track_id else None
        if (
            selected_track_id is not None
            and self.last_selected_track_id is not None
            and selected_track_id != self.last_selected_track_id
        ):
            self._reset_capture_state()
        self.last_selected_track_id = selected_track_id

        stable_bbox, current_center_area = bbox_is_stable(
            bbox,
            self.last_center_area,
            self.center_tolerance_px,
            self.area_ratio_tolerance,
        )
        self.last_center_area = current_center_area

        normalized_embedding = normalize_embedding(embedding)
        same_identity = True
        if self.embedding_samples:
            average_embedding = normalize_embedding(np.mean(np.array(self.embedding_samples), axis=0))
            same_identity = (
                cosine_similarity(normalized_embedding, average_embedding)
                >= self.embedding_similarity_threshold
            )

        if not stable_bbox or not same_identity:
            self._reset_capture_state()
            self.embedding_samples.append(normalized_embedding)
            self.last_accepted_face_stamp = current_stamp
            self._merge_appearance(self.latest_appearance)
            return False

        self.embedding_samples.append(normalized_embedding)
        self.last_accepted_face_stamp = current_stamp
        self._merge_appearance(self.latest_appearance)
        return len(self.embedding_samples) >= self.min_enrollment_samples

    def _build_profile(self):
        """Special function: Build the captured ROS profile including optional person metadata."""
        # Section: PersonProfile exposes metadata even when the current action goal does not.
        profile = PersonProfile()
        profile.profile_id = self.target_profile_id
        profile.role = self.target_role
        profile.name = self.target_name
        profile.favorite_drink = self.target_favorite_drink
        profile.age = self.target_age
        profile.appearance = copy.deepcopy(self.appearance_snapshot)
        profile.confidence_score = 1.0
        profile.is_confirmed = bool(profile.name)
        profile.stamp = self.get_clock().now().to_msg()
        profile.appearance.stamp = profile.stamp
        return profile

    def _build_snapshot_profile(self, profile_id, role, name):
        profile = PersonProfile()
        profile.profile_id = profile_id
        profile.role = role
        profile.name = name
        profile.appearance = copy.deepcopy(self.latest_appearance or self._make_unknown_appearance())
        profile.confidence_score = 1.0 if self.latest_face_detected() else 0.0
        profile.is_confirmed = bool(profile.name or self._appearance_has_signal(profile.appearance))
        profile.stamp = self.get_clock().now().to_msg()
        profile.appearance.stamp = profile.stamp
        return profile

    def _appearance_has_signal(self, appearance):
        return bool(
            appearance.shirt_color
            or appearance.glasses != GuestAppearance.UNKNOWN
            or appearance.hat != GuestAppearance.UNKNOWN
            or appearance.face_mask != GuestAppearance.UNKNOWN
        )

    def latest_face_detected(self):
        return bool(self.latest_face is not None and self.latest_face.get("detected", False))

    def _appearance_flag_to_bool(self, value):
        return bool(value == GuestAppearance.YES)

    def _profile_json_payload(self, profile):
        appearance = profile.appearance
        return {
            "guest_id": profile.profile_id,
            "profile_id": profile.profile_id,
            "role": profile.role,
            "name": profile.name,
            "t_shirt_color": appearance.shirt_color or "unknown",
            "shirt_color": appearance.shirt_color or "unknown",
            "shirt_color_confidence": float(appearance.shirt_color_confidence),
            "wearing_glasses": self._appearance_flag_to_bool(appearance.glasses),
            "wearing_glasses_confidence": float(appearance.glasses_confidence),
            "wearing_mask": self._appearance_flag_to_bool(appearance.face_mask),
            "wearing_mask_confidence": float(appearance.face_mask_confidence),
            "wearing_hat": self._appearance_flag_to_bool(appearance.hat),
            "wearing_hat_confidence": float(appearance.hat_confidence),
            "face_detected": self.latest_face_detected(),
        }

    def _publish_profile_json(self, profile):
        payload = self._profile_json_payload(profile)
        msg = String()
        msg.data = json.dumps(payload)
        self.snapshot_json_pub.publish(msg)
        return msg.data

    def _wait_for_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.02)
        return None

    def _store_profile_in_memory(self, profile, commit_current_guest, commit_profile_id):
        role = profile.role.strip().lower()
        if role == "host":
            if not self.set_host_profile_client.wait_for_service(timeout_sec=1.0):
                return False, "Host memory service is unavailable."
            request = SetHostProfile.Request()
            request.profile = profile
            response = self._wait_for_future(self.set_host_profile_client.call_async(request), 2.0)
            if not response or not response.success:
                return False, response.message if response else "Host memory write timed out."
            return True, response.message

        if not self.set_current_guest_client.wait_for_service(timeout_sec=1.0):
            return False, "Current-guest memory service is unavailable."
        request = SetCurrentGuestProfile.Request()
        request.profile = profile
        response = self._wait_for_future(self.set_current_guest_client.call_async(request), 2.0)
        if not response or not response.success:
            return False, response.message if response else "Current-guest memory write timed out."

        if not commit_current_guest:
            return True, response.message

        if not self.commit_current_guest_client.wait_for_service(timeout_sec=1.0):
            return False, "Commit-current-guest memory service is unavailable."
        commit_request = CommitCurrentGuest.Request()
        commit_request.new_profile_id = commit_profile_id.strip() or profile.profile_id or "guest_1"
        commit_response = self._wait_for_future(
            self.commit_current_guest_client.call_async(commit_request),
            2.0,
        )
        if not commit_response or not commit_response.success:
            return (
                False,
                commit_response.message if commit_response else "Current-guest commit timed out.",
            )
        return True, commit_response.message

    def capture_snapshot_cb(self, request, response):
        role = request.role.strip().lower() or "guest"
        if role not in ("guest", "host"):
            response.success = False
            response.message = f"Unsupported role '{request.role}'. Use guest or host."
            response.profile = PersonProfile()
            response.json_data = "{}"
            return response

        profile_id = request.profile_id.strip()
        if not profile_id:
            profile_id = "host" if role == "host" else "current_guest"

        if self.latest_appearance is None:
            response.success = False
            response.message = "No appearance estimate is available yet."
            response.profile = PersonProfile()
            response.json_data = "{}"
            return response

        profile = self._build_snapshot_profile(profile_id, role, request.name.strip())
        json_data = self._publish_profile_json(profile)
        self._publish_profile(profile)

        # Section: Immediate snapshots remain available but cannot create a single-frame identity.
        if self.latest_face_detected():
            self.get_logger().info(
                "Snapshot captured without identity enrollment; use capture_named_profile for "
                "multi-frame face enrollment."
            )

        if request.store_in_memory:
            stored, store_message = self._store_profile_in_memory(
                profile,
                bool(request.commit_current_guest),
                request.commit_profile_id,
            )
            if not stored:
                response.success = False
                response.message = store_message
                response.profile = profile
                response.json_data = json_data
                return response

        response.success = True
        response.message = f"Captured immediate profile snapshot for '{profile.profile_id}'."
        response.profile = profile
        response.json_data = json_data
        return response

    def _build_appearance_attributes(self):
        return {
            "glasses": int(self.appearance_snapshot.glasses),
            "glasses_confidence": float(self.appearance_snapshot.glasses_confidence),
            "hat": int(self.appearance_snapshot.hat),
            "hat_confidence": float(self.appearance_snapshot.hat_confidence),
            "face_mask": int(self.appearance_snapshot.face_mask),
            "face_mask_confidence": float(self.appearance_snapshot.face_mask_confidence),
            "shirt_color": self.appearance_snapshot.shirt_color,
            "shirt_color_confidence": float(self.appearance_snapshot.shirt_color_confidence),
        }

    def _persist_profile_identity(self, profile):
        """Special function: Persist a normalized centroid and representative face templates."""
        if not self.embedding_samples:
            self.get_logger().warning(
                f"Skipping identity persistence for '{profile.profile_id}' because no embeddings were captured."
            )
            return

        averaged_embedding = normalize_embedding(
            np.mean(np.array(self.embedding_samples, dtype=np.float32), axis=0)
        )
        if self.identity_store is None:
            raise RuntimeError('Identity store was not configured from a valid SFace observation.')
        # Section: Preserve the public profile embedding field using the SFace centroid only.
        profile.face_embedding = [float(value) for value in averaged_embedding.tolist()]
        profile.register_time = self.get_clock().now().to_msg()
        self.identity_store.register_profile(
            profile_id=profile.profile_id,
            name=profile.name,
            role=profile.role,
            favorite_drink=profile.favorite_drink,
            age=profile.age,
            person_uuid=self.target_person_uuid,
            embedding=averaged_embedding,
            shirt_color=profile.appearance.shirt_color,
            attributes=self._build_appearance_attributes(),
            templates=list(self.embedding_samples),
        )

    def _publish_profile(self, profile):
        if profile.role.strip().lower() == "host":
            self.host_profile_pub.publish(profile)
            target_topic = self.host_profile_topic
        else:
            self.guest_profile_pub.publish(profile)
            target_topic = self.guest_profile_topic

        self.get_logger().info(
            f"Published captured profile_id={profile.profile_id} role={profile.role} to {target_topic}."
        )

    def _publish_status(self, state, completeness_score, confidence_score, active_profile_id):
        status = CurrentGuestStatus()
        status.state = state
        status.completeness_score = float(completeness_score)
        status.confidence_score = float(confidence_score)
        status.active_profile_id = active_profile_id
        self.status_pub.publish(status)

    def execute_cb(self, goal_handle):
        """Special function: Collect enrollment evidence until success, cancellation, or timeout."""
        feedback = CaptureNamedProfile.Feedback()
        result = CaptureNamedProfile.Result()
        capture_started = time.monotonic()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._publish_status("canceled", 0.0, 0.0, self.target_profile_id)
                    result.success = False
                    result.message = "Profile capture canceled."
                    result.profile = PersonProfile()
                    goal_handle.canceled()
                    return result

                if time.monotonic() - capture_started >= self.profile_capture_timeout_sec:
                    self._publish_status("timeout", 0.0, 0.0, self.target_profile_id)
                    result.success = False
                    result.message = (
                        "PROFILE_CAPTURE_TIMEOUT: insufficient distinct valid face samples "
                        f"within {self.profile_capture_timeout_sec:.1f} seconds."
                    )
                    result.profile = PersonProfile()
                    goal_handle.abort()
                    return result

                capture_ready = self._process_latest_observations()

                if locked_track_timed_out(
                    self.target_tracking_session_id,
                    self.last_target_seen_monotonic,
                    time.monotonic(),
                    self.track_loss_timeout_sec,
                ):
                    self._publish_status("track_lost", 0.0, 0.0, self.target_profile_id)
                    result.success = False
                    result.message = (
                        "TARGET_TRACK_LOST: requested session/track was absent for "
                        f"{self.track_loss_timeout_sec:.1f} seconds."
                    )
                    result.person_uuid = self.target_person_uuid
                    result.profile = PersonProfile()
                    goal_handle.abort()
                    return result

                feedback.face_detected = bool(
                    self.latest_face is not None and self.latest_face.get("detected", False)
                )
                feedback.stable_sample_count = len(self.embedding_samples)
                feedback.latest_appearance = copy.deepcopy(self.appearance_snapshot)
                self._publish_status(
                    "capturing",
                    min(
                        1.0,
                        float(len(self.embedding_samples))
                        / float(self.min_enrollment_samples),
                    ),
                    self.appearance_snapshot.shirt_color_confidence,
                    self.target_profile_id,
                )
                goal_handle.publish_feedback(feedback)

                if capture_ready:
                    profile = self._build_profile()
                    try:
                        self._persist_profile_identity(profile)
                    except (OSError, RuntimeError, ValueError) as exc:
                        result.success = False
                        result.message = f"IDENTITY_DATABASE_ERROR: {exc}"
                        result.profile = PersonProfile()
                        self._publish_status("database_error", 0.0, 0.0, self.target_profile_id)
                        goal_handle.abort()
                        return result
                    self._publish_profile(profile)
                    self._publish_status("completed", 1.0, profile.confidence_score, profile.profile_id)
                    result.success = True
                    result.person_uuid = self.target_person_uuid
                    result.message = (
                        f"Captured and published profile '{profile.profile_id}' for role '{profile.role}'."
                    )
                    result.profile = profile
                    goal_handle.succeed()
                    return result

                time.sleep(self.feedback_period_sec)

            result.success = False
            result.message = "Profile capture interrupted."
            result.profile = PersonProfile()
            self._publish_status("interrupted", 0.0, 0.0, self.target_profile_id)
            if goal_handle.is_active:
                goal_handle.abort()
            return result
        finally:
            if self.active_goal_handle is goal_handle:
                self.active_goal_handle = None
            self._reset_capture_state()
            self.target_profile_id = ""
            self.target_role = "guest"
            self.target_name = ""
            self.target_favorite_drink = ""
            self.target_age = 0
            self.target_person_uuid = ""
            self.target_tracking_session_id = ""
            self.target_track_id = 0
            self._publish_status("idle", 0.0, 0.0, "")


def main(args=None):
    rclpy.init(args=args)
    node = ProfileCaptureNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
