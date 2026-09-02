#!/usr/bin/env python3
"""Open-set temporal face identification with fresh-observation action semantics."""

import copy
import json
import threading
import time
from collections import deque

import rclpy
from face_detection.face_backend import EMBEDDING_BACKEND
from face_detection.identity_store import (
    DEFAULT_EMBEDDING_BACKEND,
    DEFAULT_EMBEDDING_DIMENSION,
    DEFAULT_EMBEDDING_MODEL,
    IdentityStore,
    TemporalIdentityVoter,
    observation_is_fresh,
    observation_stamp,
    recognized_identity_payload,
    unknown_match,
)
from face_detection.paths import default_guest_db_path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from receptionist_interfaces.action import RecognizeHumanId
from receptionist_interfaces.msg import GuestAppearance, RecognitionStatus
from receptionist_interfaces.srv import LookupIdentityProfile
from std_msgs.msg import String


class ProfileIdentificationNode(Node):
    """Special function: Match face templates and publish only temporally stable identities."""

    def __init__(self):
        """Special function: Configure model-safe matching, voting, and action evidence state."""
        super().__init__('profile_identification_node')

        # Section: Existing topics remain stable while recognition policy becomes configurable.
        self.declare_parameter('face_topic', '/perception/largest_face')
        self.declare_parameter('appearance_topic', '/appearance/current_estimate')
        self.declare_parameter('db_path', default_guest_db_path())
        self.declare_parameter('embedding_backend', DEFAULT_EMBEDDING_BACKEND)
        self.declare_parameter('embedding_model', DEFAULT_EMBEDDING_MODEL)
        self.declare_parameter('embedding_dimension', DEFAULT_EMBEDDING_DIMENSION)
        self.declare_parameter('max_face_age_sec', 0.5)
        self.declare_parameter('recognition_threshold', 0.45)
        self.declare_parameter('recognition_margin', 0.05)
        self.declare_parameter('identity_window_size', 5)
        self.declare_parameter('identity_min_votes', 3)
        self.declare_parameter('stable_identity_timeout_sec', 1.5)
        self.declare_parameter('recognition_action_timeout_sec', 3.0)
        self.declare_parameter('recognition_action_min_votes', 3)
        self.declare_parameter('recognition_status_topic', '/appearance/recognition_status')
        self.declare_parameter(
            'recognized_identity_topic',
            '/appearance/recognized_human_identity',
        )
        self.declare_parameter('observation_topic', '/appearance/current_person_observation')
        self.declare_parameter('recognize_action_name', '/appearance/recognize_human_id')
        self.declare_parameter('profile_lookup_service', '/appearance/lookup_identity_profile')
        self.declare_parameter('observation_publish_period_sec', 0.5)
        self.declare_parameter('unknown_profile_id', 'unknown')
        self.declare_parameter('target_roi_reset_iou_threshold', 0.5)

        self.face_topic = str(self.get_parameter('face_topic').value)
        self.appearance_topic = str(self.get_parameter('appearance_topic').value)
        self.db_path = str(self.get_parameter('db_path').value)
        self.embedding_backend = str(self.get_parameter('embedding_backend').value).strip()
        self.embedding_model = str(self.get_parameter('embedding_model').value).strip()
        self.embedding_dimension = int(self.get_parameter('embedding_dimension').value)
        self.max_face_age_sec = max(0.0, float(self.get_parameter('max_face_age_sec').value))
        self.recognition_threshold = float(self.get_parameter('recognition_threshold').value)
        self.recognition_margin = max(0.0, float(self.get_parameter('recognition_margin').value))
        self.identity_window_size = max(1, int(self.get_parameter('identity_window_size').value))
        self.identity_min_votes = max(1, int(self.get_parameter('identity_min_votes').value))
        self.stable_identity_timeout_sec = max(
            0.1,
            float(self.get_parameter('stable_identity_timeout_sec').value),
        )
        self.recognition_action_timeout_sec = max(
            0.1,
            float(self.get_parameter('recognition_action_timeout_sec').value),
        )
        self.recognition_action_min_votes = max(
            1,
            int(self.get_parameter('recognition_action_min_votes').value),
        )
        self.recognition_status_topic = str(self.get_parameter('recognition_status_topic').value)
        self.recognized_identity_topic = str(
            self.get_parameter('recognized_identity_topic').value
        )
        self.observation_topic = str(self.get_parameter('observation_topic').value)
        self.recognize_action_name = str(self.get_parameter('recognize_action_name').value)
        self.profile_lookup_service = str(self.get_parameter('profile_lookup_service').value)
        self.observation_publish_period_sec = max(
            0.1,
            float(self.get_parameter('observation_publish_period_sec').value),
        )
        self.unknown_profile_id = str(self.get_parameter('unknown_profile_id').value)
        self.target_roi_reset_iou_threshold = max(
            0.0,
            min(1.0, float(self.get_parameter('target_roi_reset_iou_threshold').value)),
        )

        # Section: Database construction waits for the actual SFace feature dimension.
        self.identity_store = None
        if self.embedding_dimension > 0:
            self._ensure_identity_store(self.embedding_dimension)
        self.identity_voter = TemporalIdentityVoter(
            self.identity_window_size,
            self.identity_min_votes,
        )
        self.latest_appearance = None
        self.latest_face_detected = False
        self.latest_match = unknown_match()
        self.latest_candidate = unknown_match()
        self.latest_match_stable = False
        self.latest_status = self._build_status_msg(self.latest_match)
        self.last_face_stamp = None
        self.last_face_received_monotonic = 0.0
        self.last_selected_track_id = None
        self.last_tracking_session_id = ""
        self.last_selected_roi = None
        self.latest_face_candidates = []
        self.observation_sequence = 0
        self.observation_history = deque(maxlen=max(20, self.identity_window_size * 4))
        self.observation_condition = threading.Condition()
        self.action_lock = threading.Lock()

        # Section: ROS interfaces retain their existing names and message/action types.
        # Section: Reentrant callbacks keep observations flowing while recognition waits.
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
        self.recognition_status_pub = self.create_publisher(
            RecognitionStatus,
            self.recognition_status_topic,
            10,
        )
        # Section: Dedicated identity JSON avoids coupling GUI consumers to action/status messages.
        self.recognized_identity_pub = self.create_publisher(
            String,
            self.recognized_identity_topic,
            10,
        )
        self.observation_pub = self.create_publisher(String, self.observation_topic, 10)
        self.observation_timer = self.create_timer(
            self.observation_publish_period_sec,
            self._publish_observation,
        )
        self.recognize_action_server = ActionServer(
            self,
            RecognizeHumanId,
            self.recognize_action_name,
            execute_callback=self.execute_recognize_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.callback_group,
        )
        # Section: Name lookup lets GPSR choose identity mode before searching for a person.
        self.profile_lookup_server = self.create_service(
            LookupIdentityProfile,
            self.profile_lookup_service,
            self.lookup_identity_profile_cb,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            'Profile identification started. '
            f'backend/model={self.embedding_backend}/{self.embedding_model}, '
            f'threshold={self.recognition_threshold:.3f}, margin={self.recognition_margin:.3f}, '
            f'window={self.identity_window_size}, min_votes={self.identity_min_votes}, '
            f'db_pending_dimension={self.identity_store is None}'
        )

    def _ensure_identity_store(self, embedding_dimension):
        """Special function: Load the database once SFace reports its real feature length."""
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

    @staticmethod
    def _roi_iou(first_roi, second_roi):
        """Special function: Measure target ROI continuity in full-frame coordinates."""
        if not first_roi or not second_roi or len(first_roi) != 4 or len(second_roi) != 4:
            return 0.0
        ax1, ay1, ax2, ay2 = [float(value) for value in first_roi]
        bx1, by1, bx2, by2 = [float(value) for value in second_roi]
        intersection_width = max(0.0, min(ax2, bx2) - max(ax1, bx1))
        intersection_height = max(0.0, min(ay2, by2) - max(ay1, by1))
        intersection = intersection_width * intersection_height
        first_area = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        second_area = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = first_area + second_area - intersection
        return intersection / union if union > 1e-6 else 0.0

    def _reset_for_target_change(self, face_data):
        """Special function: Clear voting when track identity or selected ROI changes substantially."""
        has_track_id = bool(face_data.get('has_track_id', False))
        track_id = int(face_data.get('track_id', 0)) if has_track_id else None
        tracking_session_id = str(face_data.get('tracking_session_id', '')).strip()
        selected_roi = list(face_data.get('selected_roi', []))
        track_changed = (
            (tracking_session_id and tracking_session_id != self.last_tracking_session_id)
            or (
                track_id is not None
                and self.last_selected_track_id is not None
                and track_id != self.last_selected_track_id
            )
        )
        roi_changed = bool(
            selected_roi
            and self.last_selected_roi
            and self._roi_iou(selected_roi, self.last_selected_roi)
            < self.target_roi_reset_iou_threshold
        )
        if track_changed or roi_changed:
            self.identity_voter.reset()
            self.latest_match = unknown_match()
            self.latest_match_stable = False
        self.last_selected_track_id = track_id
        self.last_tracking_session_id = tracking_session_id
        self.last_selected_roi = selected_roi or None

    def appearance_callback(self, msg):
        """Special function: Cache the latest appearance estimate for combined observations."""
        self.latest_appearance = copy.deepcopy(msg)

    def face_callback(self, msg):
        """Special function: Match each distinct face frame and update temporal evidence."""
        try:
            face_data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Failed to parse face message JSON: {exc}')
            return
        stamp = observation_stamp(face_data)
        if stamp is None or stamp == self.last_face_stamp:
            return
        if not observation_is_fresh(
            face_data,
            self.get_clock().now().nanoseconds,
            self.max_face_age_sec,
        ):
            return
        self.last_face_stamp = stamp
        self.last_face_received_monotonic = time.monotonic()
        # Section: Preserve every unambiguous tracked face for name-candidate iteration.
        self.latest_face_candidates = []
        for raw_face in list(face_data.get('faces', [])):
            if not isinstance(raw_face, dict) or not raw_face.get('has_track_id', False):
                continue
            session_id = str(raw_face.get('tracking_session_id', '')).strip()
            if not session_id:
                continue
            candidate = copy.deepcopy(raw_face)
            candidate.update({
                'face_detected': True,
                'detected': True,
                'stamp_sec': int(face_data.get('stamp_sec', 0)),
                'stamp_nanosec': int(face_data.get('stamp_nanosec', 0)),
                'selected_roi': list(raw_face.get('person_bbox', [])),
            })
            self.latest_face_candidates.append(candidate)
        self._reset_for_target_change(face_data)
        self.latest_face_detected = bool(face_data.get('detected', False))
        raw_match = unknown_match()
        if self.latest_face_detected:
            embedding = face_data.get('embedding', [])
            incoming_backend = str(face_data.get('embedding_backend', '')).strip()
            incoming_model = str(face_data.get('embedding_model', '')).strip()
            incoming_dimension = int(face_data.get('embedding_dimension', len(embedding)))
            if incoming_backend != self.embedding_backend or incoming_backend != EMBEDDING_BACKEND:
                self.get_logger().error(
                    f'Rejected face embedding backend {incoming_backend}; '
                    f'expected {self.embedding_backend}.'
                )
            elif incoming_model != self.embedding_model:
                self.get_logger().error(
                    f'Rejected face embedding model {incoming_model}; expected {self.embedding_model}.'
                )
            elif len(embedding) == incoming_dimension and incoming_dimension > 0:
                try:
                    identity_store = self._ensure_identity_store(incoming_dimension)
                    identity_store.refresh_if_changed()
                    raw_match = identity_store.recognize_profile(
                        embedding,
                        threshold=self.recognition_threshold,
                        margin=self.recognition_margin,
                    )
                except ValueError as exc:
                    self.get_logger().error(f'Rejected face embedding metadata: {exc}')
        self.latest_candidate = copy.deepcopy(raw_match)
        # Section: Score every associated face independently for track-based name search.
        if self.identity_store is not None:
            for candidate in self.latest_face_candidates:
                embedding = candidate.get('embedding', [])
                incoming_dimension = int(
                    candidate.get('embedding_dimension', len(embedding))
                )
                metadata_matches = (
                    str(candidate.get('embedding_backend', '')).strip()
                    == self.embedding_backend
                    and str(candidate.get('embedding_model', '')).strip()
                    == self.embedding_model
                    and len(embedding) == incoming_dimension
                    and incoming_dimension > 0
                )
                candidate_match = (
                    self.identity_store.recognize_profile(
                        embedding,
                        threshold=self.recognition_threshold,
                        margin=self.recognition_margin,
                    )
                    if metadata_matches else unknown_match()
                )
                candidate['candidate_profile_id'] = str(
                    candidate_match.get('profile_id', '')
                )
                candidate['candidate_is_known'] = bool(
                    candidate_match.get('is_known', False)
                )
                candidate['candidate_match_score'] = float(
                    candidate_match.get('match_score', -1.0)
                )
                candidate['person_uuid'] = str(
                    candidate_match.get('person_uuid', '')
                )
        stable_match, is_stable = self.identity_voter.add(raw_match)
        self._publish_match(stable_match, is_stable)
        with self.observation_condition:
            self.observation_sequence += 1
            self.observation_history.append(
                (self.observation_sequence, self.latest_face_detected, copy.deepcopy(raw_match))
            )
            self.observation_condition.notify_all()

    def _appearance_flag_to_text(self, value):
        """Special function: Convert GuestAppearance enum values into JSON text."""
        if value == GuestAppearance.YES:
            return 'yes'
        if value == GuestAppearance.NO:
            return 'no'
        return 'unknown'

    def _build_status_msg(self, match):
        """Special function: Preserve the existing RecognitionStatus message contract."""
        status_msg = RecognitionStatus()
        status_msg.profile_id = str(match.get('profile_id', '')).strip()
        status_msg.name = str(match.get('name', '')).strip()
        status_msg.role = str(match.get('role', '')).strip()
        status_msg.match_score = float(match.get('match_score', -1.0))
        status_msg.is_known = bool(match.get('is_known', False))
        status_msg.stamp = self.get_clock().now().to_msg()
        return status_msg

    def _build_observation_payload(self):
        """Special function: Expose stable state and open-set score details in compatible JSON."""
        human_id = self.unknown_profile_id
        if self.latest_match.get('is_known', False):
            human_id = str(self.latest_match.get('profile_id', '')).strip() or self.unknown_profile_id
        shirt_color = 'unknown'
        shirt_color_confidence = 0.0
        glasses = 'unknown'
        glasses_confidence = 0.0
        wearing_glasses = False
        if self.latest_face_detected and self.latest_appearance is not None:
            shirt_color = self.latest_appearance.shirt_color or 'unknown'
            shirt_color_confidence = float(self.latest_appearance.shirt_color_confidence)
            glasses = self._appearance_flag_to_text(self.latest_appearance.glasses)
            glasses_confidence = float(self.latest_appearance.glasses_confidence)
            wearing_glasses = bool(self.latest_appearance.glasses == GuestAppearance.YES)
        return {
            'face_detected': self.latest_face_detected,
            'human_id': human_id,
            'matched_profile_id': str(self.latest_match.get('profile_id', '')).strip(),
            'name': str(self.latest_match.get('name', '')).strip(),
            'role': str(self.latest_match.get('role', '')).strip(),
            'is_known': bool(self.latest_match.get('is_known', False)),
            'is_stable': bool(self.latest_match_stable),
            'candidate_profile_id': str(self.latest_candidate.get('profile_id', '')).strip(),
            'candidate_is_known': bool(self.latest_candidate.get('is_known', False)),
            'candidate_match_score': float(self.latest_candidate.get('match_score', -1.0)),
            'match_score': float(self.latest_match.get('match_score', -1.0)),
            'top2_score': float(self.latest_match.get('top2_score', -1.0)),
            'score_margin': float(self.latest_match.get('score_margin', 0.0)),
            'shirt_color': shirt_color,
            't_shirt_color': shirt_color,
            'shirt_color_confidence': shirt_color_confidence,
            'glasses': glasses,
            'wearing_glasses': wearing_glasses,
            'glasses_confidence': glasses_confidence,
            # Section: Candidate identity remains tied to the upstream tracked face selection.
            'has_track_id': self.last_selected_track_id is not None,
            'track_id': int(self.last_selected_track_id or 0),
            'tracking_session_id': self.last_tracking_session_id,
            'selected_roi': list(self.last_selected_roi or []),
            'candidates': copy.deepcopy(self.latest_face_candidates),
        }

    def lookup_identity_profile_cb(self, request, response):
        """Special function: Return profile usability without exposing stored embeddings."""
        # Section: Reject empty names explicitly so callers do not infer profile absence.
        requested_name = str(request.name).strip()
        if not requested_name:
            response.message = 'INVALID_NAME: profile lookup requires a non-empty name.'
            return response
        # Section: The store initializes from the first valid SFace observation and then refreshes atomically.
        if self.identity_store is None:
            response.message = 'IDENTITY_STORE_NOT_READY: no valid SFace observation initialized the database.'
            return response
        self.identity_store.refresh_if_changed()
        record = self.identity_store.find_profile_by_name(requested_name)
        if record is None:
            response.message = f"PROFILE_NOT_FOUND: no identity profile named '{requested_name}'."
            return response
        # Section: A usable identity requires both a stable id and at least one face template.
        templates = list(record.get('templates', []))
        response.found = True
        response.usable = bool(str(record.get('profile_id', '')).strip() and templates)
        response.profile_id = str(record.get('profile_id', '')).strip()
        response.name = str(record.get('name', '')).strip()
        response.template_count = len(templates)
        response.message = (
            'PROFILE_AVAILABLE: usable face identity profile found.'
            if response.usable
            else 'PROFILE_UNUSABLE: identity profile has no face templates.'
        )
        return response

    def _publish_match(self, match, is_stable):
        """Special function: Publish a known identity only when temporal voting confirms it."""
        self.latest_match = copy.deepcopy(match)
        self.latest_match_stable = bool(is_stable)
        self.latest_status = self._build_status_msg(self.latest_match)
        self.recognition_status_pub.publish(self.latest_status)
        # Section: Publish one explicit real-time identity state for GUI and simple subscribers.
        identity_msg = String()
        identity_msg.data = recognized_identity_payload(
            self.latest_match,
            self.latest_match_stable,
            self.latest_face_detected,
            self.unknown_profile_id,
        )
        self.recognized_identity_pub.publish(identity_msg)

    def _publish_observation(self):
        """Special function: Publish combined status and expire stale stable identity state."""
        if (
            self.last_face_received_monotonic > 0.0
            and time.monotonic() - self.last_face_received_monotonic
            > self.stable_identity_timeout_sec
        ):
            self.identity_voter.reset()
            self.latest_face_detected = False
            self.latest_candidate = unknown_match()
            if self.latest_match_stable or self.latest_match.get('is_known', False):
                self._publish_match(unknown_match(), False)
        observation_msg = String()
        observation_msg.data = json.dumps(self._build_observation_payload())
        self.observation_pub.publish(observation_msg)

    def goal_cb(self, _goal_request):
        """Special function: Reject overlapping recognition actions that would share evidence."""
        return GoalResponse.REJECT if self.action_lock.locked() else GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle):
        """Special function: Preserve cancellation for an active recognition action."""
        return CancelResponse.ACCEPT

    def _finish_action(self, goal_handle, success, message, status, disposition):
        """Special function: Populate the unchanged action result and set its terminal state."""
        result = RecognizeHumanId.Result()
        result.success = bool(success)
        result.message = str(message)
        result.status = copy.deepcopy(status)
        if disposition == 'succeed':
            goal_handle.succeed()
        elif disposition == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def execute_recognize_cb(self, goal_handle):
        """Special function: Wait for new frames and return confirmed known/UNKNOWN evidence."""
        with self.action_lock:
            with self.observation_condition:
                baseline_sequence = self.observation_sequence
            action_voter = TemporalIdentityVoter(
                self.identity_window_size,
                self.recognition_action_min_votes,
            )
            next_sequence = baseline_sequence + 1
            received_observations = 0
            received_faces = 0
            deadline = time.monotonic() + self.recognition_action_timeout_sec
            while rclpy.ok() and time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    return self._finish_action(
                        goal_handle,
                        False,
                        'CANCELLED: human-id recognition canceled.',
                        self._build_status_msg(unknown_match()),
                        'canceled',
                    )
                with self.observation_condition:
                    pending = [item for item in self.observation_history if item[0] >= next_sequence]
                    if not pending:
                        remaining = max(0.0, deadline - time.monotonic())
                        self.observation_condition.wait(timeout=min(0.1, remaining))
                        continue
                for sequence, face_detected, raw_match in pending:
                    next_sequence = sequence + 1
                    received_observations += 1
                    received_faces += int(face_detected)
                    if not face_detected:
                        feedback = RecognizeHumanId.Feedback()
                        feedback.latest_status = self._build_status_msg(unknown_match())
                        goal_handle.publish_feedback(feedback)
                        continue
                    stable_match, stable = action_voter.add(raw_match)
                    feedback = RecognizeHumanId.Feedback()
                    feedback.latest_status = self._build_status_msg(stable_match)
                    goal_handle.publish_feedback(feedback)
                    if not stable:
                        continue
                    status = self._build_status_msg(stable_match)
                    if stable_match.get('is_known', False):
                        return self._finish_action(
                            goal_handle,
                            True,
                            f"SUCCESS_KNOWN: detected {status.profile_id}.",
                            status,
                            'succeed',
                        )
                    if not goal_handle.request.require_known_match:
                        return self._finish_action(
                            goal_handle,
                            True,
                            'SUCCESS_UNKNOWN: face observed but no unambiguous known identity matched.',
                            status,
                            'succeed',
                        )
            timeout_status = self._build_status_msg(unknown_match())
            if received_observations == 0:
                message = 'TIMEOUT: no new face observations arrived before the deadline.'
            elif received_faces == 0:
                message = 'NO_FACE: new observations arrived but no face was detected.'
            else:
                message = 'TIMEOUT: face evidence did not produce the required confirmed result.'
            return self._finish_action(goal_handle, False, message, timeout_status, 'abort')


def main(args=None):
    """Special function: Run callbacks concurrently so actions can wait for new face frames."""
    rclpy.init(args=args)
    node = ProfileIdentificationNode()
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


if __name__ == '__main__':
    main()
