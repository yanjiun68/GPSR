"""Section: ROS adapters used by the two-mode FIND_PERSON state machine."""

from __future__ import annotations

import json
import re
import threading
import time
from collections import deque
from typing import Any

from rclpy.action import ActionClient
from std_msgs.msg import String

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.action_types import ASK_AND_LISTEN
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.gpsr.outcomes import SUCCESS
from robot_yasmin_tasks.ros_clients.dialogue_clients import DialogueClients

try:
    # Section: These are the repository's existing face recognition and enrollment interfaces.
    from receptionist_interfaces.action import CaptureNamedProfile, RecognizeHumanId
    from receptionist_interfaces.srv import LookupIdentityProfile
except ImportError:  # pragma: no cover - permits source-only unit tests before interface build.
    CaptureNamedProfile = None
    RecognizeHumanId = None
    LookupIdentityProfile = None


class FindPersonClients:
    """Special function: Adapt shared face identity, speech, and tracking interfaces for GPSR."""

    def __init__(
        self,
        node: Any | None = None,
        simulate: bool = True,
        dialogue: DialogueClients | None = None,
        observation_topic: str = "/appearance/current_person_observation",
        lookup_service: str = "/appearance/lookup_identity_profile",
        recognition_action: str = "/appearance/recognize_human_id",
        enrollment_action: str = "/appearance/capture_named_profile",
    ) -> None:
        """Special function: Create persistent ROS clients without owning perception processes."""
        # Section: Store runtime policy and reuse the executor's shared dialogue client.
        self.node = node
        self.simulate = bool(simulate)
        self.dialogue = dialogue or DialogueClients(node=node, simulate=simulate)
        self.observation_topic = observation_topic
        self.lookup_service = lookup_service
        self.recognition_action = recognition_action
        self.enrollment_action = enrollment_action
        # Section: A bounded task-independent cache carries fresh tracked-face observations.
        self._condition = threading.Condition()
        self._sequence = 0
        self._history: deque[tuple[int, float, dict[str, Any]]] = deque(maxlen=100)
        self._latest_observation: dict[str, Any] = {}
        # Section: Construct interfaces only in a real ROS runtime with generated types present.
        live = node is not None and not self.simulate
        self._observation_sub = (
            node.create_subscription(String, observation_topic, self._observation_cb, 10)
            if live
            else None
        )
        self._lookup_client = (
            node.create_client(LookupIdentityProfile, lookup_service)
            if live and LookupIdentityProfile is not None
            else None
        )
        self._recognition_client = (
            ActionClient(node, RecognizeHumanId, recognition_action)
            if live and RecognizeHumanId is not None
            else None
        )
        self._enrollment_client = (
            ActionClient(node, CaptureNamedProfile, enrollment_action)
            if live and CaptureNamedProfile is not None
            else None
        )

    def _observation_cb(self, message: String) -> None:
        """Special function: Cache one validated JSON observation and wake candidate searches."""
        # Section: Malformed producer data is ignored rather than treated as a candidate.
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        # Section: Reception monotonic time lets consumers reject stale cached observations.
        with self._condition:
            self._sequence += 1
            self._latest_observation = dict(payload)
            self._history.append((self._sequence, time.monotonic(), dict(payload)))
            self._condition.notify_all()

    @staticmethod
    def _is_canceled(blackboard: Any) -> bool:
        """Special function: Read the top-level action cancellation callback."""
        callback = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        return bool(callback()) if callable(callback) else False

    def _wait_future(self, future: Any, deadline: float, blackboard: Any) -> Any:
        """Special function: Wait for a ROS future while preserving cancellation and deadlines."""
        # Section: An event avoids spinning the shared executor from inside a state callback.
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        while time.monotonic() < deadline:
            if done.wait(timeout=min(0.05, max(0.0, deadline - time.monotonic()))):
                return future.result()
            if self._is_canceled(blackboard):
                return None
        # Section: Completion observed after the absolute deadline is never accepted as success.
        return None

    def lookup_profile(
        self,
        blackboard: Any,
        action: ActionStepData,
        target_name: str,
        timeout_sec: float,
    ) -> dict[str, Any]:
        """Special function: Ask the face registry whether a usable named profile exists."""
        # Section: Deterministic simulation data supports camera-free state-machine tests.
        if self.simulate:
            profile = action.parameters.get("simulated_profile")
            if isinstance(profile, dict) and profile.get("usable", True):
                return {"status": "found", **profile}
            return {"status": "missing", "reason": "PROFILE_NOT_FOUND"}
        if self._lookup_client is None:
            return {"status": "failed", "reason": "PROFILE_LOOKUP_INTERFACE_UNAVAILABLE"}
        if self._is_canceled(blackboard):
            return {"status": "canceled", "reason": "CANCELED"}
        # Section: Bound service discovery separately so a missing registry fails explicitly.
        wait_sec = min(max(0.05, timeout_sec), 2.0)
        if not self._lookup_client.wait_for_service(timeout_sec=wait_sec):
            return {"status": "failed", "reason": "PROFILE_LOOKUP_SERVICE_UNAVAILABLE"}
        request = LookupIdentityProfile.Request()
        request.name = target_name
        response = self._wait_future(
            self._lookup_client.call_async(request),
            time.monotonic() + max(0.05, timeout_sec),
            blackboard,
        )
        if self._is_canceled(blackboard):
            return {"status": "canceled", "reason": "CANCELED"}
        if response is None:
            return {"status": "timeout", "reason": "PROFILE_LOOKUP_TIMEOUT"}
        if not response.found or not response.usable:
            return {
                "status": "missing",
                "reason": str(response.message or "PROFILE_NOT_FOUND"),
            }
        return {
            "status": "found",
            "profile_id": str(response.profile_id),
            "name": str(response.name),
            "template_count": int(response.template_count),
            "usable": True,
        }

    def find_registered(
        self,
        blackboard: Any,
        action: ActionStepData,
        target_name: str,
        profile: dict[str, Any],
        threshold: float,
        timeout_sec: float,
    ) -> dict[str, Any]:
        """Special function: Search recognition results until the requested profile is confirmed."""
        # Section: Simulation can model success, mismatch, timeout, and explicit failure.
        if self.simulate:
            result = action.parameters.get("simulated_registered_result", {})
            if isinstance(result, dict) and result:
                return dict(result)
            return {"status": "not_found", "reason": "PROFILE_NOT_OBSERVED"}
        # Section: Track-keyed candidate evidence avoids largest-face identity switching.
        deadline = time.monotonic() + max(0.05, timeout_sec)
        expected_profile_id = str(profile.get("profile_id", "")).strip()
        minimum_frames = max(
            1, int(action.parameters.get("profile_match_min_frames", 3))
        )
        evidence: dict[str, int] = {}
        last_stamp_by_key: dict[str, tuple[int, int]] = {}
        with self._condition:
            next_sequence = self._sequence + 1
        while time.monotonic() < deadline:
            if self._is_canceled(blackboard):
                return {"status": "canceled", "reason": "CANCELED"}
            with self._condition:
                pending = [item for item in self._history if item[0] >= next_sequence]
                if not pending:
                    self._condition.wait(
                        timeout=min(0.1, max(0.0, deadline - time.monotonic()))
                    )
                    continue
            for sequence, received_at, observation in pending:
                next_sequence = sequence + 1
                if time.monotonic() - received_at > 1.0:
                    continue
                for candidate in list(observation.get("candidates", [])):
                    if not isinstance(candidate, dict):
                        continue
                    key = self._candidate_id(candidate)
                    matched_id = str(candidate.get("candidate_profile_id", "")).strip()
                    score = float(candidate.get("candidate_match_score", -1.0))
                    if (
                        not key
                        or matched_id != expected_profile_id
                        or not bool(candidate.get("candidate_is_known", False))
                        or score < float(threshold)
                    ):
                        continue
                    stamp = (
                        int(candidate.get("stamp_sec", 0)),
                        int(candidate.get("stamp_nanosec", 0)),
                    )
                    if stamp == last_stamp_by_key.get(key):
                        continue
                    last_stamp_by_key[key] = stamp
                    evidence[key] = evidence.get(key, 0) + 1
                    if evidence[key] >= minimum_frames:
                        return {
                            "status": "found",
                            "person_ref": self._person_ref(
                                action,
                                target_name,
                                matched_id,
                                candidate,
                                score,
                                "registered_profile",
                            ),
                        }
        return {"status": "not_found", "reason": "PROFILE_NOT_OBSERVED"}

    def next_candidate(
        self,
        blackboard: Any,
        action: ActionStepData,
        rejected_ids: set[str],
        timeout_sec: float,
    ) -> dict[str, Any]:
        """Special function: Return the next fresh, non-rejected tracked face candidate."""
        # Section: Simulation candidates are consumed once through task-local blackboard state.
        if self.simulate:
            index_key = "_find_person_simulated_candidate_index"
            index = int(bb_get(blackboard, index_key, 0))
            candidates = list(action.parameters.get("simulated_candidates", []))
            while index < len(candidates):
                candidate = dict(candidates[index])
                index += 1
                blackboard[index_key] = index
                candidate.setdefault("candidate_id", f"simulated:{index}")
                if candidate["candidate_id"] not in rejected_ids:
                    return {"status": "candidate", "candidate": candidate}
            return {"status": "exhausted", "reason": "NO_MORE_CANDIDATES"}
        # Section: Only observations received after this call are eligible for confirmation.
        with self._condition:
            next_sequence = self._sequence + 1
        deadline = time.monotonic() + max(0.05, timeout_sec)
        saw_fresh_observation = False
        while time.monotonic() < deadline:
            if self._is_canceled(blackboard):
                return {"status": "canceled", "reason": "CANCELED"}
            with self._condition:
                pending = [item for item in self._history if item[0] >= next_sequence]
                if not pending:
                    self._condition.wait(timeout=min(0.1, max(0.0, deadline - time.monotonic())))
                    continue
            for sequence, received_at, observation in pending:
                next_sequence = sequence + 1
                if time.monotonic() - received_at > 1.0:
                    continue
                saw_fresh_observation = True
                # Section: One frame may contain several independently tracked face candidates.
                candidates = observation.get("candidates", [])
                frame_candidates = (
                    list(candidates)
                    if isinstance(candidates, list) and candidates
                    else [observation]
                )
                for candidate_observation in frame_candidates:
                    if not isinstance(candidate_observation, dict):
                        continue
                    if not (
                        candidate_observation.get("face_detected", False)
                        or candidate_observation.get("detected", False)
                    ):
                        continue
                    candidate_id = self._candidate_id(candidate_observation)
                    if not candidate_id or candidate_id in rejected_ids:
                        continue
                    session_id = str(
                        candidate_observation.get("tracking_session_id", "")
                    ).strip()
                    track_id = int(candidate_observation.get("track_id", 0))
                    return {
                        "status": "candidate",
                        "candidate": {
                            "candidate_id": candidate_id,
                            "tracking_session_id": session_id,
                            "track_id": track_id,
                            "tracking_id": track_id,
                            "face_roi": list(candidate_observation.get("bbox", [])),
                            "last_seen": {
                                "stamp_sec": int(candidate_observation.get("stamp_sec", 0)),
                                "stamp_nanosec": int(candidate_observation.get("stamp_nanosec", 0)),
                            },
                            "degraded_tracking": not bool(session_id),
                            "observation": candidate_observation,
                        },
                    }
        if saw_fresh_observation:
            return {"status": "exhausted", "reason": "NO_VISIBLE_FACE_CANDIDATE"}
        return {"status": "failed", "reason": "PERSON_OBSERVATION_STREAM_UNAVAILABLE"}

    def ask_confirmation(
        self,
        blackboard: Any,
        action: ActionStepData,
        target_name: str,
        candidate: dict[str, Any],
        timeout_sec: float,
    ) -> dict[str, Any]:
        """Special function: Ask the exact binary identity question through shared TTS/STT."""
        # Section: Each simulated candidate may carry one transcript or a retry transcript list.
        if self.simulate:
            return {"status": "answer", "transcript": ""}
        if self._is_canceled(blackboard):
            return {"status": "canceled", "reason": "CANCELED"}
        dialogue_action = ActionStepData(
            action_type=ASK_AND_LISTEN,
            parameters={
                "question": f"Are you {target_name}?",
                "record_sec": min(4.0, max(1.0, timeout_sec)),
                "listen_timeout_sec": max(0.1, timeout_sec),
            },
            timeout_sec=max(0.1, timeout_sec),
        )
        result = self.dialogue.ask_and_listen(blackboard, dialogue_action)
        if self._is_canceled(blackboard):
            return {"status": "canceled", "reason": "CANCELED"}
        if result.outcome != SUCCESS:
            return {"status": "unknown", "reason": str(result.message)}
        return {
            "status": "answer",
            "transcript": str(result.result.get("answer", bb_get(blackboard, bb.DIALOGUE_ANSWER, ""))),
        }

    def enroll_profile(
        self,
        blackboard: Any,
        action: ActionStepData,
        target_name: str,
        candidate: dict[str, Any],
        timeout_sec: float,
    ) -> dict[str, Any]:
        """Special function: Best-effort enroll the verbally confirmed face under the target name."""
        # Section: Simulation lets tests verify that enrollment failure never loses a confirmed person.
        if self.simulate:
            simulated = action.parameters.get("simulated_enrollment", {"status": "enrolled"})
            return dict(simulated) if isinstance(simulated, dict) else {"status": "enrolled"}
        if self._enrollment_client is None:
            return {"status": "failed", "reason": "ENROLLMENT_INTERFACE_UNAVAILABLE"}
        if not self._enrollment_client.wait_for_server(
            timeout_sec=min(2.0, max(0.05, timeout_sec))
        ):
            return {"status": "failed", "reason": "ENROLLMENT_ACTION_UNAVAILABLE"}
        # Section: The normalized name makes a stable profile key while preserving the spoken name.
        profile_id = "gpsr_" + re.sub(r"[^a-z0-9]+", "_", target_name.casefold()).strip("_")
        goal = CaptureNamedProfile.Goal()
        goal.profile_id = profile_id or "gpsr_person"
        goal.role = "guest"
        goal.name = target_name
        session_id = str(candidate.get("tracking_session_id", "")).strip()
        track_id = int(candidate.get("track_id", candidate.get("tracking_id", 0)) or 0)
        if not session_id or track_id <= 0:
            return {"status": "failed", "reason": "ENROLLMENT_TARGET_NOT_TRACK_LOCKED"}
        # Section: The action goal carries the exact verbally confirmed canonical track.
        goal.tracking_session_id = session_id
        goal.track_id = track_id
        goal.person_uuid = str(candidate.get("person_uuid", ""))
        deadline = time.monotonic() + max(0.05, timeout_sec)
        goal_handle = self._wait_future(
            self._enrollment_client.send_goal_async(goal), deadline, blackboard
        )
        if self._is_canceled(blackboard):
            return {"status": "canceled", "reason": "CANCELED"}
        if goal_handle is None or not goal_handle.accepted:
            return {"status": "failed", "reason": "ENROLLMENT_GOAL_REJECTED_OR_TIMEOUT"}
        wrapped = self._wait_future(goal_handle.get_result_async(), deadline, blackboard)
        if self._is_canceled(blackboard):
            goal_handle.cancel_goal_async()
            return {"status": "canceled", "reason": "CANCELED"}
        if wrapped is None:
            goal_handle.cancel_goal_async()
            return {"status": "failed", "reason": "ENROLLMENT_RESULT_TIMEOUT"}
        result = wrapped.result
        if not result.success:
            return {"status": "failed", "reason": str(result.message)}
        return {
            "status": "enrolled",
            "profile_id": str(result.profile.profile_id),
            "person_uuid": str(getattr(result, "person_uuid", "")),
            "message": str(result.message),
        }

    @staticmethod
    def _candidate_id(observation: dict[str, Any]) -> str:
        """Special function: Build a canonical session/track key or explicit degraded fallback."""
        session_id = str(observation.get("tracking_session_id", "")).strip()
        track_id = int(observation.get("track_id", 0) or 0)
        if session_id and observation.get("has_track_id", False) and track_id > 0:
            return f"{session_id}:{track_id}"
        # Section: Compatibility keys are task-local and visibly marked as degraded.
        if observation.get("has_track_id", False) and track_id > 0:
            return f"degraded:track:{track_id}"
        roi = tuple(int(value) for value in observation.get("selected_roi", [])[:4])
        return f"degraded:roi:{roi}" if len(roi) == 4 else ""

    @staticmethod
    def _person_ref(
        action: ActionStepData,
        target_name: str,
        profile_id: str | None,
        observation: dict[str, Any],
        confidence: float,
        source: str,
    ) -> dict[str, Any]:
        """Special function: Normalize identity evidence for downstream GPSR skills."""
        # Section: Keep every required consumer field JSON-safe and explicit.
        tracking_id = (
            int(observation.get("track_id", 0))
            if observation.get("has_track_id", False)
            else None
        )
        tracking_session_id = str(observation.get("tracking_session_id", "")).strip()
        person_id = str(profile_id or (f"track_{tracking_id}" if tracking_id is not None else target_name))
        return {
            "person_id": person_id,
            "name": target_name,
            "target_name": target_name,
            "profile_id": str(profile_id) if profile_id else None,
            "person_uuid": str(observation.get("person_uuid", "")),
            "tracking_id": tracking_id,
            "track_id": tracking_id,
            "tracking_session_id": tracking_session_id,
            "candidate_key": (
                f"{tracking_session_id}:{tracking_id}"
                if tracking_session_id and tracking_id is not None else ""
            ),
            "location": str(action.target_location or action.parameters.get("location", "")),
            "pose": observation.get("pose"),
            "confidence": float(confidence),
            "source": str(source).upper(),
            "identification_source": str(source).upper(),
        }
