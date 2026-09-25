#!/usr/bin/env python3
"""Section: Lifecycle-managed multi-person appearance perception for GPSR."""

from __future__ import annotations

import time
import uuid
from typing import Any

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from robot_ai_interfaces.msg import (
    GuestAppearance,
    PersonProfile,
    TrackedPersonAppearance,
    TrackedPersonAppearanceArray,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String
from putrabot_interfaces.msg import HumanSkeletons
from vision.msg import RecognizedPerson

from face_detection.appearance_estimators import (
    HsvShirtColorEstimator,
    TemporalCategoricalVoter,
    YoloAttributeEstimator,
    best_attribute_confidence,
    crop_person_roi,
)
from face_detection.paths import default_detection_model_paths


def bbox_iou(first: list[float], second: list[float]) -> float:
    """Special function: Measure overlap between identity and skeleton boxes."""
    left = max(float(first[0]), float(second[0]))
    top = max(float(first[1]), float(second[1]))
    right = min(float(first[2]), float(second[2]))
    bottom = min(float(first[3]), float(second[3]))
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, float(first[2]) - float(first[0])) * max(0.0, float(first[3]) - float(first[1]))
    second_area = max(0.0, float(second[2]) - float(second[0])) * max(0.0, float(second[3]) - float(second[1]))
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def associate_unique_skeletons(
    track_boxes: list[list[float]],
    skeleton_boxes: list[list[float]],
    minimum_iou: float,
) -> dict[int, int]:
    """Special function: Greedily assign unique track/skeleton pairs by descending IoU."""
    # Section: Stable indices break equal-score ties deterministically without new dependencies.
    pairs: list[tuple[float, int, int]] = []
    for track_index, track_box in enumerate(track_boxes):
        for skeleton_index, skeleton_box in enumerate(skeleton_boxes):
            score = bbox_iou(track_box, skeleton_box)
            if score >= float(minimum_iou):
                pairs.append((score, track_index, skeleton_index))
    pairs.sort(key=lambda item: (-item[0], item[1], item[2]))
    assignments: dict[int, int] = {}
    used_skeletons: set[int] = set()
    for _score, track_index, skeleton_index in pairs:
        if track_index in assignments or skeleton_index in used_skeletons:
            continue
        assignments[track_index] = skeleton_index
        used_skeletons.add(skeleton_index)
    return assignments


def torso_roi(frame: Any, person_bbox: list[float]) -> Any:
    """Special function: Crop the upper torso inside one canonical person box."""
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = [int(value) for value in person_bbox]
    box_width = max(1, x2 - x1)
    box_height = max(1, y2 - y1)
    left = max(0, min(width, x1 + int(0.18 * box_width)))
    right = max(0, min(width, x2 - int(0.18 * box_width)))
    top = max(0, min(height, y1 + int(0.20 * box_height)))
    bottom = max(0, min(height, y1 + int(0.58 * box_height)))
    return frame[top:bottom, left:right] if right > left and bottom > top else None


class TrackedAppearanceLifecycleNode(LifecycleNode):
    """Special function: Publish appearance estimates keyed by canonical identity tracks."""

    def __init__(self) -> None:
        """Special function: Declare GPSR perception resources without allocating models."""
        super().__init__("gpsr_tracked_appearance_node")
        # Section: Topics and thresholds remain stable across lifecycle generations.
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("identity_topic", "/human_identity/recognized_people")
        self.declare_parameter("skeleton_topic", "/perception/human_skeletons")
        self.declare_parameter("pose_summary_topic", "/perception/human_poses")
        self.declare_parameter("output_topic", "/appearance/tracked_people")
        self.declare_parameter("model_paths", default_detection_model_paths())
        self.declare_parameter("appearance_device", "0")
        self.declare_parameter("attribute_confidence", 0.4)
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("association_time_tolerance_sec", 0.35)
        self.declare_parameter("association_iou_threshold", 0.10)
        self.declare_parameter("shirt_history_size", 7)
        self.declare_parameter("shirt_min_votes", 3)
        # Section: Every configured resource starts empty in UNCONFIGURED.
        self.skill_active = False
        self.bridge = None
        self.models: list[Any] = []
        self.attribute_estimator = None
        self.shirt_estimator = None
        self.publisher = None
        self.image_sub = None
        self.identity_sub = None
        self.skeleton_sub = None
        self.pose_sub = None
        self.tracking_session_id = ""
        self.identities: dict[int, tuple[Any, float]] = {}
        self.skeletons: list[Any] = []
        self.skeleton_received_at = 0.0
        self.pose_people: dict[int, dict[str, Any]] = {}
        self.pose_received_at = 0.0
        self.color_voters: dict[int, TemporalCategoricalVoter] = {}

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Load models and allocate ROS interfaces for one generation."""
        try:
            from ultralytics import YOLO

            # Section: A fresh session prevents track IDs being reused across cleanup boundaries.
            self.tracking_session_id = uuid.uuid4().hex
            self.bridge = CvBridge()
            self.models = [
                YOLO(str(path))
                for path in self.get_parameter("model_paths").value
                if str(path).strip()
            ]
            self.attribute_estimator = YoloAttributeEstimator(
                self.models,
                float(self.get_parameter("attribute_confidence").value),
                str(self.get_parameter("appearance_device").value),
            )
            self.shirt_estimator = HsvShirtColorEstimator()
            self.identities = {}
            self.skeletons = []
            self.skeleton_received_at = 0.0
            self.pose_people = {}
            self.pose_received_at = 0.0
            self.color_voters = {}
            self.publisher = self.create_lifecycle_publisher(
                TrackedPersonAppearanceArray,
                str(self.get_parameter("output_topic").value),
                10,
            )
            self.image_sub = self.create_subscription(
                Image,
                str(self.get_parameter("image_topic").value),
                self.image_callback,
                qos_profile_sensor_data,
            )
            self.identity_sub = self.create_subscription(
                RecognizedPerson,
                str(self.get_parameter("identity_topic").value),
                self.identity_callback,
                10,
            )
            self.skeleton_sub = self.create_subscription(
                HumanSkeletons,
                str(self.get_parameter("skeleton_topic").value),
                self.skeleton_callback,
                10,
            )
            self.pose_sub = self.create_subscription(
                String,
                str(self.get_parameter("pose_summary_topic").value),
                self.pose_callback,
                10,
            )
            self.get_logger().info("[GPSR_LIFECYCLE] Tracked appearance configured.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Tracked appearance configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Enable inference only after lifecycle activation."""
        result = super().on_activate(state)
        if result == TransitionCallbackReturn.SUCCESS:
            self.skill_active = True
        return result

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Stop inference before deactivating publishers."""
        self.skill_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Release models, subscriptions, and track histories."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Release configured resources during shutdown."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Restore a clean unconfigured baseline after errors."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> None:
        """Special function: Destroy every lifecycle-owned ROS and model resource."""
        for name in ("image_sub", "identity_sub", "skeleton_sub", "pose_sub"):
            subscription = getattr(self, name)
            if subscription is not None:
                self.destroy_subscription(subscription)
                setattr(self, name, None)
        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
            self.publisher = None
        self.models = []
        self.attribute_estimator = None
        self.shirt_estimator = None
        self.bridge = None
        self.identities = {}
        self.skeletons = []
        self.skeleton_received_at = 0.0
        self.pose_people = {}
        self.pose_received_at = 0.0
        self.color_voters = {}

    def identity_callback(self, message: RecognizedPerson) -> None:
        """Special function: Cache canonical identity tracks using receipt-time freshness."""
        self.identities[int(message.track_id)] = (message, time.monotonic())

    def skeleton_callback(self, message: HumanSkeletons) -> None:
        """Special function: Cache pose boxes used for cross-detector association."""
        self.skeletons = list(message.skeletons)
        self.skeleton_received_at = time.monotonic()

    def pose_callback(self, message: String) -> None:
        """Special function: Cache gesture labels keyed by the skeleton detector's local id."""
        import json

        try:
            payload = json.loads(str(message.data))
        except (json.JSONDecodeError, TypeError):
            return
        people = payload.get("people", []) if isinstance(payload, dict) else []
        self.pose_people = {
            int(person.get("person_id", 0)): dict(person)
            for person in people
            if isinstance(person, dict)
        }
        self.pose_received_at = time.monotonic()

    def image_callback(self, message: Image) -> None:
        """Special function: Estimate and publish every fresh canonical person's appearance."""
        if not self.skill_active or self.bridge is None or self.publisher is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Tracked appearance image conversion failed: {exc}")
            return
        now = time.monotonic()
        stale_timeout = max(0.1, float(self.get_parameter("stale_timeout_sec").value))
        output = TrackedPersonAppearanceArray()
        output.header = message.header
        # Section: Freeze fresh tracks and compute one frame-wide one-to-one pose assignment.
        fresh_identities: list[Any] = []
        for track_id, (identity, received_at) in sorted(self.identities.items()):
            if now - received_at > stale_timeout:
                self.identities.pop(track_id, None)
                self.color_voters.pop(track_id, None)
                continue
            fresh_identities.append(identity)
        skeletons_are_fresh = (
            now - self.skeleton_received_at
            <= max(0.01, float(self.get_parameter("association_time_tolerance_sec").value))
        )
        current_skeletons = list(self.skeletons) if skeletons_are_fresh else []
        assignments = associate_unique_skeletons(
            [[float(value) for value in identity.bbox_xyxy] for identity in fresh_identities],
            [
                [skeleton.x1, skeleton.y1, skeleton.x2, skeleton.y2]
                for skeleton in current_skeletons
            ],
            float(self.get_parameter("association_iou_threshold").value),
        )
        for track_index, identity in enumerate(fresh_identities):
            skeleton = (
                current_skeletons[assignments[track_index]]
                if track_index in assignments
                else None
            )
            observation = self.build_observation(message, frame, identity, skeleton)
            if observation is not None:
                output.people.append(observation)
        self.publisher.publish(output)

    def build_observation(
        self,
        image: Image,
        frame: Any,
        identity: RecognizedPerson,
        skeleton: Any | None = None,
    ) -> Any:
        """Special function: Build one tracked appearance and associated pose snapshot."""
        bbox = [float(value) for value in identity.bbox_xyxy]
        person_crop, _ = crop_person_roi(frame, bbox)
        if person_crop is None:
            return None
        scores = self.attribute_estimator.estimate(person_crop) if self.attribute_estimator else {}
        shirt_color, shirt_confidence = (
            self.shirt_estimator.estimate(torso_roi(frame, bbox))
            if self.shirt_estimator else ("unknown", 0.0)
        )
        stable_color, stable_confidence = self.update_shirt_color(
            int(identity.track_id), shirt_color, shirt_confidence
        )
        appearance = GuestAppearance()
        appearance.header = image.header
        appearance.stamp = image.header.stamp
        self.set_positive_attribute(appearance, "glasses", scores, ("glasses", "eyeglasses"))
        self.set_positive_attribute(appearance, "hat", scores, ("hat", "cap", "headwear"))
        self.set_positive_attribute(appearance, "face_mask", scores, ("face_mask", "mask"))
        appearance.shirt_color = stable_color
        appearance.shirt_color_confidence = float(stable_confidence)
        pose_data = self.pose_data_for_skeleton(skeleton)
        tracked = TrackedPersonAppearance()
        tracked.header = image.header
        tracked.tracking_session_id = str(
            getattr(identity, "tracking_session_id", "") or self.tracking_session_id
        )
        tracked.track_id = int(identity.track_id)
        tracked.person_id = int(identity.person_id)
        tracked.profile_id = f"human_identity:{int(identity.person_id)}" if int(identity.person_id) > 0 else ""
        tracked.person_uuid = str(getattr(identity, "person_uuid", ""))
        tracked.bbox_xyxy = bbox
        tracked.appearance = appearance
        tracked.profile = self.identity_profile(identity, tracked.profile_id, appearance)
        tracked.pose = str(getattr(skeleton, "pose", "unknown") or "unknown")
        tracked.pose_confidence = float(getattr(skeleton, "confidence", 0.0) or 0.0)
        tracked.gesture = str(pose_data.get("gesture", "none"))
        tracked.gesture_confidence = float(pose_data.get("gesture_confidence", 0.0) or 0.0)
        return tracked

    def update_shirt_color(self, track_id: int, color: str, confidence: float) -> tuple[str, float]:
        """Special function: Stabilize shirt color independently for each canonical track."""
        voter = self.color_voters.setdefault(
            track_id,
            TemporalCategoricalVoter(
                int(self.get_parameter("shirt_history_size").value),
                int(self.get_parameter("shirt_min_votes").value),
            ),
        )
        return voter.update(color, confidence)

    @staticmethod
    def set_positive_attribute(
        appearance: GuestAppearance,
        field_name: str,
        scores: dict[str, float],
        aliases: tuple[str, ...],
    ) -> None:
        """Special function: Store positive evidence while preserving UNKNOWN on absence."""
        confidence = best_attribute_confidence(scores, aliases)
        setattr(appearance, field_name, GuestAppearance.YES if confidence > 0.0 else GuestAppearance.UNKNOWN)
        setattr(appearance, f"{field_name}_confidence", float(confidence))

    def pose_data_for_skeleton(self, skeleton: Any | None) -> dict[str, Any]:
        """Special function: Resolve gesture data for an already unique skeleton assignment."""
        if skeleton is None:
            return {}
        tolerance = max(0.01, float(self.get_parameter("association_time_tolerance_sec").value))
        now = time.monotonic()
        try:
            local_id = int(str(skeleton.skeleton_id).rsplit("_", 1)[-1])
        except ValueError:
            local_id = 0
        return (
            dict(self.pose_people.get(local_id, {}))
            if now - self.pose_received_at <= tolerance else {}
        )

    @staticmethod
    def identity_profile(
        identity: RecognizedPerson,
        profile_id: str,
        appearance: GuestAppearance,
    ) -> PersonProfile:
        """Special function: Convert registered identity metadata into a guest profile snapshot."""
        profile = PersonProfile()
        profile.profile_id = profile_id
        profile.role = "guest"
        profile.name = str(identity.person_name)
        profile.age = int(identity.age)
        profile.favorite_drink = str(identity.favourite_drink)
        profile.appearance = appearance
        profile.is_confirmed = bool(identity.has_registered_profile)
        profile.confidence_score = float(identity.final_score)
        profile.stamp = appearance.stamp
        return profile


def main(args: list[str] | None = None) -> None:
    """Special function: Run the GPSR tracked-appearance lifecycle node."""
    rclpy.init(args=args)
    node = TrackedAppearanceLifecycleNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
