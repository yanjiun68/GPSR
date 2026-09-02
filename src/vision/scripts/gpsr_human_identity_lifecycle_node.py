#!/usr/bin/env python3
"""Section: Lifecycle-managed canonical person tracking and identity for GPSR."""

from __future__ import annotations

from pathlib import Path
from threading import Lock
from typing import Any
from uuid import uuid4

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from receptionist_interfaces.msg import PersonProfile
from receptionist_interfaces.srv import GetProfile
from sensor_msgs.msg import Image
from ultralytics import YOLO

from human_identity.human_identity.database import IdentityDatabase
from human_identity.human_identity.feature_extractor import FeatureExtractor
from human_identity.human_identity.matcher import IdentityMatcher
from human_identity.human_identity.registration_manager import rank_bounding_boxes
from vision.msg import RecognizedPerson


class GpsrHumanIdentityLifecycleNode(LifecycleNode):
    """Special function: Publish stable tracked-person IDs only while lifecycle ACTIVE."""

    def __init__(self) -> None:
        """Special function: Declare model and topic settings without loading resources."""
        super().__init__("gpsr_human_identity_node")
        # Section: Parameters mirror the existing human_identity node for asset reuse.
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_topic", "/human_identity/recognized_people")
        self.declare_parameter("person_model", "yolo11n.pt")
        self.declare_parameter("attribute_model", "human_attributes.pt")
        self.declare_parameter("pose_model", "pose_landmarker_full.task")
        self.declare_parameter("database_path", str(Path.home() / ".ros/vision/human_identity.db"))
        self.declare_parameter("device", "0")
        self.declare_parameter("identity_threshold", 0.72)
        self.declare_parameter("ambiguity_margin", 0.08)
        self.declare_parameter("confidence_threshold", 0.40)
        self.skill_active = False
        self.bridge = None
        self.person_model = None
        self.attribute_model = None
        self.database = None
        self.extractor = None
        self.matcher = None
        self.publisher = None
        self.subscription = None
        self.profile_service = None
        self.device = "0"
        self.confidence_threshold = 0.40
        self.tracking_session_id = ""
        self.inference_lock = Lock()

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Load tracking, feature, and identity resources."""
        try:
            share = Path(get_package_share_directory("vision"))
            model_dir = share / "models"
            person_path = model_dir / str(self.get_parameter("person_model").value)
            if not person_path.is_file():
                raise FileNotFoundError(f"Person model not found: {person_path}")
            attribute_path = model_dir / str(self.get_parameter("attribute_model").value)
            self.device = str(self.get_parameter("device").value)
            self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
            self.bridge = CvBridge()
            # Section: A new session prevents reused tracker integers colliding across cleanup.
            self.tracking_session_id = uuid4().hex
            self.person_model = YOLO(str(person_path))
            self.attribute_model = YOLO(str(attribute_path)) if attribute_path.is_file() else None
            self.database = IdentityDatabase(str(self.get_parameter("database_path").value))
            self.extractor = FeatureExtractor(
                attribute_model=self.attribute_model,
                device=self.device,
                pose_model_path=str(model_dir / str(self.get_parameter("pose_model").value)),
            )
            self.matcher = IdentityMatcher(
                threshold=float(self.get_parameter("identity_threshold").value),
                ambiguity_margin=float(self.get_parameter("ambiguity_margin").value),
            )
            self.publisher = self.create_lifecycle_publisher(
                RecognizedPerson,
                str(self.get_parameter("output_topic").value),
                10,
            )
            self.subscription = self.create_subscription(
                Image,
                str(self.get_parameter("image_topic").value),
                self.image_callback,
                qos_profile_sensor_data,
            )
            self.profile_service = self.create_service(
                GetProfile,
                "/gpsr/identity/get_profile",
                self.get_profile_callback,
            )
            self.get_logger().info("[GPSR_LIFECYCLE] Human identity configured.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"GPSR human identity configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Enable tracking after activating the lifecycle publisher."""
        result = super().on_activate(state)
        if result == TransitionCallbackReturn.SUCCESS:
            self.skill_active = True
        return result

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Stop inference before publisher deactivation."""
        self.skill_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Close persistence and unload every configured model."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Release resources during process shutdown."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Restore a clean unconfigured baseline after failure."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> None:
        """Special function: Destroy ROS entities and close model helper resources."""
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
            self.publisher = None
        if self.profile_service is not None:
            self.destroy_service(self.profile_service)
            self.profile_service = None
        if self.extractor is not None and self.extractor.pose_landmarker is not None:
            self.extractor.pose_landmarker.close()
        if self.database is not None:
            self.database.close()
        self.bridge = None
        self.person_model = None
        self.attribute_model = None
        self.database = None
        self.extractor = None
        self.matcher = None
        self.tracking_session_id = ""

    def get_profile_callback(self, request: Any, response: Any) -> Any:
        """Special function: Resolve registered identity metadata without exposing the DB."""
        if self.database is None:
            response.message = "GPSR identity database is not configured."
            return response
        raw_profile_id = str(request.profile_id).strip()
        try:
            person_id = int(raw_profile_id.rsplit(":", 1)[-1])
        except ValueError:
            response.message = f"Invalid GPSR identity profile id: {raw_profile_id}."
            return response
        stored = self.database.get_person_profile(person_id)
        if stored is None:
            response.message = f"GPSR identity profile {raw_profile_id} was not found."
            return response
        profile = PersonProfile()
        profile.profile_id = f"human_identity:{person_id}"
        profile.role = "guest"
        profile.name = str(stored["person_name"])
        profile.age = int(stored["age"])
        profile.favorite_drink = str(stored["favourite_drink"])
        profile.is_confirmed = True
        profile.stamp = self.get_clock().now().to_msg()
        response.success = True
        response.profile = profile
        response.message = f"Retrieved GPSR identity profile {profile.profile_id}."
        return response

    def image_callback(self, image_message: Image) -> None:
        """Special function: Track, identify, and publish every visible person."""
        if not self.skill_active or self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
            with self.inference_lock:
                observations = rank_bounding_boxes(self.observe(frame))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"GPSR human identity inference failed: {exc}")
            return
        nearest_person_id = 0
        if observations and observations[0]["match"].status == "RECOGNIZED":
            nearest_person_id = int(observations[0]["match"].person_id)
        for observation in observations:
            self.publish_observation(image_message, observation, nearest_person_id)

    def observe(self, frame: np.ndarray) -> list[dict[str, Any]]:
        """Special function: Run persistent YOLO tracking and reusable identity features."""
        results = self.person_model.track(
            frame,
            persist=True,
            classes=[0],
            conf=self.confidence_threshold,
            verbose=False,
            device=self.device,
        )
        if not results or results[0].boxes is None or results[0].boxes.id is None:
            return []
        height, width = frame.shape[:2]
        image_area = float(width * height)
        observations: list[dict[str, Any]] = []
        for track_id, raw_box in zip(
            results[0].boxes.id.int().cpu().tolist(),
            results[0].boxes.xyxy.cpu().numpy(),
        ):
            box = self.extractor._clip_box(tuple(raw_box.astype(int).tolist()), width, height)
            area = float(max(0, box[2] - box[0]) * max(0, box[3] - box[1]))
            features = self.extractor.extract(frame, box, int(track_id))
            observations.append({
                "track_id": int(track_id),
                "box": box,
                "area": area,
                "ratio": area / image_area if image_area else 0.0,
                "features": features,
                "match": self.matcher.match(features, self.database.iter_prototypes()),
            })
        return observations

    def publish_observation(
        self,
        image_message: Image,
        observation: dict[str, Any],
        nearest_person_id: int,
    ) -> None:
        """Special function: Publish canonical track and optional registered profile metadata."""
        match = observation["match"]
        features = observation["features"]
        message = RecognizedPerson()
        message.header = image_message.header
        message.status_text = str(match.status)
        message.status = {
            "RECOGNIZED": RecognizedPerson.RECOGNIZED,
            "AMBIGUOUS": RecognizedPerson.AMBIGUOUS,
        }.get(match.status, RecognizedPerson.UNKNOWN)
        message.person_id = int(match.person_id) if match.status == "RECOGNIZED" else 0
        message.track_id = int(observation["track_id"])
        message.tracking_session_id = self.tracking_session_id
        message.person_uuid = ""
        profile = self.database.get_person_profile(message.person_id) if message.person_id > 0 else None
        if profile is not None:
            message.person_uuid = str(profile.get("canonical_person_uuid", ""))
            message.person_name = str(profile["person_name"])
            message.age = max(0, min(int(profile["age"]), 65535))
            message.favourite_drink = str(profile["favourite_drink"])
            message.has_registered_profile = True
        visual = features.visual_attributes
        message.wearing_glasses = int(visual.wearing_glasses)
        message.glasses_confidence = float(visual.glasses_confidence)
        message.shirt_colour = str(visual.shirt_colour)
        message.shirt_colour_confidence = float(visual.shirt_colour_confidence)
        message.wearing_face_mask = int(visual.wearing_face_mask)
        message.face_mask_confidence = float(visual.face_mask_confidence)
        message.hair_colour = str(visual.hair_colour)
        message.hair_colour_confidence = float(visual.hair_colour_confidence)
        message.is_nearest = int(observation["rank"]) == 1
        message.proximity_rank = int(observation["rank"])
        message.view_area_ratio = float(observation["ratio"])
        message.proximity_method = "bbox_area"
        message.nearest_track_id = int(observation["track_id"]) if message.is_nearest else 0
        message.nearest_person_id = int(nearest_person_id)
        message.final_score = float(match.final_score)
        message.second_score = float(match.second_score)
        message.body_score = float(match.body_score)
        message.upper_clothing_score = float(match.upper_score)
        message.lower_clothing_score = float(match.lower_score)
        message.object_score = float(match.object_score)
        message.body_quality = float(features.body_quality)
        message.upper_clothing_quality = float(features.upper_quality)
        message.lower_clothing_quality = float(features.lower_quality)
        message.object_quality = float(features.object_quality)
        message.visual_attribute_quality = float(visual.overall_quality)
        message.bbox_xyxy = [float(value) for value in observation["box"]]
        message.visual_summary = f"track={message.track_id}; person={message.person_id}; shirt={message.shirt_colour}"
        self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Special function: Run the canonical GPSR identity lifecycle resource."""
    rclpy.init(args=args)
    node = GpsrHumanIdentityLifecycleNode()
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
