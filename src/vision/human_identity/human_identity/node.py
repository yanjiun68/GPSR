"""Section: Standalone ROS 2 human identity node."""

from __future__ import annotations

import asyncio
from pathlib import Path
from threading import Lock, RLock
from typing import Any
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from vision.action import RegisterPerson
from vision.msg import RecognizedPerson
from vision.srv import UpdatePersonProfile

from .database import IdentityDatabase
from .feature_extractor import FeatureExtractor
from .feature_types import PersonFeatures
from .matcher import IdentityMatcher
from .registration_manager import RegistrationManager, rank_bounding_boxes, resolve_profile_target


class HumanIdentityNode(Node):
    """Section: Track visible people and expose persistent identity registration."""

    def __init__(self) -> None:
        """Special function: Configure models, persistence, ROS interfaces, and shared state."""
        super().__init__("human_identity_node")

        # Section: Declare all runtime configuration before resolving package assets.
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/human_identity/recognized_people")
        self.declare_parameter("person_model", "yolo11n.pt")
        self.declare_parameter("attribute_model", "human_attributes.pt")
        self.declare_parameter("pose_model", "pose_landmarker_full.task")
        self.declare_parameter("database_path", str(Path.home() / ".ros/vision/human_identity.db"))
        self.declare_parameter("device", "0")
        self.declare_parameter("publish_interval", 0.5)
        self.declare_parameter("identity_threshold", 0.72)
        self.declare_parameter("ambiguity_margin", 0.08)
        self.declare_parameter("nearest_timeout", 2.0)
        self.declare_parameter("show_gui", True)
        self.declare_parameter("gui_window_name", "Human Identity")
        self.declare_parameter("gui_scale", 1.0)

        # Section: Load models from the package's installed model directory.
        share = Path(get_package_share_directory("vision"))
        model_dir = share / "models"
        self.device = str(self.get_parameter("device").value)
        person_model_path = model_dir / str(self.get_parameter("person_model").value)
        if not person_model_path.is_file():
            raise FileNotFoundError(f"Person model not found: {person_model_path}")
        self.person_model = YOLO(str(person_model_path))
        attribute_path = model_dir / str(self.get_parameter("attribute_model").value)
        self.attribute_model = YOLO(str(attribute_path)) if attribute_path.is_file() else None

        # Section: Construct stateful helpers without touching existing perception nodes.
        self.bridge = CvBridge()
        self.database = IdentityDatabase(str(self.get_parameter("database_path").value))
        self.registration = RegistrationManager()
        self.extractor = FeatureExtractor(
            attribute_model=self.attribute_model,
            device=self.device,
            pose_model_path=str(model_dir / str(self.get_parameter("pose_model").value)),
        )
        self.matcher = IdentityMatcher(
            threshold=float(self.get_parameter("identity_threshold").value),
            ambiguity_margin=float(self.get_parameter("ambiguity_margin").value),
        )

        # Section: Protect model inference and cross-callback observation state independently.
        self._inference_lock = Lock()
        self._gui_lock = Lock()
        self._state_lock = RLock()
        self._registration_start_lock = Lock()
        self.latest_features: dict[int, PersonFeatures] = {}
        self.latest_feature_timestamps_ns: dict[int, int] = {}
        self.current_nearest_track_id = 0
        self.current_nearest_timestamp_ns = 0
        self.tracking_session_id = uuid4().hex
        self.last_publish_ns = 0
        publish_interval = float(self.get_parameter("publish_interval").value)
        nearest_timeout = float(self.get_parameter("nearest_timeout").value)
        self.publish_interval_ns = int(publish_interval * 1_000_000_000)
        self.nearest_timeout_ns = int(nearest_timeout * 1_000_000_000)
        self.show_gui = bool(self.get_parameter("show_gui").value)
        self.gui_window_name = str(self.get_parameter("gui_window_name").value)
        self.gui_scale = max(0.25, float(self.get_parameter("gui_scale").value))

        # Section: Create ROS entities in a reentrant group for action and camera concurrency.
        self.callback_group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(
            RecognizedPerson,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            5,
            callback_group=self.callback_group,
        )
        self.profile_service = self.create_service(
            UpdatePersonProfile,
            "/human_identity/update_person_profile",
            self.update_profile_callback,
            callback_group=self.callback_group,
        )
        self.registration_server = ActionServer(
            self,
            RegisterPerson,
            "/human_identity/register_person",
            execute_callback=self.execute_registration,
            cancel_callback=self.cancel_registration_callback,
            callback_group=self.callback_group,
        )
        image_topic = self.get_parameter("image_topic").value
        attribute_status = (
            "enabled"
            if self.attribute_model is not None
            else "unknown (model unavailable)"
        )
        self.get_logger().info(
            f"Human identity ready: image={image_topic}, attributes={attribute_status}"
        )

    def image_callback(self, image_msg: Image) -> None:
        """Special function: Track, rank, match, and publish visible people."""
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Image conversion failed: {exc}")
            return
        try:
            with self._inference_lock:
                observations = self._observe_frame(frame)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Human identity inference failed: {exc}")
            return
        if not observations:
            self.display_gui(frame, [])
            return
        observations = rank_bounding_boxes(observations)
        now_ns = self.get_clock().now().nanoseconds
        with self._state_lock:
            for observation in observations:
                self.latest_features[observation["track_id"]] = observation["features"]
                self.latest_feature_timestamps_ns[observation["track_id"]] = now_ns
            self.current_nearest_track_id = observations[0]["track_id"]
            self.current_nearest_timestamp_ns = now_ns
            should_publish = now_ns - self.last_publish_ns >= self.publish_interval_ns
            if should_publish:
                self.last_publish_ns = now_ns
        self.display_gui(frame, observations)
        if not should_publish:
            return
        nearest_match = observations[0]["match"]
        nearest_person_id = (
            int(nearest_match.person_id)
            if nearest_match.status == "RECOGNIZED"
            else 0
        )
        for observation in observations:
            self.publish_observation(image_msg, observation, nearest_person_id)

    def _observe_frame(self, frame: np.ndarray) -> list[dict[str, Any]]:
        """Special function: Run serialized YOLO tracking and feature extraction for one frame."""
        results = self.person_model.track(
            frame,
            persist=True,
            classes=[0],
            conf=0.40,
            verbose=False,
            device=self.device,
        )
        if not results or results[0].boxes is None or results[0].boxes.id is None:
            return []
        boxes = results[0].boxes.xyxy.cpu().numpy()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        image_height, image_width = frame.shape[:2]
        image_area = float(image_width * image_height)
        observations: list[dict[str, Any]] = []
        for track_id, raw_box in zip(track_ids, boxes):
            raw_coordinates = tuple(raw_box.astype(int).tolist())
            x1, y1, x2, y2 = self.extractor._clip_box(
                raw_coordinates,
                image_width,
                image_height,
            )
            area = float(max(0, x2 - x1) * max(0, y2 - y1))
            features = self.extractor.extract(frame, (x1, y1, x2, y2), int(track_id))
            match = self.matcher.match(features, self.database.iter_prototypes())
            observations.append(
                {
                    "track_id": int(track_id),
                    "box": (x1, y1, x2, y2),
                    "area": area,
                    "ratio": area / image_area if image_area else 0.0,
                    "features": features,
                    "match": match,
                }
            )
        return observations

    def display_gui(
        self,
        frame: np.ndarray,
        observations: list[dict[str, Any]],
    ) -> None:
        """Special function: Display an annotated live identity preview when enabled."""
        if not self.show_gui:
            return
        annotated = frame.copy()
        active = self.registration.get_active()

        # Section: Draw a stable status banner above all per-person annotations.
        nearest_track_id = observations[0]["track_id"] if observations else 0
        registration_text = (
            f"registration: person {active.person_id}, track {active.track_id}"
            if active is not None
            else "registration: inactive"
        )
        status_text = (
            f"visible: {len(observations)}  nearest track: {nearest_track_id}  "
            f"{registration_text}"
        )
        cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 34), (25, 25, 25), -1)
        cv2.putText(
            annotated,
            status_text,
            (10, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # Section: Draw identity, proximity, registration, and attribute details per person.
        for observation in observations:
            self._draw_observation(annotated, observation, active)

        # Section: Scale the preview without changing inference coordinates.
        if self.gui_scale != 1.0:
            annotated = cv2.resize(
                annotated,
                None,
                fx=self.gui_scale,
                fy=self.gui_scale,
                interpolation=cv2.INTER_AREA,
            )
        try:
            with self._gui_lock:
                cv2.imshow(self.gui_window_name, annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.show_gui = False
                    cv2.destroyWindow(self.gui_window_name)
                    self.get_logger().info("Human identity GUI closed")
        except cv2.error as exc:
            self.show_gui = False
            self.get_logger().error(f"Human identity GUI disabled: {exc}")

    def _draw_observation(
        self,
        frame: np.ndarray,
        observation: dict[str, Any],
        active: Any,
    ) -> None:
        """Special function: Draw one tracked person's identity and attributes."""
        x1, y1, x2, y2 = observation["box"]
        match = observation["match"]
        visual = observation["features"].visual_attributes
        is_nearest = observation["rank"] == 1
        is_registration_target = active is not None and active.track_id == observation["track_id"]

        # Section: Select colors that distinguish identity and registration state.
        status_colors = {
            "RECOGNIZED": (70, 210, 80),
            "AMBIGUOUS": (0, 170, 255),
            "UNKNOWN": (180, 180, 180),
        }
        color = status_colors.get(match.status, (180, 180, 180))
        if is_nearest:
            color = (255, 220, 0)
        if is_registration_target:
            color = (255, 0, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3 if is_nearest else 2)

        # Section: Resolve a stored name only after a positive recognition decision.
        profile = (
            self.database.get_person_profile(int(match.person_id))
            if match.status == "RECOGNIZED"
            else None
        )
        person_name = profile["person_name"] if profile and profile["person_name"] else "Unknown"
        person_id = int(match.person_id) if match.status == "RECOGNIZED" else 0
        proximity = "NEAREST" if is_nearest else f"RANK {observation['rank']}"
        registration = "  REGISTERING" if is_registration_target else ""
        first_line = (
            f"{proximity}{registration}  track={observation['track_id']}  "
            f"person={person_id} {person_name}"
        )
        second_line = (
            f"{match.status} score={match.final_score:.2f} "
            f"second={match.second_score:.2f}"
        )
        third_line = (
            f"shirt={visual.shirt_colour} hair={visual.hair_colour} "
            f"glasses={self._attribute_label(visual.wearing_glasses)} "
            f"mask={self._attribute_label(visual.wearing_face_mask)}"
        )
        self._draw_label_block(frame, x1, y1, color, [first_line, second_line, third_line])

    @staticmethod
    def _attribute_label(value: int) -> str:
        """Special function: Convert a three-state attribute into compact GUI text."""
        return {1: "yes", 0: "no", -1: "unknown"}.get(int(value), "unknown")

    @staticmethod
    def _draw_label_block(
        frame: np.ndarray,
        x: int,
        y: int,
        color: tuple[int, int, int],
        lines: list[str],
    ) -> None:
        """Special function: Draw readable multiline text within image boundaries."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.48
        thickness = 1
        line_height = 19
        widths = [cv2.getTextSize(line, font, scale, thickness)[0][0] for line in lines]
        block_width = min(max(widths) + 12, frame.shape[1])
        block_height = line_height * len(lines) + 8
        left = max(0, min(x, frame.shape[1] - block_width))
        top = (
            y - block_height
            if y >= block_height + 36
            else min(y + 3, frame.shape[0] - block_height)
        )
        top = max(35, top)
        right = min(frame.shape[1], left + block_width)
        bottom = min(frame.shape[0], top + block_height)
        cv2.rectangle(frame, (left, top), (right, bottom), (20, 20, 20), -1)
        cv2.rectangle(frame, (left, top), (right, bottom), color, 1)
        for index, line in enumerate(lines):
            clipped_line = line
            while clipped_line and cv2.getTextSize(
                clipped_line, font, scale, thickness
            )[0][0] > block_width - 12:
                clipped_line = clipped_line[:-1]
            cv2.putText(
                frame,
                clipped_line,
                (left + 6, top + 17 + index * line_height),
                font,
                scale,
                (245, 245, 245),
                thickness,
                cv2.LINE_AA,
            )

    def publish_observation(
        self,
        image_msg: Image,
        observation: dict[str, Any],
        nearest_person_id: int,
    ) -> None:
        """Special function: Convert one matched observation into the public ROS message."""
        match = observation["match"]
        features = observation["features"]
        msg = RecognizedPerson()
        msg.header = image_msg.header
        msg.status_text = match.status
        msg.person_id = int(match.person_id) if match.status == "RECOGNIZED" else 0
        msg.track_id = int(observation["track_id"])
        msg.tracking_session_id = self.tracking_session_id
        msg.person_uuid = ""
        msg.status = {
            "RECOGNIZED": RecognizedPerson.RECOGNIZED,
            "AMBIGUOUS": RecognizedPerson.AMBIGUOUS,
        }.get(match.status, RecognizedPerson.UNKNOWN)

        # Section: Add stored profile fields only for a recognized positive ID.
        profile = self.database.get_person_profile(msg.person_id)
        if profile is not None:
            msg.person_uuid = str(profile.get("canonical_person_uuid", ""))
            msg.person_name = profile["person_name"]
            msg.age = max(0, min(int(profile["age"]), 65535))
            msg.favourite_drink = profile["favourite_drink"]
            msg.has_registered_profile = True

        # Section: Add pose-guided visual attributes without converting unknown into false.
        visual = features.visual_attributes
        msg.wearing_glasses = visual.wearing_glasses
        msg.glasses_confidence = visual.glasses_confidence
        msg.shirt_colour = visual.shirt_colour
        msg.shirt_colour_confidence = visual.shirt_colour_confidence
        msg.wearing_face_mask = visual.wearing_face_mask
        msg.face_mask_confidence = visual.face_mask_confidence
        msg.hair_colour = visual.hair_colour
        msg.hair_colour_confidence = visual.hair_colour_confidence
        msg.visual_attribute_quality = visual.overall_quality

        # Section: Publish nearest-person and locked-registration context for every visible person.
        msg.is_nearest = observation["rank"] == 1
        msg.proximity_rank = int(observation["rank"])
        msg.view_area_ratio = float(observation["ratio"])
        msg.proximity_method = "bbox_area"
        with self._state_lock:
            msg.nearest_track_id = self.current_nearest_track_id
        msg.nearest_person_id = nearest_person_id
        active = self.registration.get_active()
        if active is not None:
            msg.registration_active = True
            msg.active_registration_person_id = active.person_id
            msg.active_registration_track_id = active.track_id
            msg.is_registration_target = active.track_id == msg.track_id

        # Section: Publish component scores and observation qualities for downstream diagnosis.
        msg.final_score = match.final_score
        msg.second_score = match.second_score
        msg.body_score = match.body_score
        msg.upper_clothing_score = match.upper_score
        msg.lower_clothing_score = match.lower_score
        msg.object_score = match.object_score
        msg.body_quality = features.body_quality
        msg.upper_clothing_quality = features.upper_quality
        msg.lower_clothing_quality = features.lower_quality
        msg.object_quality = features.object_quality
        msg.bbox_xyxy = [float(value) for value in observation["box"]]
        name = msg.person_name or "Unknown person"
        nearest_text = " nearest" if msg.is_nearest else ""
        msg.visual_summary = (
            f"{name}{nearest_text}; shirt={msg.shirt_colour}; hair={msg.hair_colour}; "
            f"glasses={msg.wearing_glasses}; mask={msg.wearing_face_mask}; "
            f"drink={msg.favourite_drink or 'unknown'}"
        )
        self.publisher.publish(msg)

    def update_profile_callback(self, request: Any, response: Any) -> Any:
        """Special function: Apply profile fields to an ID or locked target."""
        try:
            person_id, track_id = resolve_profile_target(
                bool(request.is_nearest),
                int(request.person_id),
                self.registration,
            )
            self.database.update_person_profile(
                person_id=person_id,
                person_name=request.person_name,
                age=int(request.age),
                favourite_drink=request.favourite_drink,
                update_name=request.update_name,
                update_age=request.update_age,
                update_favourite_drink=request.update_favourite_drink,
            )
            response.success = True
            response.resolved_person_id = person_id
            response.resolved_track_id = track_id
            response.message = f"Updated person {person_id}"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.resolved_person_id = 0
            response.resolved_track_id = 0
            response.message = str(exc)
        return response

    @staticmethod
    def cancel_registration_callback(_goal_handle: Any) -> CancelResponse:
        """Special function: Accept action cancellation for cooperative shutdown of collection."""
        return CancelResponse.ACCEPT

    async def execute_registration(self, goal_handle: Any) -> RegisterPerson.Result:
        """Special function: Lock one track, collect stable samples, and save one prototype."""
        goal = goal_handle.request
        result = RegisterPerson.Result()
        context = None
        person_id = 0
        track_id = 0
        try:
            # Section: Snapshot the requested track and allocate/start registration atomically.
            with self._registration_start_lock:
                if self.registration.get_active() is not None:
                    raise RuntimeError("A person registration is already active")
                with self._state_lock:
                    track_id = (
                        self.current_nearest_track_id
                        if goal.use_nearest
                        else int(goal.requested_track_id)
                    )
                    observation_ns = self.latest_feature_timestamps_ns.get(track_id, 0)
                    observation_age_ns = self.get_clock().now().nanoseconds - observation_ns
                    sample_exists = track_id in self.latest_features
                if observation_age_ns > self.nearest_timeout_ns:
                    raise RuntimeError("Requested track is stale")
                if track_id <= 0 or not sample_exists:
                    raise RuntimeError("Requested track is not visible")
                person_id = int(goal.person_id)
                if person_id == 0:
                    person_id = self.database.create_registration_profile(
                        str(getattr(goal, "requested_person_uuid", ""))
                    )
                elif not self.database.person_exists(person_id):
                    raise RuntimeError(f"Person {person_id} does not exist")
                elif str(getattr(goal, "requested_person_uuid", "")).strip():
                    self.database.bind_canonical_person(
                        person_id,
                        str(goal.requested_person_uuid),
                    )
                context = self.registration.start(
                    person_id,
                    track_id,
                    self.get_clock().now().nanoseconds,
                )

            # Section: Collect the latest feature for the locked track at a bounded cadence.
            start_ns = self.get_clock().now().nanoseconds
            duration_ns = int(max(1.0, float(goal.capture_duration)) * 1_000_000_000)
            collected: list[PersonFeatures] = []
            last_sample: PersonFeatures | None = None
            while self.get_clock().now().nanoseconds - start_ns < duration_ns:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.message = "Registration cancelled"
                    return result
                with self._state_lock:
                    sample = self.latest_features.get(track_id)
                if sample is not None and sample is not last_sample:
                    collected.append(sample)
                    last_sample = sample
                feedback = RegisterPerson.Feedback()
                feedback.active_person_id = person_id
                feedback.active_track_id = track_id
                feedback.frames_collected = len(collected)
                elapsed_ns = self.get_clock().now().nanoseconds - start_ns
                feedback.elapsed_time = elapsed_ns / 1_000_000_000.0
                feedback.pose_quality = (
                    sample.body_quality if sample is not None else 0.0
                )
                feedback.current_phase = (
                    "collecting_walking_features"
                    if goal.require_walking
                    else "collecting_features"
                )
                goal_handle.publish_feedback(feedback)
                await asyncio.sleep(0.1)
            if not collected:
                raise RuntimeError("No valid features collected")
            if goal.require_walking and not self._walking_observed(collected):
                raise RuntimeError("Walking motion was not observed")

            # Section: Aggregate and persist only after the complete capture succeeds.
            prototype = self.extractor.aggregate(collected)
            self.database.add_prototype(person_id, prototype)
            goal_handle.succeed()
            result.success = True
            result.person_id = person_id
            result.registered_track_id = track_id
            result.prototypes_saved = 1
            result.average_quality = float(np.mean([item.body_quality for item in collected]))
            profile = self.database.get_person_profile(person_id) or {}
            result.person_uuid = str(profile.get("canonical_person_uuid", ""))
            result.message = f"Registered person {person_id} from track {track_id}"
            return result
        except Exception as exc:  # noqa: BLE001
            goal_handle.abort()
            result.success = False
            result.person_id = person_id
            result.registered_track_id = track_id
            result.prototypes_saved = 0
            result.average_quality = 0.0
            result.message = str(exc)
            return result
        finally:
            if context is not None:
                self.registration.finish(context.registration_token)

    @staticmethod
    def _walking_observed(samples: list[PersonFeatures]) -> bool:
        """Special function: Require pose variation when walking is requested."""
        vectors = [item.body_vector for item in samples if item.body_vector.size]
        if len(vectors) < 3 or len({item.size for item in vectors}) != 1:
            return False
        return float(np.mean(np.std(np.stack(vectors), axis=0))) >= 0.01

    def destroy_node(self) -> bool:
        """Special function: Release action, model, and database resources before ROS shutdown."""
        self.registration_server.destroy()
        if self.show_gui:
            cv2.destroyAllWindows()
        if self.extractor.pose_landmarker is not None:
            self.extractor.pose_landmarker.close()
        self.database.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Special function: Run concurrent camera, service, and action callbacks."""
    rclpy.init(args=args)
    node = HumanIdentityNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
