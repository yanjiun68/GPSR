"""ROS 2 node that converts an existing metric localization into semantic state."""

# Standard-library section: use monotonic time for hysteresis and math for pose yaw.
import math
import time
from typing import Optional, Tuple

# ROS core section: provide node, QoS, parameters, AMCL messages, and TF consumption.
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# Package section: import generated interfaces and pure semantic-domain components.
from semantic_location_server.msg import SemanticLocation
from SemanticLocationServer.semantic_map import (
    ConfigError,
    SemanticClassifier,
    SemanticState,
    SemanticStateTracker,
    load_semantic_map,
)
from semantic_location_server.srv import GetSemanticLocation


# Quaternion helper: extract planar yaw from a normalized or non-normalized quaternion.
def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw using the standard quaternion-to-Euler relationship."""

    sin_yaw = 2.0 * ((w * z) + (x * y))
    cos_yaw = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(sin_yaw, cos_yaw)


# Node section: own semantic configuration and APIs while consuming external localization.
class SemanticLocationServerNode(Node):
    """Publish/query polygon-derived semantic location from TF or AMCL."""

    def __init__(self) -> None:
        """Declare parameters, validate configuration, and connect the selected pose source."""

        super().__init__("semantic_location_server")

        # Parameter section: defaults follow the audited navigation stack conventions.
        default_map = str(
            get_package_share_directory("semantic_location_server")
            + "/config/semantic_locations.yaml"
        )
        self.declare_parameter("semantic_map_file", default_map)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("pose_source", "tf")
        self.declare_parameter("amcl_topic", "/amcl_pose")
        self.declare_parameter("update_rate_hz", 5.0)
        self.declare_parameter("stable_duration_sec", 1.0)
        self.declare_parameter("use_localization_covariance", True)
        self.declare_parameter("max_xy_variance", 0.5)
        self.declare_parameter("pose_timeout_sec", 1.0)
        self.declare_parameter("publish_debug_markers", False)

        # Configuration section: fail node startup rather than running with ambiguity.
        semantic_map_file = str(
            self.get_parameter("semantic_map_file").value
        ).strip()
        self._semantic_map = load_semantic_map(semantic_map_file)
        self._map_frame = str(self.get_parameter("map_frame").value).strip()
        self._base_frame = str(self.get_parameter("base_frame").value).strip()
        if not self._map_frame or not self._base_frame:
            raise ConfigError("map_frame and base_frame parameters must be non-empty.")
        if self._map_frame != self._semantic_map.map_frame:
            raise ConfigError(
                "map_frame parameter does not match frames.map in the semantic map: "
                f"{self._map_frame!r} != {self._semantic_map.map_frame!r}"
            )
        if self._base_frame != self._semantic_map.base_frame:
            raise ConfigError(
                "base_frame parameter does not match frames.base in the semantic map: "
                f"{self._base_frame!r} != {self._semantic_map.base_frame!r}"
            )

        # Runtime section: validate numeric controls before constructing timers.
        self._pose_source = str(
            self.get_parameter("pose_source").value
        ).strip().lower()
        if self._pose_source not in {"tf", "amcl"}:
            raise ConfigError("pose_source must be either 'tf' or 'amcl'.")
        self._update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self._max_xy_variance = float(
            self.get_parameter("max_xy_variance").value
        )
        self._pose_timeout_sec = float(
            self.get_parameter("pose_timeout_sec").value
        )
        if self._update_rate_hz <= 0.0:
            raise ConfigError("update_rate_hz must be greater than zero.")
        if self._max_xy_variance < 0.0:
            raise ConfigError("max_xy_variance must be non-negative.")
        if self._pose_timeout_sec <= 0.0:
            raise ConfigError("pose_timeout_sec must be greater than zero.")
        self._use_covariance = bool(
            self.get_parameter("use_localization_covariance").value
        )
        self._publish_debug_markers = bool(
            self.get_parameter("publish_debug_markers").value
        )

        # Domain section: separate instantaneous geometry from stable runtime state.
        self._classifier = SemanticClassifier(self._semantic_map)
        self._tracker = SemanticStateTracker(
            float(self.get_parameter("stable_duration_sec").value)
        )

        # Publisher section: retain the latest state for late-joining consumers.
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_publisher = self.create_publisher(
            SemanticLocation,
            "/semantic_location/current",
            state_qos,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            "/semantic_location/debug_markers",
            state_qos,
        )
        self._service = self.create_service(
            GetSemanticLocation,
            "/semantic_location/get_current",
            self._handle_get_current,
        )

        # Pose-state section: retain only the latest externally produced metric pose.
        self._latest_amcl_pose: Optional[Tuple[float, float, float]] = None
        self._latest_amcl_variance = 0.0
        self._latest_amcl_received = 0.0
        self._covariance_available = False
        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        self._amcl_subscription = None

        # Pose-source section: connect exactly one configured localization adapter.
        if self._pose_source == "tf":
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            if self._use_covariance:
                self.get_logger().warning(
                    "TF pose source has no AMCL covariance; uncertainty checking is disabled."
                )
        else:
            amcl_topic = str(self.get_parameter("amcl_topic").value).strip()
            if not amcl_topic:
                raise ConfigError("amcl_topic must be non-empty for pose_source=amcl.")
            self._amcl_subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                amcl_topic,
                self._on_amcl_pose,
                10,
            )

        # Initial-state section: make service responses meaningful before the first pose.
        self._current_message = self._make_message(
            SemanticState(status="UNKNOWN"),
            pose=None,
            covariance_available=False,
            max_variance=0.0,
        )
        self._state_publisher.publish(self._current_message)
        if self._publish_debug_markers:
            self._publish_markers()

        # Timer section: classify at a bounded rate independently of pose transport rate.
        self._timer = self.create_timer(
            1.0 / self._update_rate_hz,
            self._update,
        )
        self.get_logger().info(
            "Loaded semantic map from "
            f"{semantic_map_file} with {len(self._semantic_map.rooms)} rooms; "
            f"pose_source={self._pose_source}, frames={self._map_frame}->{self._base_frame}."
        )

    # AMCL callback: cache metric pose and the largest planar covariance diagonal.
    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        """Store the latest AMCL pose and x/y variance for timer-driven classification."""

        if message.header.frame_id and message.header.frame_id != self._map_frame:
            self.get_logger().warning(
                f"Ignoring AMCL pose in frame {message.header.frame_id!r}; "
                f"expected {self._map_frame!r}."
            )
            return
        pose = message.pose.pose
        self._latest_amcl_pose = (
            float(pose.position.x),
            float(pose.position.y),
            _yaw_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        covariance = message.pose.covariance
        self._latest_amcl_variance = max(float(covariance[0]), float(covariance[7]))
        self._covariance_available = True
        self._latest_amcl_received = time.monotonic()

    # TF helper: obtain the current map-to-base transform without publishing localization.
    def _read_tf_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return the current TF pose or None while the external transform is unavailable."""

        if self._tf_buffer is None:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(
                f"Waiting for TF {self._map_frame}->{self._base_frame}: {exc}"
            )
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            _yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    # Pose helper: select the configured source and report covariance metadata.
    def _read_pose(
        self,
    ) -> Tuple[Optional[Tuple[float, float, float]], bool, float]:
        """Return pose, covariance availability, and maximum planar variance."""

        if self._pose_source == "tf":
            return self._read_tf_pose(), False, 0.0
        if (
            self._latest_amcl_pose is None
            or time.monotonic() - self._latest_amcl_received > self._pose_timeout_sec
        ):
            return None, self._covariance_available, self._latest_amcl_variance
        return (
            self._latest_amcl_pose,
            self._covariance_available,
            self._latest_amcl_variance,
        )

    # Message helper: convert stable domain state and diagnostics into the typed ROS API.
    def _make_message(
        self,
        state: SemanticState,
        pose: Optional[Tuple[float, float, float]],
        covariance_available: bool,
        max_variance: float,
    ) -> SemanticLocation:
        """Build one complete semantic-location message."""

        message = SemanticLocation()
        message.stamp = self.get_clock().now().to_msg()
        message.frame_id = self._map_frame
        message.status = state.status
        message.room = state.room
        message.location = state.location
        message.nearest_waypoint = state.nearest_waypoint
        message.pose_available = pose is not None
        if pose is not None:
            message.x, message.y, message.yaw = pose
        message.covariance_available = covariance_available
        message.max_xy_variance = float(max_variance)
        return message

    # Timer callback: classify reliable poses and publish the current structured state.
    def _update(self) -> None:
        """Process one pose sample through classification, uncertainty, and hysteresis."""

        pose, covariance_available, max_variance = self._read_pose()
        if pose is None:
            state = SemanticState(status="UNKNOWN")
        else:
            match = self._classifier.classify(pose[0], pose[1])
            reliable = not (
                self._pose_source == "amcl"
                and self._use_covariance
                and covariance_available
                and max_variance > self._max_xy_variance
            )
            state = self._tracker.update(
                match,
                now_sec=time.monotonic(),
                localization_reliable=reliable,
            )
        self._current_message = self._make_message(
            state,
            pose,
            covariance_available,
            max_variance,
        )
        self._state_publisher.publish(self._current_message)
        if self._publish_debug_markers:
            self._publish_markers()

    # Service callback: return the same latest snapshot published on the current topic.
    def _handle_get_current(
        self,
        _request: GetSemanticLocation.Request,
        response: GetSemanticLocation.Response,
    ) -> GetSemanticLocation.Response:
        """Copy the latest semantic message into the query response."""

        response.current = self._current_message
        return response

    # Marker helper: append a closed polygon line strip in the semantic map frame.
    def _polygon_marker(
        self,
        marker_id: int,
        namespace: str,
        name: str,
        polygon,
        red: float,
        green: float,
        blue: float,
    ) -> Marker:
        """Create one persistent polygon-outline marker with a semantic text label."""

        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.9
        marker.pose.orientation.w = 1.0
        for x, y in tuple(polygon) + (polygon[0],):
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.05
            marker.points.append(point)
        marker.text = name
        return marker

    # Marker publisher: visualize configured geometry without affecting classification.
    def _publish_markers(self) -> None:
        """Publish room and sublocation polygon outlines for optional RViz inspection."""

        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)
        marker_id = 1
        for room in self._semantic_map.rooms:
            marker_array.markers.append(
                self._polygon_marker(
                    marker_id,
                    "semantic_rooms",
                    room.name,
                    room.polygon,
                    0.1,
                    0.7,
                    1.0,
                )
            )
            marker_id += 1
            for location in room.locations:
                marker_array.markers.append(
                    self._polygon_marker(
                        marker_id,
                        "semantic_locations",
                        location.name,
                        location.polygon,
                        1.0,
                        0.7,
                        0.1,
                    )
                )
                marker_id += 1
        self._marker_publisher.publish(marker_array)


# Process entry point: initialize ROS, report startup validation failures, and shut down cleanly.
def main(args=None) -> None:
    """Run the semantic-location server until ROS shutdown or keyboard interruption."""

    rclpy.init(args=args)
    node = None
    try:
        node = SemanticLocationServerNode()
        rclpy.spin(node)
    except (ConfigError, ValueError) as exc:
        if node is not None:
            node.get_logger().fatal(f"Semantic location configuration rejected: {exc}")
        else:
            print(f"Semantic location configuration rejected: {exc}")
        raise
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
