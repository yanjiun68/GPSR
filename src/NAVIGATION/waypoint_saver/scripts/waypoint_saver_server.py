#!/usr/bin/env python3
"""Section: ROS service that saves a fresh map-to-robot TF pose as a waypoint."""

# Section: Import callback locking and hardware-independent waypoint helpers.
from __future__ import annotations

from pathlib import Path
from threading import Lock

from waypoint_saver_logic import (
    WaypointValidationError,
    atomic_write_waypoint_data,
    load_waypoint_data,
    resolve_yaml_path,
    update_waypoint_data,
    validate_transform_age,
)

# Section: Import ROS, TF, and generated-service interfaces.
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from waypoint_saver.srv import SaveWaypoint


def transform_stamp_seconds(transform: object) -> float:
    """Special function: Convert a TransformStamped header timestamp into fractional seconds."""
    # Section: Read ROS builtin_interfaces/Time fields without consulting a live clock.
    stamp = transform.header.stamp
    return float(stamp.sec) + (float(stamp.nanosec) / 1_000_000_000.0)


def pose_from_transform(transform: object) -> PoseStamped:
    """Special function: Convert a validated TransformStamped into a PoseStamped."""
    # Section: Copy the transform header and translation into the service response pose.
    pose = PoseStamped()
    pose.header = transform.header
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose


def lookup_fresh_robot_pose(
    tf_buffer: object,
    target_frame: str,
    robot_frame: str,
    transform_timeout_sec: float,
    max_transform_age_sec: float,
    now_sec: float,
) -> PoseStamped:
    """Special function: Look up the latest transform and reject stale, future, or zero stamps."""
    # Section: Request only the configured target-to-robot transform from the supplied buffer.
    transform = tf_buffer.lookup_transform(
        target_frame,
        robot_frame,
        Time(),
        timeout=Duration(seconds=float(transform_timeout_sec)),
    )

    # Section: Validate fabricated or live timestamps before exposing a pose for persistence.
    validate_transform_age(
        now_sec,
        transform_stamp_seconds(transform),
        max_transform_age_sec,
    )
    return pose_from_transform(transform)


def create_waypoint_entry(waypoint_id: int, name: str, pose: PoseStamped) -> dict:
    """Special function: Convert a captured pose to the named-navigation YAML schema."""
    # Section: Preserve every map-frame position and quaternion component as a float.
    return {
        "id": waypoint_id,
        "frame_id": pose.header.frame_id,
        "name": name,
        "pose": {
            "position": {
                "x": float(pose.pose.position.x),
                "y": float(pose.pose.position.y),
                "z": float(pose.pose.position.z),
            },
            "orientation": {
                "x": float(pose.pose.orientation.x),
                "y": float(pose.pose.orientation.y),
                "z": float(pose.pose.orientation.z),
                "w": float(pose.pose.orientation.w),
            },
        },
    }


def save_current_waypoint(
    tf_buffer: object,
    yaml_path: Path,
    name: str,
    requested_id: int,
    overwrite: bool,
    target_frame: str,
    robot_frame: str,
    transform_timeout_sec: float,
    max_transform_age_sec: float,
    now_sec: float,
) -> tuple[int, PoseStamped]:
    """Special function: Capture fresh mocked/live TF and atomically persist one safe update."""
    # Section: Capture and validate TF before preparing any file replacement.
    pose = lookup_fresh_robot_pose(
        tf_buffer,
        target_frame,
        robot_frame,
        transform_timeout_sec,
        max_transform_age_sec,
        now_sec,
    )

    # Section: Build an updated copy so collision rejection leaves the original data untouched.
    data = load_waypoint_data(yaml_path)
    updated_data, waypoint_id = update_waypoint_data(
        data,
        name,
        requested_id,
        overwrite,
        lambda assigned_id, assigned_name: create_waypoint_entry(
            assigned_id,
            assigned_name,
            pose,
        ),
    )

    # Section: Create missing directories/files or atomically replace the existing YAML.
    atomic_write_waypoint_data(yaml_path, updated_data)
    return waypoint_id, pose


class WaypointSaverServer(Node):
    """Special class: Save current TF poses without controlling or publishing robot state."""

    def __init__(self) -> None:
        """Special function: Configure the isolated service and read-only TF listener."""
        # Section: Initialize the ROS node and declare every standalone launch parameter.
        super().__init__("waypoint_saver_server")
        self.declare_parameter("service_name", "/waypoint_saver/save_current")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("transform_timeout_sec", 2.0)
        self.declare_parameter("max_transform_age_sec", 1.0)

        # Section: Cache validated frame and freshness settings used by every service request.
        self._target_frame = str(self.get_parameter("target_frame").value).strip()
        self._robot_frame = str(self.get_parameter("robot_frame").value).strip()
        self._transform_timeout_sec = float(
            self.get_parameter("transform_timeout_sec").value
        )
        self._max_transform_age_sec = float(
            self.get_parameter("max_transform_age_sec").value
        )
        if not self._target_frame or not self._robot_frame:
            raise ValueError("target_frame and robot_frame must not be empty")
        if self._transform_timeout_sec < 0.0:
            raise ValueError("transform_timeout_sec must be non-negative")
        if self._max_transform_age_sec < 0.0:
            raise ValueError("max_transform_age_sec must be non-negative")
        self._write_lock = Lock()

        # Section: Subscribe to TF only; this node does not publish transforms or motion commands.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Section: Keep service work separate from TF subscription callbacks.
        self._service_group = ReentrantCallbackGroup()
        service_name = str(self.get_parameter("service_name").value).strip()
        self._service = self.create_service(
            SaveWaypoint,
            service_name,
            self.save_waypoint_callback,
            callback_group=self._service_group,
        )

        # Section: Report the read-only TF contract and freshness threshold at startup.
        self.get_logger().info(
            f"Ready on {service_name}; reads {self._target_frame} -> {self._robot_frame}; "
            f"requires TF age <= {self._max_transform_age_sec:.3f}s; writes only request yaml_path."
        )

    def save_waypoint_callback(
        self,
        request: SaveWaypoint.Request,
        response: SaveWaypoint.Response,
    ) -> SaveWaypoint.Response:
        """Special function: Validate one request and atomically save its fresh current pose."""
        # Section: Reject invalid identity values without looking up TF or touching the filesystem.
        name = request.name.strip()
        if not name:
            response.message = "Waypoint name must not be empty"
            return response
        if request.id < -1:
            response.message = "Waypoint id must be -1 or non-negative"
            return response

        try:
            # Section: Preserve the absolute YAML path contract before acquiring the write lock.
            yaml_path = resolve_yaml_path(request.yaml_path)

            # Section: Serialize file updates while allowing TF subscription callbacks to run.
            with self._write_lock:
                now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
                waypoint_id, pose = save_current_waypoint(
                    self._tf_buffer,
                    yaml_path,
                    name,
                    int(request.id),
                    bool(request.overwrite),
                    self._target_frame,
                    self._robot_frame,
                    self._transform_timeout_sec,
                    self._max_transform_age_sec,
                    now_sec,
                )

            # Section: Return the exact pose and ID persisted to disk.
            response.success = True
            response.message = (
                f"Saved waypoint '{name}' with id {waypoint_id} to {yaml_path}"
            )
            response.id = waypoint_id
            response.pose = pose
            self.get_logger().info(response.message)
        except (OSError, TypeError, ValueError, WaypointValidationError, TransformException) as exc:
            # Section: Report lookup, freshness, schema, collision, or persistence rejection uniformly.
            response.message = f"Failed to save waypoint '{name}': {exc}"
            self.get_logger().error(response.message)
        return response


def main(args: list[str] | None = None) -> None:
    """Special function: Run concurrent TF and service callbacks until ROS shutdown."""
    # Section: Own the ROS context and a two-thread executor for TF plus service work.
    rclpy.init(args=args)
    node = WaypointSaverServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
