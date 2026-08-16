#!/usr/bin/env python3

# Section: Import standard-library support for atomic files and callback locking.
import os
from pathlib import Path
import tempfile
from threading import Lock

# Section: Import ROS, TF, generated-service, and YAML support.
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from waypoint_saver.srv import SaveWaypoint
from yaml import safe_dump, safe_load


class WaypointSaverServer(Node):
    """Save the current TF pose without controlling or publishing robot state."""

    # Special function: Configure the isolated service and read-only TF listener.
    def __init__(self) -> None:
        super().__init__("waypoint_saver_server")

        # Section: Declare settings without depending on my_robot_nav parameters.
        self.declare_parameter("service_name", "/waypoint_saver/save_current")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("transform_timeout_sec", 2.0)

        # Section: Cache frame settings used for every current-pose lookup.
        self._target_frame = str(self.get_parameter("target_frame").value).strip()
        self._robot_frame = str(self.get_parameter("robot_frame").value).strip()
        self._transform_timeout_sec = float(
            self.get_parameter("transform_timeout_sec").value
        )
        self._write_lock = Lock()

        # Section: Subscribe to TF only; this node does not publish robot transforms.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Section: Keep the service callback separate from TF subscription callbacks.
        self._service_group = ReentrantCallbackGroup()
        service_name = str(self.get_parameter("service_name").value).strip()
        self._service = self.create_service(
            SaveWaypoint,
            service_name,
            self.save_waypoint_callback,
            callback_group=self._service_group,
        )

        self.get_logger().info(
            f"Ready on {service_name}; reads {self._target_frame} -> "
            f"{self._robot_frame} and writes only the request yaml_path."
        )

    # Special function: Validate and normalize the caller-selected YAML path.
    @staticmethod
    def _resolve_yaml_path(raw_path: str) -> Path:
        expanded_path = Path(raw_path.strip()).expanduser()
        if not raw_path.strip():
            raise ValueError("yaml_path must not be empty")
        if not expanded_path.is_absolute():
            raise ValueError("yaml_path must be an absolute path")
        if expanded_path.suffix.lower() not in {".yaml", ".yml"}:
            raise ValueError("yaml_path must end in .yaml or .yml")
        return expanded_path.resolve(strict=False)

    # Special function: Load the repository-compatible waypoint-list schema.
    @staticmethod
    def _load_waypoint_data(path: Path) -> dict:
        if not path.exists():
            return {"waypoints": []}
        if not path.is_file():
            raise ValueError(f"yaml_path is not a regular file: {path}")

        data = safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            raise ValueError("Waypoint YAML root must be a mapping")

        waypoints = data.setdefault("waypoints", [])
        if not isinstance(waypoints, list):
            raise ValueError("Waypoint YAML 'waypoints' field must be a list")
        return data

    # Special function: Write through a same-directory temporary file atomically.
    @staticmethod
    def _write_waypoint_data(path: Path, data: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        serialized = safe_dump(data, sort_keys=False, allow_unicode=True)
        temporary_path = None

        # Section: Flush the complete YAML before replacing the destination path.
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
            os.replace(temporary_path, path)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()

    # Special function: Capture the latest target-frame pose of the robot frame.
    def _current_robot_pose(self) -> PoseStamped:
        transform = self._tf_buffer.lookup_transform(
            self._target_frame,
            self._robot_frame,
            Time(),
            timeout=Duration(seconds=self._transform_timeout_sec),
        )

        # Section: Convert the TF transform into the service response pose.
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    # Special function: Convert a captured pose to the my_robot_nav YAML schema.
    @staticmethod
    def _create_waypoint_entry(
        waypoint_id: int,
        name: str,
        pose: PoseStamped,
    ) -> dict:
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

    # Special function: Collect valid numeric IDs already present in the YAML.
    @staticmethod
    def _existing_ids(waypoints: list) -> set:
        existing_ids = set()
        for entry in waypoints:
            if not isinstance(entry, dict):
                continue
            raw_id = str(entry.get("id", "")).strip()
            if raw_id.lstrip("-").isdigit():
                existing_ids.add(int(raw_id))
        return existing_ids

    # Special function: Save one named current pose into the exact requested path.
    def save_waypoint_callback(
        self,
        request: SaveWaypoint.Request,
        response: SaveWaypoint.Response,
    ) -> SaveWaypoint.Response:
        name = request.name.strip()
        if not name:
            response.message = "Waypoint name must not be empty"
            return response
        if request.id < -1:
            response.message = "Waypoint id must be -1 or non-negative"
            return response

        try:
            yaml_path = self._resolve_yaml_path(request.yaml_path)

            # Section: Serialize file updates while allowing TF callbacks to run.
            with self._write_lock:
                data = self._load_waypoint_data(yaml_path)
                waypoints = data["waypoints"]
                existing_ids = self._existing_ids(waypoints)
                waypoint_id = (
                    max(existing_ids, default=-1) + 1
                    if request.id == -1
                    else int(request.id)
                )
                conflicts = [
                    entry
                    for entry in waypoints
                    if isinstance(entry, dict)
                    and (
                        str(entry.get("name", "")).strip() == name
                        or str(entry.get("id", "")).strip() == str(waypoint_id)
                    )
                ]
                if conflicts and not request.overwrite:
                    response.message = (
                        f"Waypoint name '{name}' or id {waypoint_id} already "
                        "exists; set overwrite=true to replace it"
                    )
                    return response

                # Section: Capture TF before changing the caller-selected file.
                pose = self._current_robot_pose()
                if conflicts:
                    waypoints[:] = [
                        entry for entry in waypoints if entry not in conflicts
                    ]
                waypoints.append(
                    self._create_waypoint_entry(waypoint_id, name, pose)
                )
                self._write_waypoint_data(yaml_path, data)

            # Section: Return the exact pose and ID persisted to disk.
            response.success = True
            response.message = (
                f"Saved waypoint '{name}' with id {waypoint_id} to {yaml_path}"
            )
            response.id = waypoint_id
            response.pose = pose
            self.get_logger().info(response.message)
        except (OSError, TypeError, ValueError, TransformException) as exc:
            response.message = f"Failed to save waypoint '{name}': {exc}"
            self.get_logger().error(response.message)
        return response


# Special function: Run concurrent TF and service callbacks until ROS shutdown.
def main(args=None) -> None:
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
        rclpy.shutdown()


# Section: Provide the executable entry point installed by CMake.
if __name__ == "__main__":
    main()
