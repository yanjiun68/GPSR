#!/usr/bin/env python3

from pathlib import Path
from typing import Dict, Iterable

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from yaml import safe_load

from receptionist_interfaces.action import NavigateNamedPose


class NavigateNamedPoseServer(Node):
    def __init__(self) -> None:
        super().__init__("navigate_named_pose_server")

        default_yaml = self._default_waypoint_yaml()
        self.declare_parameter("waypoints_file", default_yaml)
        self.declare_parameter("action_name", "/nav/navigate_named_pose")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("navigate_to_pose_server_timeout_sec", 30.0)

        self._waypoints_file = Path(str(self.get_parameter("waypoints_file").value))
        self._waypoint_map = self._load_waypoint_map(self._waypoints_file)

        nav_action_name = str(self.get_parameter("navigate_to_pose_action").value)
        self._nav_action_name = nav_action_name
        self._nav_server_timeout_sec = float(
            self.get_parameter("navigate_to_pose_server_timeout_sec").value
        )
        self._nav_client = ActionClient(self, NavigateToPose, nav_action_name)
        self._goal_handles: Dict[str, object] = {}

        action_name = str(self.get_parameter("action_name").value)
        self._server = ActionServer(
            self,
            NavigateNamedPose,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            f"NavigateNamedPose server ready on {action_name} using "
            f"{self._waypoints_file} with {len(self._waypoint_map)} names."
        )

    def _default_waypoint_yaml(self) -> str:
        try:
            share = Path(get_package_share_directory("my_robot_nav"))
            return str(share / "config" / "waypoints.yaml")
        except PackageNotFoundError:
            return ""

    def _load_waypoint_map(self, path: Path) -> Dict[str, PoseStamped]:
        waypoint_map: Dict[str, PoseStamped] = {}

        if not path.exists():
            self.get_logger().warning(
                f"Waypoints file {path} does not exist. "
                "Named pose navigation will reject all names."
            )
            return waypoint_map

        data = safe_load(path.read_text()) or {}
        waypoints = data.get("waypoints", [])
        if not isinstance(waypoints, list):
            self.get_logger().warning(
                f"Waypoints file {path} is missing a 'waypoints' list."
            )
            return waypoint_map

        for entry in waypoints:
            if not isinstance(entry, dict):
                continue
            waypoint_id = str(entry.get("id", "")).strip()
            frame_id = str(entry.get("frame_id", "map")).strip() or "map"
            pose_data = entry.get("pose", {})
            position = pose_data.get("position", {})
            orientation = pose_data.get("orientation", {})

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(position.get("x", 0.0))
            pose.pose.position.y = float(position.get("y", 0.0))
            pose.pose.position.z = float(position.get("z", 0.0))
            pose.pose.orientation.x = float(orientation.get("x", 0.0))
            pose.pose.orientation.y = float(orientation.get("y", 0.0))
            pose.pose.orientation.z = float(orientation.get("z", 0.0))
            pose.pose.orientation.w = float(orientation.get("w", 1.0))

            for name in self._names_for_entry(waypoint_id, entry):
                waypoint_map[name] = pose

        self.get_logger().info(
            f"Loaded {len(waypoint_map)} named pose aliases from {path}: "
            f"{', '.join(sorted(waypoint_map.keys()))}"
        )
        return waypoint_map

    def _names_for_entry(self, waypoint_id: str, entry: dict) -> Iterable[str]:
        names = []
        explicit_name = str(entry.get("name", "")).strip()
        if explicit_name:
            names.append(explicit_name)

        if waypoint_id:
            names.append(waypoint_id)
            names.append(f"waypoint_{waypoint_id}")

        return dict.fromkeys(name for name in names if name)

    def goal_callback(self, goal_request: NavigateNamedPose.Goal) -> GoalResponse:
        target_name = goal_request.target_name.strip()
        if not target_name:
            self.get_logger().warning("Rejecting empty target_name for NavigateNamedPose.")
            return GoalResponse.REJECT
        if target_name not in self._waypoint_map:
            self.get_logger().warning(
                f"Rejecting unknown named pose '{target_name}'. Known names: "
                f"{', '.join(sorted(self._waypoint_map.keys()))}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        target_name = goal_handle.request.target_name.strip()
        nav_goal_handle = self._goal_handles.get(target_name)
        if nav_goal_handle is not None:
            nav_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle) -> NavigateNamedPose.Result:
        target_name = goal_handle.request.target_name.strip()
        result = NavigateNamedPose.Result()

        target_pose = self._waypoint_map.get(target_name)
        if target_pose is None:
            goal_handle.abort()
            result.success = False
            result.message = f"Unknown named pose '{target_name}'"
            return result

        if not self._nav_client.wait_for_server(timeout_sec=self._nav_server_timeout_sec):
            self.get_logger().error(
                f"{self._nav_action_name} action server unavailable after "
                f"{self._nav_server_timeout_sec:.1f}s while resolving '{target_name}'."
            )
            goal_handle.abort()
            result.success = False
            result.message = (
                f"NavigateToPose action server is unavailable after "
                f"{self._nav_server_timeout_sec:.1f}s"
            )
            return result

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = target_pose
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()

        self._publish_feedback(goal_handle, f"resolved {target_name}")
        self.get_logger().info(
            f"Named pose '{target_name}' -> frame={nav_goal.pose.header.frame_id} "
            f"pos=({nav_goal.pose.pose.position.x:.3f}, "
            f"{nav_goal.pose.pose.position.y:.3f}, "
            f"{nav_goal.pose.pose.position.z:.3f})"
        )

        send_goal_future = self._nav_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda msg: self._forward_feedback(goal_handle, target_name, msg),
        )
        nav_goal_handle = await send_goal_future

        if nav_goal_handle is None or not nav_goal_handle.accepted:
            goal_handle.abort()
            result.success = False
            result.message = f"/navigate_to_pose rejected named pose '{target_name}'"
            return result

        self._goal_handles[target_name] = nav_goal_handle
        self._publish_feedback(goal_handle, f"{target_name}: accepted by /navigate_to_pose")

        nav_result = await nav_goal_handle.get_result_async()
        self._goal_handles.pop(target_name, None)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = f"Named pose '{target_name}' canceled"
            return result

        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            result.success = True
            result.message = f"Reached named pose '{target_name}'"
            return result

        goal_handle.abort()
        result.success = False
        result.message = (
            f"Failed to reach named pose '{target_name}' "
            f"(NavigateToPose status={nav_result.status})"
        )
        return result

    def _forward_feedback(self, goal_handle, target_name: str, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance = getattr(feedback, "distance_remaining", None)
        eta = getattr(feedback, "estimated_time_remaining", None)
        recoveries = getattr(feedback, "number_of_recoveries", None)

        parts = [f"{target_name}: navigating"]
        if distance is not None:
            parts.append(f"distance={distance:.2f}m")
        if eta is not None:
            eta_sec = float(getattr(eta, "sec", 0)) + float(getattr(eta, "nanosec", 0)) / 1e9
            parts.append(f"eta={eta_sec:.1f}s")
        if recoveries is not None:
            parts.append(f"recoveries={recoveries}")

        self._publish_feedback(goal_handle, ", ".join(parts))

    def _publish_feedback(self, goal_handle, stage: str) -> None:
        feedback = NavigateNamedPose.Feedback()
        feedback.stage = stage
        goal_handle.publish_feedback(feedback)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigateNamedPoseServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
