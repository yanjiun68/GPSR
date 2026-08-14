#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


GOAL_STATUS_TEXT = {
    0: 'unknown',
    1: 'accepted',
    2: 'executing',
    3: 'canceling',
    4: 'succeeded',
    5: 'canceled',
    6: 'aborted',
}


@dataclass(frozen=True)
class SemanticLocation:
    name: str
    x: float
    y: float
    yaw: float
    frame_id: str
    description: str


def yaw_from_quaternion(msg) -> float:
    siny_cosp = 2.0 * (msg.w * msg.z + msg.x * msg.y)
    cosy_cosp = 1.0 - 2.0 * (msg.y * msg.y + msg.z * msg.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SemanticNavigator(Node):
    def __init__(self):
        super().__init__('semantic_navigator')

        self.declare_parameter('locations_file', '')
        self.declare_parameter('goal_name_topic', '/semantic_nav/goal_name')
        self.declare_parameter('status_topic', '/semantic_nav/status')
        self.declare_parameter('current_location_topic', '/semantic_nav/current_location')
        self.declare_parameter('localization_topic', '/amcl_pose')
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        self.declare_parameter('location_match_radius', 0.75)

        locations_file = str(self.get_parameter('locations_file').value).strip()
        self.goal_name_topic = self.get_parameter('goal_name_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.current_location_topic = self.get_parameter('current_location_topic').value
        localization_topic = self.get_parameter('localization_topic').value
        action_name = self.get_parameter('navigate_to_pose_action').value
        self.location_match_radius = float(
            self.get_parameter('location_match_radius').value
        )

        self.locations = self.load_locations(locations_file)
        self.goal_client = ActionClient(self, NavigateToPose, action_name)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.current_location_pub = self.create_publisher(
            String,
            self.current_location_topic,
            10,
        )
        self.create_subscription(String, self.goal_name_topic, self.goal_name_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            localization_topic,
            self.localization_callback,
            10,
        )

        self.active_goal_handle = None
        self.active_goal_name: Optional[str] = None
        self.pending_goal_name: Optional[str] = None
        self.last_current_location: Optional[str] = None
        self.last_feedback_log_ns = 0

        loaded_names = ', '.join(sorted(self.locations.keys())) or '(none)'
        self.publish_status(
            f'Semantic navigator ready. Goal topic: {self.goal_name_topic}. '
            f'Loaded locations: {loaded_names}'
        )

    def load_locations(self, locations_file: str) -> Dict[str, SemanticLocation]:
        if not locations_file:
            self.get_logger().warn('No semantic locations file configured.')
            return {}

        try:
            with open(locations_file, 'r', encoding='utf-8') as stream:
                raw_data = yaml.safe_load(stream) or {}
        except FileNotFoundError:
            self.get_logger().error(
                f'Semantic locations file not found: {locations_file}'
            )
            return {}
        except yaml.YAMLError as exc:
            self.get_logger().error(
                f'Failed to parse semantic locations file {locations_file}: {exc}'
            )
            return {}

        raw_locations = raw_data.get('locations', raw_data)
        if not isinstance(raw_locations, dict):
            self.get_logger().error(
                'Semantic locations YAML must be a dictionary or contain a '
                '"locations" dictionary.'
            )
            return {}

        locations: Dict[str, SemanticLocation] = {}
        for name, value in raw_locations.items():
            if not isinstance(value, dict):
                self.get_logger().warn(
                    f'Skipping semantic location "{name}" because it is not a mapping.'
                )
                continue

            try:
                x = float(value['x'])
                y = float(value['y'])
                if 'yaw' in value:
                    yaw = float(value['yaw'])
                else:
                    yaw = math.radians(float(value.get('yaw_deg', 0.0)))
                frame_id = str(value.get('frame_id', 'map'))
                description = str(value.get('description', ''))
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warn(
                    f'Skipping semantic location "{name}" due to invalid data: {exc}'
                )
                continue

            locations[name] = SemanticLocation(
                name=str(name),
                x=x,
                y=y,
                yaw=yaw,
                frame_id=frame_id,
                description=description,
            )

        return locations

    def goal_name_callback(self, msg: String):
        requested_name = msg.data.strip()
        if not requested_name:
            self.publish_status('Ignored empty semantic goal name.')
            return

        if requested_name not in self.locations:
            available = ', '.join(sorted(self.locations.keys())) or '(none)'
            self.publish_status(
                f'Unknown semantic goal "{requested_name}". '
                f'Available locations: {available}'
            )
            return

        if self.active_goal_name is not None:
            self.pending_goal_name = requested_name
            if self.active_goal_handle is None:
                self.publish_status(
                    f'Queued semantic goal "{requested_name}" while '
                    f'"{self.active_goal_name}" is still being accepted.'
                )
                return
            self.publish_status(
                f'Canceling active goal "{self.active_goal_name}" before '
                f'switching to "{requested_name}".'
            )
            cancel_future = self.active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            return

        self.send_named_goal(requested_name)

    def send_named_goal(self, goal_name: str):
        if goal_name not in self.locations:
            return

        if not self.goal_client.wait_for_server(timeout_sec=3.0):
            self.publish_status(
                'navigate_to_pose action server is not available yet.'
            )
            return

        location = self.locations[goal_name]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.build_goal_pose(location)

        self.active_goal_name = goal_name
        send_future = self.goal_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback: self.feedback_callback(goal_name, feedback),
        )
        send_future.add_done_callback(
            lambda future: self.goal_response_callback(goal_name, future)
        )

        heading_deg = math.degrees(location.yaw)
        self.publish_status(
            f'Sending semantic goal "{goal_name}" to '
            f'x={location.x:.2f}, y={location.y:.2f}, yaw={heading_deg:.1f} deg.'
        )

    def build_goal_pose(self, location: SemanticLocation) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = location.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = location.x
        pose.pose.position.y = location.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(location.yaw / 2.0)
        pose.pose.orientation.w = math.cos(location.yaw / 2.0)
        return pose

    def goal_response_callback(self, goal_name: str, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.active_goal_name = None
            self.active_goal_handle = None
            self.publish_status(
                f'Failed to send semantic goal "{goal_name}": {exc}'
            )
            return

        if not goal_handle.accepted:
            self.active_goal_name = None
            self.active_goal_handle = None
            self.publish_status(f'Semantic goal "{goal_name}" was rejected.')
            return

        self.active_goal_handle = goal_handle
        self.publish_status(f'Semantic goal "{goal_name}" accepted by Nav2.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.goal_result_callback(goal_name, result)
        )

        if self.pending_goal_name is not None and self.pending_goal_name != goal_name:
            self.publish_status(
                f'Preempting "{goal_name}" because a newer semantic goal '
                f'"{self.pending_goal_name}" is waiting.'
            )
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def goal_result_callback(self, goal_name: str, future):
        try:
            result = future.result()
            status = int(result.status)
        except Exception as exc:
            self.publish_status(
                f'Semantic goal "{goal_name}" finished with an error: {exc}'
            )
            status = 0

        self.active_goal_handle = None
        self.active_goal_name = None
        self.publish_status(
            f'Semantic goal "{goal_name}" finished with status '
            f'{GOAL_STATUS_TEXT.get(status, str(status))}.'
        )

        if self.pending_goal_name is not None:
            next_goal = self.pending_goal_name
            self.pending_goal_name = None
            self.send_named_goal(next_goal)

    def cancel_done_callback(self, future):
        try:
            cancel_response = future.result()
        except Exception as exc:
            self.publish_status(f'Failed to cancel active semantic goal: {exc}')
            return

        if cancel_response.goals_canceling:
            self.publish_status('Active semantic goal is canceling.')
        else:
            self.publish_status('Nav2 reported no active semantic goal to cancel.')

    def feedback_callback(self, goal_name: str, feedback_msg):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_feedback_log_ns < 1000000000:
            return

        self.last_feedback_log_ns = now_ns
        distance_remaining = float(feedback_msg.feedback.distance_remaining)
        self.publish_status(
            f'Navigating to "{goal_name}", remaining distance: '
            f'{distance_remaining:.2f} m.'
        )

    def localization_callback(self, msg: PoseWithCovarianceStamped):
        if not self.locations:
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        nearest_name, nearest_distance = self.find_nearest_location(x, y)
        if nearest_name is None or nearest_distance > self.location_match_radius:
            current_location = 'unknown'
        else:
            current_location = nearest_name

        if current_location == self.last_current_location:
            return

        self.last_current_location = current_location
        current_msg = String()
        current_msg.data = current_location
        self.current_location_pub.publish(current_msg)

        if current_location == 'unknown':
            self.get_logger().info(
                f'Localized in map frame at x={x:.2f}, y={y:.2f}, '
                f'yaw={math.degrees(yaw):.1f} deg, not near a semantic point.'
            )
            return

        self.get_logger().info(
            f'Current semantic location: "{current_location}" '
            f'(x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg).'
        )

    def find_nearest_location(self, x: float, y: float) -> Tuple[Optional[str], float]:
        nearest_name: Optional[str] = None
        nearest_distance = float('inf')

        for name, location in self.locations.items():
            distance = math.hypot(location.x - x, location.y - y)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_name = name

        return nearest_name, nearest_distance

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
