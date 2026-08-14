#!/usr/bin/env python3

import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


def resolve_default_locations_file() -> str:
    pkg_name = 'my_robot_nav'

    try:
        pkg_share = get_package_share_directory(pkg_name)
    except PackageNotFoundError:
        return ''

    install_file = os.path.join(pkg_share, 'config', 'semantic_locations.yaml')
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(pkg_share)))
    )
    source_file = os.path.join(
        workspace_root,
        'src',
        pkg_name,
        'config',
        'semantic_locations.yaml',
    )

    if os.path.isfile(source_file):
        return source_file

    return install_file


def yaw_from_quaternion(msg) -> float:
    siny_cosp = 2.0 * (msg.w * msg.z + msg.x * msg.y)
    cosy_cosp = 1.0 - 2.0 * (msg.y * msg.y + msg.z * msg.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RvizSemanticLocationCapture(Node):
    def __init__(self):
        super().__init__('rviz_semantic_location_capture')

        self.declare_parameter('name', '')
        self.declare_parameter('description', '')
        self.declare_parameter('locations_file', resolve_default_locations_file())
        self.declare_parameter('input_mode', 'clicked_point')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('pose_topic', '/initialpose_raw')
        self.declare_parameter('fallback_pose_topic', '/initialpose')
        self.declare_parameter('timeout_sec', 60.0)
        self.declare_parameter('overwrite', False)
        self.declare_parameter('frame_id', '')
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter(
            'yaw_deg',
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )

        self.name = str(self.get_parameter('name').value).strip()
        self.description = str(self.get_parameter('description').value).strip()
        self.locations_file = str(self.get_parameter('locations_file').value).strip()
        self.input_mode = str(self.get_parameter('input_mode').value).strip().lower()
        self.clicked_point_topic = str(
            self.get_parameter('clicked_point_topic').value
        ).strip()
        self.pose_topic = str(self.get_parameter('pose_topic').value).strip()
        self.fallback_pose_topic = str(
            self.get_parameter('fallback_pose_topic').value
        ).strip()
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.overwrite = bool(self.get_parameter('overwrite').value)
        self.frame_id_override = str(self.get_parameter('frame_id').value).strip()

        yaw_param = self.get_parameter('yaw').value
        yaw_deg_param = self.get_parameter('yaw_deg').value
        self.default_yaw = float(yaw_param)
        if yaw_deg_param is not None:
            self.default_yaw = math.radians(float(yaw_deg_param))

        self.done = False
        self.success = False
        self.deadline_ns = (
            self.get_clock().now().nanoseconds + int(self.timeout_sec * 1e9)
        )

        if not self.name:
            self.get_logger().error(
                'Parameter "name" is required, for example: -p name:=kitchen'
            )
            self.done = True
            return

        if not self.locations_file:
            self.get_logger().error('Could not resolve a semantic locations file path.')
            self.done = True
            return

        if self.input_mode == 'clicked_point':
            self.subscription = self.create_subscription(
                PointStamped,
                self.clicked_point_topic,
                self.clicked_point_callback,
                10,
            )
            self.get_logger().info(
                f'Waiting for RViz Publish Point on {self.clicked_point_topic} to '
                f'save semantic location "{self.name}". '
                f'Yaw will use {self.default_yaw:.4f} rad '
                f'({math.degrees(self.default_yaw):.1f} deg).'
            )
        elif self.input_mode == 'initial_pose':
            self.pose_subscriptions = []
            pose_topics = []
            if self.pose_topic:
                pose_topics.append(self.pose_topic)
            if (
                self.fallback_pose_topic
                and self.fallback_pose_topic not in pose_topics
            ):
                pose_topics.append(self.fallback_pose_topic)

            if not pose_topics:
                self.get_logger().error(
                    'At least one initial pose topic must be configured.'
                )
                self.done = True
                return

            for topic in pose_topics:
                self.pose_subscriptions.append(
                    self.create_subscription(
                        PoseWithCovarianceStamped,
                        topic,
                        lambda msg, source_topic=topic: self.pose_callback(
                            msg, source_topic
                        ),
                        10,
                    )
                )

            self.get_logger().info(
                'Waiting for RViz Set Initial Pose on '
                f'{", ".join(pose_topics)} to save semantic location '
                f'"{self.name}".'
            )
        else:
            self.get_logger().error(
                'input_mode must be "clicked_point" or "initial_pose".'
            )
            self.done = True
            return

        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        if self.done:
            return

        if self.get_clock().now().nanoseconds < self.deadline_ns:
            return

        self.get_logger().error(
            f'Timed out after {self.timeout_sec:.1f} s waiting for RViz input.'
        )
        self.done = True

    def clicked_point_callback(self, msg: PointStamped):
        if self.done:
            return

        try:
            self.save_location(
                x=float(msg.point.x),
                y=float(msg.point.y),
                yaw=self.default_yaw,
                frame_id=self.frame_id_override or msg.header.frame_id or 'map',
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to save semantic location: {exc}')
            self.done = True
            return

        self.success = True
        self.done = True

    def pose_callback(self, msg: PoseWithCovarianceStamped, source_topic: str):
        if self.done:
            return

        pose = msg.pose.pose
        try:
            self.save_location(
                x=float(pose.position.x),
                y=float(pose.position.y),
                yaw=yaw_from_quaternion(pose.orientation),
                frame_id=self.frame_id_override or msg.header.frame_id or 'map',
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to save semantic location: {exc}')
            self.done = True
            return

        self.success = True
        self.done = True
        self.get_logger().info(
            f'Recorded semantic location "{self.name}" from {source_topic}.'
        )

    def save_location(self, x: float, y: float, yaw: float, frame_id: str):
        locations_data = {}
        if os.path.exists(self.locations_file):
            with open(self.locations_file, 'r', encoding='utf-8') as stream:
                loaded = yaml.safe_load(stream) or {}
            if not isinstance(loaded, dict):
                raise ValueError('Semantic locations YAML root must be a mapping.')
            locations_data = loaded

        if 'locations' not in locations_data:
            locations_data['locations'] = {}

        if not isinstance(locations_data['locations'], dict):
            raise ValueError('"locations" must be a mapping in the YAML file.')

        if self.name in locations_data['locations'] and not self.overwrite:
            raise ValueError(
                f'Location "{self.name}" already exists. '
                f'Use -p overwrite:=true to replace it.'
            )

        location_entry = {
            'x': round(x, 4),
            'y': round(y, 4),
            'yaw': round(yaw, 4),
            'yaw_deg': round(math.degrees(yaw), 1),
            'frame_id': frame_id,
        }

        if self.description:
            location_entry['description'] = self.description

        locations_data['locations'][self.name] = location_entry

        parent_dir = os.path.dirname(self.locations_file)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

        with open(self.locations_file, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(
                locations_data,
                stream,
                sort_keys=False,
                allow_unicode=False,
            )

        self.get_logger().info(
            f'Saved semantic location "{self.name}" to {self.locations_file}: '
            f'x={location_entry["x"]:.4f}, y={location_entry["y"]:.4f}, '
            f'yaw={location_entry["yaw"]:.4f} rad '
            f'({location_entry["yaw_deg"]:.1f} deg)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RvizSemanticLocationCapture()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted before saving semantic location.')
    finally:
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
