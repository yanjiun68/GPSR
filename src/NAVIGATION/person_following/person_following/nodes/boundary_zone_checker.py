#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

from person_following.utils.boundary_events import (
    EVENT_ZONE_TRIGGERED,
    REGION_INSIDE,
    REGION_OUTSIDE,
    ROLE_ENTER,
    ROLE_EXIT,
    is_valid_region,
    is_valid_role,
    normalize_region,
)


@dataclass
class CircleZone:
    name: str
    role: str
    x: float
    y: float
    radius_m: float
    armed: bool = True
    inside_since: Optional[float] = None

    def contains(self, px: float, py: float) -> bool:
        return math.hypot(px - self.x, py - self.y) <= self.radius_m


class BoundaryZoneChecker(Node):
    def __init__(self):
        super().__init__('boundary_zone_checker')

        self.declare_parameter('boundary_config_file', '')
        self.declare_parameter('boundary_enable_rviz_override', False)
        self.declare_parameter('boundary_clicked_point_topic', '/clicked_point')
        self.declare_parameter('boundary_rviz_target_zone', '')

        self.config_file = str(self.get_parameter('boundary_config_file').value).strip()
        self.enable_rviz_override = bool(self.get_parameter('boundary_enable_rviz_override').value)
        self.clicked_point_topic = str(self.get_parameter('boundary_clicked_point_topic').value)
        self.rviz_target_zone = str(self.get_parameter('boundary_rviz_target_zone').value).strip()

        self.global_frame = 'map'
        self.base_frame = 'base_footprint'
        self.boundary_event_topic = '/person_follow/boundary_event'
        self.boundary_status_topic = '/person_follow/boundary_status'
        self.hold_time_sec = 0.5
        self.status_hz = 5.0
        self.initial_region = REGION_INSIDE
        self.current_region = REGION_INSIDE
        self.zones: list[CircleZone] = []
        self.zone_by_name: dict[str, CircleZone] = {}

        self._load_config()
        self._validate_loaded_config()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_event = self.create_publisher(String, self.boundary_event_topic, 10)
        self.pub_status = self.create_publisher(String, self.boundary_status_topic, 10)
        self.create_subscription(PointStamped, self.clicked_point_topic, self._clicked_point_cb, 10)

        self.last_pose_x: Optional[float] = None
        self.last_pose_y: Optional[float] = None
        self.last_tf_ok = False
        self._last_tf_warn_sec = 0.0
        self._last_rviz_warn_sec = 0.0
        self._last_triggered_zone = ''

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.timer = self.create_timer(1.0 / self.status_hz, self._timer_cb)

        self.get_logger().info(
            f'[BoundaryZoneChecker] Started region={self.current_region} '
            f'config="{self.config_file}" zones={len(self.zones)} '
            f'rviz_override={self.enable_rviz_override} rviz_target={self.rviz_target_zone or "<unset>"}'
        )

    def _load_config(self):
        if not self.config_file:
            raise ValueError('boundary_config_file is empty')

        path = Path(self.config_file)
        if not path.is_file():
            raise FileNotFoundError(f'boundary_config_file not found: {self.config_file}')

        with path.open('r', encoding='utf-8') as fh:
            raw = yaml.safe_load(fh) or {}

        params = raw
        if 'boundary_zone_checker' in raw:
            params = raw.get('boundary_zone_checker', {}).get('ros__parameters', {})

        self.global_frame = str(params.get('global_frame', self.global_frame))
        self.base_frame = str(params.get('base_frame', self.base_frame))
        self.boundary_event_topic = str(params.get('boundary_event_topic', self.boundary_event_topic))
        self.boundary_status_topic = str(params.get('boundary_status_topic', self.boundary_status_topic))
        self.hold_time_sec = float(params.get('hold_time_sec', self.hold_time_sec))
        self.status_hz = max(1.0, float(params.get('status_hz', self.status_hz)))
        self.initial_region = str(params.get('initial_region', self.initial_region))
        self.current_region = normalize_region(self.initial_region)

        zone_defs = params.get('zones')
        if not isinstance(zone_defs, list):
            raise ValueError('zones must be a list in boundary config')

        zones: list[CircleZone] = []
        seen_names: set[str] = set()
        for index, item in enumerate(zone_defs):
            if not isinstance(item, dict):
                raise ValueError(f'zones[{index}] must be a mapping')
            name = str(item.get('name', '')).strip()
            role = str(item.get('role', '')).strip().lower()
            if not name:
                raise ValueError(f'zones[{index}].name is required')
            if name in seen_names:
                raise ValueError(f'duplicate zone name: {name}')
            if not is_valid_role(role):
                raise ValueError(f'zones[{index}].role must be exit or enter, got {role!r}')
            try:
                x = float(item['x'])
                y = float(item['y'])
                radius_m = float(item['radius_m'])
            except KeyError as exc:
                raise ValueError(f'zones[{index}] missing key: {exc.args[0]}') from exc
            except (TypeError, ValueError) as exc:
                raise ValueError(f'zones[{index}] has invalid numeric value: {exc}') from exc
            zones.append(CircleZone(name=name, role=role, x=x, y=y, radius_m=radius_m))
            seen_names.add(name)

        self.zones = zones
        self.zone_by_name = {zone.name: zone for zone in self.zones}

    def _validate_loaded_config(self):
        if self.hold_time_sec <= 0.0:
            raise ValueError('hold_time_sec must be > 0')
        if not is_valid_region(self.initial_region):
            raise ValueError(f'invalid initial_region: {self.initial_region!r}')
        if not self.zones:
            raise ValueError('boundary config must define at least one zone')
        for zone in self.zones:
            if zone.radius_m <= 0.0:
                raise ValueError(f'zone {zone.name!r} radius_m must be > 0')

        roles = {zone.role for zone in self.zones}
        if ROLE_EXIT not in roles:
            self.get_logger().warn('[BoundaryZoneChecker] No exit zones configured')
        if ROLE_ENTER not in roles:
            self.get_logger().warn('[BoundaryZoneChecker] No enter zones configured')

        self._warn_if_zones_overlap()
        self._validate_rviz_target_zone(initial=True)

    def _warn_if_zones_overlap(self):
        for idx, zone_a in enumerate(self.zones):
            for zone_b in self.zones[idx + 1:]:
                center_distance = math.hypot(zone_a.x - zone_b.x, zone_a.y - zone_b.y)
                radius_sum = zone_a.radius_m + zone_b.radius_m
                if center_distance < radius_sum:
                    if zone_a.role == zone_b.role:
                        self.get_logger().warn(
                            f'[BoundaryZoneChecker] Overlapping same-role zones: {zone_a.name} and {zone_b.name}'
                        )
                    else:
                        self.get_logger().warn(
                            f'[BoundaryZoneChecker] Overlapping zones with different roles: {zone_a.name} and {zone_b.name}'
                        )

    def _validate_rviz_target_zone(self, initial: bool = False):
        if not self.rviz_target_zone:
            return
        if self.rviz_target_zone not in self.zone_by_name:
            if initial:
                self.get_logger().warn(
                    f'[BoundaryZoneChecker] boundary_rviz_target_zone not found in config: {self.rviz_target_zone!r}'
                )
            else:
                raise ValueError(f'unknown boundary_rviz_target_zone: {self.rviz_target_zone}')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _throttled_warn(self, message: str, attribute: str, interval_sec: float = 2.0):
        now = self._now_sec()
        last = getattr(self, attribute)
        if (now - last) >= interval_sec:
            self.get_logger().warn(message)
            setattr(self, attribute, now)

    def _lookup_pose(self) -> Optional[tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except Exception as exc:
            self.last_tf_ok = False
            self._throttled_warn(
                f'[BoundaryZoneChecker] TF lookup failed {self.global_frame} <- {self.base_frame}: {exc}',
                '_last_tf_warn_sec',
            )
            return None
        self.last_tf_ok = True
        self.last_pose_x = float(tf.transform.translation.x)
        self.last_pose_y = float(tf.transform.translation.y)
        return self.last_pose_x, self.last_pose_y

    def _active_role(self) -> str:
        return ROLE_EXIT if self.current_region == REGION_INSIDE else ROLE_ENTER

    def _switch_region(self, next_region: str):
        previous = self.current_region
        self.current_region = next_region
        for zone in self.zones:
            zone.inside_since = None
        if previous != next_region:
            self.get_logger().info(f'[BoundaryZoneChecker] region: {previous} -> {next_region}')

    def _emit_event(self, zone: CircleZone):
        msg = String()
        msg.data = json.dumps(
            {'event': EVENT_ZONE_TRIGGERED, 'name': zone.name, 'role': zone.role},
            separators=(',', ':'),
        )
        self.pub_event.publish(msg)
        self._last_triggered_zone = zone.name
        self.get_logger().info(f'[BoundaryZoneChecker] event emitted: zone={zone.name} role={zone.role}')

    def _status_payload(self, matched_zone: str, inside_zone_names: list[str]) -> str:
        payload = {
            'region': self.current_region,
            'active_role': self._active_role(),
            'matched_zone': matched_zone,
            'inside_zone_names': inside_zone_names,
            'tf_healthy': bool(self.last_tf_ok),
            'robot_x': self.last_pose_x,
            'robot_y': self.last_pose_y,
            'rviz_override_enabled': bool(self.enable_rviz_override),
            'rviz_target_zone': self.rviz_target_zone,
            'last_triggered_zone': self._last_triggered_zone,
        }
        return json.dumps(payload, separators=(',', ':'))

    def _publish_status(self, matched_zone: str, inside_zone_names: list[str]):
        msg = String()
        msg.data = self._status_payload(matched_zone, inside_zone_names)
        self.pub_status.publish(msg)

    def _timer_cb(self):
        pose = self._lookup_pose()
        if pose is None:
            self._publish_status('', [])
            return

        now = self._now_sec()
        px, py = pose
        matched_zone = ''
        inside_active_names: list[str] = []
        active_role = self._active_role()

        for zone in self.zones:
            inside = zone.contains(px, py)
            if not inside and not zone.armed:
                zone.armed = True
                self.get_logger().info(f'[BoundaryZoneChecker] zone rearmed: {zone.name}')

            if zone.role != active_role:
                zone.inside_since = None
                continue

            if inside:
                inside_active_names.append(zone.name)
                if not matched_zone:
                    matched_zone = zone.name
                if zone.inside_since is None:
                    zone.inside_since = now
                elif zone.armed and (now - zone.inside_since) >= self.hold_time_sec:
                    self._emit_event(zone)
                    zone.armed = False
                    self._switch_region(REGION_OUTSIDE if zone.role == ROLE_EXIT else REGION_INSIDE)
                    break
            else:
                zone.inside_since = None

        self._publish_status(matched_zone, inside_active_names)

    def _clicked_point_cb(self, msg: PointStamped):
        if not self.enable_rviz_override:
            return
        if not self.rviz_target_zone:
            self._throttled_warn(
                '[BoundaryZoneChecker] RViz override enabled but boundary_rviz_target_zone is empty',
                '_last_rviz_warn_sec',
            )
            return
        zone = self.zone_by_name.get(self.rviz_target_zone)
        if zone is None:
            self._throttled_warn(
                f'[BoundaryZoneChecker] Ignoring click, unknown boundary_rviz_target_zone: {self.rviz_target_zone!r}',
                '_last_rviz_warn_sec',
            )
            return

        point_frame = str(msg.header.frame_id or self.global_frame)
        if point_frame != self.global_frame:
            self._throttled_warn(
                f'[BoundaryZoneChecker] Ignoring clicked point in frame {point_frame!r}; expected {self.global_frame!r}',
                '_last_rviz_warn_sec',
            )
            return

        zone.x = float(msg.point.x)
        zone.y = float(msg.point.y)
        zone.inside_since = None
        self.get_logger().info(
            f'[BoundaryZoneChecker] RViz override updated zone {zone.name} to ({zone.x:.3f}, {zone.y:.3f})'
        )
        self._warn_if_zones_overlap()

    def _on_set_parameters(self, params) -> SetParametersResult:
        next_enable = self.enable_rviz_override
        next_target_zone = self.rviz_target_zone

        for param in params:
            if param.name == 'boundary_enable_rviz_override':
                next_enable = bool(param.value)
            elif param.name == 'boundary_rviz_target_zone':
                next_target_zone = str(param.value).strip()
                if next_target_zone and next_target_zone not in self.zone_by_name:
                    return SetParametersResult(
                        successful=False,
                        reason=f'unknown boundary_rviz_target_zone: {next_target_zone}',
                    )
            elif param.name in ('boundary_config_file', 'boundary_clicked_point_topic'):
                return SetParametersResult(
                    successful=False,
                    reason=f'{param.name} is startup-only',
                )

        self.enable_rviz_override = next_enable
        self.rviz_target_zone = next_target_zone
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryZoneChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
