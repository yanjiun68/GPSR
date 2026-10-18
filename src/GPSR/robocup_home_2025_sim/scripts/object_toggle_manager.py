#!/usr/bin/env python3
"""Temporarily remove and respawn registered GPSR objects in Gazebo Fortress."""

from __future__ import annotations

import csv
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Sequence

try:  # Keep the registry/command builder unit-testable without a ROS overlay.
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.node import Node
    from robot_ai_interfaces.srv import ToggleObject
except ImportError:  # pragma: no cover - exercised only by source-level tests.
    rclpy = None
    Node = object
    ToggleObject = None


@dataclass(frozen=True)
class ObjectSpec:
    friendly_name: str
    entity_name: str
    sdf_filename: Path
    pose: tuple[float, float, float, float]


def load_registry(csv_filename: Path, models_directory: Path) -> Dict[str, ObjectSpec]:
    """Load the existing world placement CSV and expose both supported names."""
    registry: Dict[str, ObjectSpec] = {}
    with csv_filename.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            friendly = row['name']
            spec = ObjectSpec(
                friendly_name=friendly,
                entity_name=f'object_2025_{friendly}',
                sdf_filename=models_directory / row['mesh_model'] / 'model.sdf',
                pose=(float(row['x']), float(row['y']), float(row['z_bottom']),
                      float(row['yaw'])),
            )
            registry[friendly] = spec
            registry[spec.entity_name] = spec
    return registry


class GazeboObjectToggle:
    """Registry-backed Gazebo command builder and executor, independent of ROS."""

    def __init__(self, registry: Dict[str, ObjectSpec], world_name: str,
                 gazebo_command: str = 'ign',
                 runner: Callable[..., subprocess.CompletedProcess] = subprocess.run):
        self.registry = registry
        self.world_name = world_name
        self.gazebo_command = gazebo_command
        self.runner = runner
        self.active = {spec.entity_name for spec in registry.values()}

    @property
    def message_namespace(self) -> str:
        return 'ignition.msgs' if self.gazebo_command == 'ign' else 'gz.msgs'

    def _run(self, service: str, request_type: str, request: str) -> tuple[bool, str]:
        command: Sequence[str] = [
            self.gazebo_command, 'service', '-s', service,
            '--reqtype', request_type,
            '--reptype', f'{self.message_namespace}.Boolean',
            '--timeout', '3000', '--req', request,
        ]
        try:
            completed = self.runner(command, capture_output=True, text=True, check=False)
        except OSError as error:
            return False, f'Unable to execute {self.gazebo_command}: {error}'
        output = ((completed.stdout or '') + (completed.stderr or '')).strip()
        success = completed.returncode == 0 and 'data: true' in output.lower()
        return success, output or ('Gazebo accepted request' if success else 'Gazebo request failed')

    @staticmethod
    def _create_request(spec: ObjectSpec) -> str:
        x, y, z, yaw = spec.pose
        return (
            f'name: "{spec.entity_name}" sdf_filename: "{spec.sdf_filename}" '
            f'pose {{ position {{ x: {x} y: {y} z: {z} }} orientation {{ '
            f'z: {math.sin(yaw / 2.0)} w: {math.cos(yaw / 2.0)} }} }}'
        )

    def toggle(self, name: str, enabled: bool) -> tuple[bool, bool, str]:
        spec = self.registry.get(name)
        if spec is None:
            return False, False, f'Unknown object {name!r}'
        is_active = spec.entity_name in self.active
        if enabled == is_active:
            return True, is_active, f'{spec.friendly_name} is already ' + ('active' if is_active else 'inactive')

        base = f'/world/{self.world_name}'
        if enabled:
            if not spec.sdf_filename.is_file():
                return False, False, f'Model file does not exist: {spec.sdf_filename}'
            success, detail = self._run(f'{base}/create', f'{self.message_namespace}.EntityFactory',
                                        self._create_request(spec))
        else:
            success, detail = self._run(f'{base}/remove', f'{self.message_namespace}.Entity',
                                        f'name: "{spec.entity_name}" type: MODEL')
        if success:
            if enabled:
                self.active.add(spec.entity_name)
            else:
                self.active.discard(spec.entity_name)
            return True, enabled, f'{spec.friendly_name} is now ' + ('active' if enabled else 'inactive')
        return False, is_active, detail


class ObjectToggleManager(Node):
    def __init__(self) -> None:
        super().__init__('object_toggle_manager')
        share = Path(get_package_share_directory('robocup_home_2025_sim'))
        self.declare_parameter('world_name', 'robocup_home_2025_vision')
        self.declare_parameter('gazebo_command', 'ign')
        self.declare_parameter('registry_csv', str(share / 'config' / '2025_item_poses.csv'))
        self.declare_parameter('models_directory', str(share / 'worlds' / 'models'))
        registry = load_registry(Path(self.get_parameter('registry_csv').value),
                                 Path(self.get_parameter('models_directory').value))
        self.toggle = GazeboObjectToggle(registry, self.get_parameter('world_name').value,
                                         self.get_parameter('gazebo_command').value)
        self.service = self.create_service(ToggleObject, '/toggle_object', self._handle_toggle)
        self.get_logger().info(f'Registered {len(self.toggle.active)} toggleable GPSR objects on /toggle_object')

    def _handle_toggle(self, request: ToggleObject.Request,
                       response: ToggleObject.Response) -> ToggleObject.Response:
        success, active, message = self.toggle.toggle(request.object_name, request.enabled)
        response.success, response.active, response.message = success, active, message
        (self.get_logger().info if success else self.get_logger().error)(message)
        return response


def main(args=None) -> None:
    if rclpy is None or ToggleObject is None:
        raise RuntimeError('object_toggle_manager requires a sourced ROS 2 workspace')
    rclpy.init(args=args)
    node = ObjectToggleManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
