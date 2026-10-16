#!/usr/bin/env python3
"""Static and live checks for the robot-description publication pipeline."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


MOVABLE_TYPES = {'continuous', 'prismatic', 'revolute'}
REQUIRED_FRAMES = {
    'base_footprint', 'base_link', 'baseup_link', 'lidar_link',
    'camera_link',
}


def result(status, detail=''):
    return {'status': status, 'detail': detail}


def expand_xacro(model):
    completed = subprocess.run(
        ['xacro', str(model)], capture_output=True, text=True, timeout=20,
        check=False)
    if completed.returncode:
        raise RuntimeError(completed.stderr.strip() or 'xacro failed')
    return completed.stdout


def static_checks(model):
    report = {}
    try:
        xml = expand_xacro(model)
        root = ET.fromstring(xml)
        report['xacro_expansion'] = result('PASS', f'{len(xml)} bytes')
    except Exception as exc:
        report['xacro_expansion'] = result('FAIL', str(exc))
        return report, '', set(), set()

    links = root.findall('link')
    joints = root.findall('joint')
    link_names = [link.get('name') for link in links]
    joint_names = [joint.get('name') for joint in joints]
    duplicate_links = sorted({name for name in link_names if link_names.count(name) > 1})
    duplicate_joints = sorted({name for name in joint_names if joint_names.count(name) > 1})
    children = {
        joint.find('child').get('link') for joint in joints
        if joint.find('child') is not None
    }
    roots = set(link_names) - children
    missing_refs = []
    edges = {}
    for joint in joints:
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is None or child is None:
            missing_refs.append(joint.get('name'))
            continue
        parent_name, child_name = parent.get('link'), child.get('link')
        if parent_name not in link_names or child_name not in link_names:
            missing_refs.append(joint.get('name'))
        edges.setdefault(parent_name, set()).add(child_name)

    valid_tree = not duplicate_links and not duplicate_joints and not missing_refs and len(roots) == 1
    report['urdf_tree'] = result(
        'PASS' if valid_tree else 'FAIL',
        f'{len(links)} links, {len(joints)} joints, roots={sorted(roots)}; '
        f'duplicate_links={duplicate_links}, duplicate_joints={duplicate_joints}, '
        f'invalid_joints={missing_refs}')

    root_name = next(iter(roots), None)
    root_link = next((link for link in links if link.get('name') == root_name), None)
    has_root_inertia = root_link is not None and root_link.find('inertial') is not None
    report['root_inertia'] = result(
        'FAIL' if has_root_inertia else 'PASS',
        f'root={root_name}, inertial={has_root_inertia}')

    visited = set()
    pending = [root_name] if root_name else []
    while pending:
        current = pending.pop()
        if current in visited:
            continue
        visited.add(current)
        pending.extend(edges.get(current, ()))
    disconnected = sorted(set(link_names) - visited)
    report['connectivity'] = result(
        'PASS' if not disconnected else 'FAIL',
        'all links connected' if not disconnected else f'disconnected={disconnected}')

    movable = {
        joint.get('name') for joint in joints
        if joint.get('type') in MOVABLE_TYPES
    }
    report['movable_joints'] = result('PASS' if movable else 'FAIL', ', '.join(sorted(movable)))
    arm_frames = {'arm_base_link', 'rx200/base_link'} & set(link_names)
    missing_frames = sorted(REQUIRED_FRAMES - set(link_names))
    if not arm_frames:
        missing_frames.append('arm base link')
    report['required_frames'] = result(
        'PASS' if not missing_frames else 'WARNING',
        'all intended frames present' if not missing_frames else f'missing={missing_frames}')

    missing_meshes = []
    checked_meshes = 0
    for mesh in root.findall('.//mesh'):
        filename = mesh.get('filename', '')
        if not filename.startswith('package://'):
            continue
        checked_meshes += 1
        package_path = filename[len('package://'):].split('/', 1)
        if len(package_path) != 2:
            missing_meshes.append(filename)
            continue
        package, relative = package_path
        try:
            resolved = Path(get_package_share_directory(package)) / relative
        except Exception:
            missing_meshes.append(filename)
            continue
        if not resolved.exists():
            missing_meshes.append(filename)
    report['mesh_resources'] = result(
        'PASS' if not missing_meshes else 'FAIL',
        f'{checked_meshes} checked' if not missing_meshes else f'missing={missing_meshes}')
    return report, xml, movable, set(link_names)


def runtime_checks(expected_xml, expected_joints, expected_links, timeout_sec,
                   expect_joint_states=True):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=None)
    node = Node('robot_description_diagnostic')
    transient = QoSProfile(
        depth=1, history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    volatile = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    state = {'description': None, 'joints': [], 'tf': set(), 'tf_static': set()}
    node.create_subscription(String, '/robot_description',
                             lambda msg: state.update(description=msg.data), transient)
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: state['joints'].append(msg), volatile)

    def collect_tf(key):
        def callback(msg):
            state[key].update((item.header.frame_id, item.child_frame_id)
                              for item in msg.transforms)
        return callback

    node.create_subscription(TFMessage, '/tf', collect_tf('tf'), volatile)
    node.create_subscription(TFMessage, '/tf_static', collect_tf('tf_static'), transient)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    report = {}
    received = state['description']
    hashes_match = received is not None and hashlib.sha256(received.encode()).digest() == hashlib.sha256(expected_xml.encode()).digest()
    report['description_payload'] = result(
        'PASS' if hashes_match else 'FAIL',
        'received and matches local expansion' if hashes_match else 'missing or differs from local expansion')
    messages = state['joints']
    names = set(messages[-1].name) if messages else set()
    finite = bool(messages) and all(math.isfinite(value) for value in messages[-1].position)
    lengths_ok = bool(messages) and len(messages[-1].name) == len(messages[-1].position)
    joints_ok = expected_joints <= names and finite and lengths_ok
    frequency = len(messages) / timeout_sec
    if expect_joint_states:
        report['joint_states'] = result(
            'PASS' if joints_ok and frequency >= 5.0 else 'FAIL',
            f'{len(messages)} messages ({frequency:.1f} Hz), names={sorted(names)}')
    else:
        report['joint_states'] = result(
            'PASS' if not messages else 'FAIL',
            f'{len(messages)} messages; expected none')
    dynamic_children = {child for _, child in state['tf']}
    static_children = {child for _, child in state['tf_static']}
    dynamic_ok = bool(dynamic_children) if expect_joint_states else not dynamic_children
    report['dynamic_tf'] = result(
        'PASS' if dynamic_ok else 'FAIL',
        f'{len(dynamic_children)} child frames; expected '
        f'{"dynamic transforms" if expect_joint_states else "none"}')
    arm_base = ('rx200/base_link' if 'rx200/base_link' in expected_links
                else 'arm_base_link')
    required_static = {
        'base_link', 'baseup_link', 'lidar_link', 'camera_link', arm_base}
    missing_static = sorted(required_static - static_children)
    report['static_tf'] = result(
        'PASS' if not missing_static else 'FAIL',
        f'{len(static_children)} child frames; missing={missing_static}')
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=Path)
    parser.add_argument('--runtime', action='store_true')
    parser.add_argument(
        '--expect-joint-states', action=argparse.BooleanOptionalAction,
        default=True,
        help='expect joint-state and dynamic-TF traffic (disable when JSP is off)')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    share = Path(get_package_share_directory('robot_description'))
    model = args.model or share / 'urdf/sim/my_car.urdf.xacro'
    report = {
        'package_discovery': result('PASS', str(share)),
        'model': result('PASS' if model.exists() else 'FAIL', str(model)),
    }
    static, xml, movable, links = static_checks(model)
    report.update(static)
    if args.runtime and xml:
        report.update(runtime_checks(
            xml, movable, links, args.timeout, args.expect_joint_states))
    elif not args.runtime:
        report['runtime_pipeline'] = result('NOT TESTED', 'pass --runtime with the launch running')
    overall = 'PASS' if all(item['status'] in {'PASS', 'NOT TESTED'} for item in report.values()) else 'FAIL'
    if args.json:
        print(json.dumps({'overall': overall, 'checks': report}, indent=2))
    else:
        print('ROBOT DESCRIPTION DIAGNOSTIC REPORT')
        for name, item in report.items():
            print(f'{name:24} {item["status"]:10} {item["detail"]}')
        print(f'OVERALL RESULT           {overall}')
    return 0 if overall == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
