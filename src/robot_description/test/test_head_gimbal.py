from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).parents[1]
MODELS = (
    PACKAGE / 'urdf/sim/my_car.urdf.xacro',
    PACKAGE / 'urdf/sim/my_car_rx200.urdf.xacro',
)
BRIDGE = PACKAGE.parent / 'GPSR/gz_sim/config/robocup_bridge.yaml'


def expand(model, **arguments):
    command = ['xacro', str(model), 'arm_model:=rx200']
    command.extend(f'{key}:={value}' for key, value in arguments.items())
    result = subprocess.run(
        command, capture_output=True, text=True, check=True, timeout=30)
    return ET.fromstring(result.stdout)


def named(root, tag):
    elements = root.findall(tag)
    names = [element.get('name') for element in elements]
    assert len(names) == len(set(names)), f'duplicate {tag} names'
    return {element.get('name'): element for element in elements}


@pytest.mark.parametrize('model', MODELS)
def test_enabled_head_contract_and_single_connected_tree(model):
    root = expand(
        model, enable_head_gimbal='true', enable_aluminium_frame='true')
    links = named(root, 'link')
    joints = named(root, 'joint')
    children = {joint.find('child').get('link') for joint in joints.values()}

    assert set(links) - children == {'base_footprint'}
    assert list(links).count('camera_link') == 1
    assert {
        'camera_color_optical_frame', 'camera_depth_optical_frame',
        'camera_accel_frame', 'camera_gyro_frame',
    } <= set(links)

    expected = {
        'head_yaw_joint': (
            'alu3030_blue_camera_top', 'head_yaw_link', '0 0 0.036', '0 0 1'),
        'head_pitch_joint': (
            'head_yaw_link', 'head_pitch_link', '0 0 0.025', '0 1 0'),
    }
    for joint_name, (parent, child, xyz, axis) in expected.items():
        joint = joints[joint_name]
        assert joint.get('type') == 'revolute'
        assert joint.find('parent').get('link') == parent
        assert joint.find('child').get('link') == child
        assert joint.find('origin').get('xyz') == xyz
        assert joint.find('axis').get('xyz') == axis

    # Both fixed RealSense mount transforms have zero rotation. Their summed
    # translation places camera_link exactly 0.145 m above head_pitch_link.
    camera_mount = joints['camera_joint']
    body_mount = next(
        joint for joint in joints.values()
        if joint.find('parent').get('link') == 'camera_bottom_screw_frame'
        and joint.find('child').get('link') == 'camera_link')
    mount_xyz = [float(value) for value in camera_mount.find('origin').get('xyz').split()]
    body_xyz = [float(value) for value in body_mount.find('origin').get('xyz').split()]
    assert camera_mount.find('parent').get('link') == 'head_pitch_link'
    assert [a + b for a, b in zip(mount_xyz, body_xyz)] == pytest.approx(
        [0.0, 0.0, 0.145], abs=1e-9)

    plugins = root.findall('.//plugin')
    head_plugins = [
        plugin for plugin in plugins
        if plugin.findtext('topic') == '/head/joint_trajectory']
    assert len(head_plugins) == 1
    assert [element.text for element in head_plugins[0].findall('joint_name')] == [
        'head_yaw_joint', 'head_pitch_joint']
    assert sum(
        plugin.get('name') == 'ignition::gazebo::systems::JointStatePublisher'
        for plugin in plugins) == 1


@pytest.mark.parametrize('model', MODELS)
@pytest.mark.parametrize('head_enabled,frame_enabled', (
    ('false', 'true'),
    ('true', 'false'),
))
def test_disabled_head_uses_one_fixed_camera_fallback(
        model, head_enabled, frame_enabled):
    root = expand(
        model,
        enable_head_gimbal=head_enabled,
        enable_aluminium_frame=frame_enabled,
    )
    links = named(root, 'link')
    joints = named(root, 'joint')
    assert list(links).count('camera_link') == 1
    assert 'head_yaw_link' not in links
    assert 'head_pitch_link' not in links
    assert 'head_yaw_joint' not in joints
    assert 'head_pitch_joint' not in joints
    assert joints['camera_joint'].find('parent').get('link') == 'baseup_link'
    assert not any(
        plugin.findtext('topic') == '/head/joint_trajectory'
        for plugin in root.findall('.//plugin'))


def test_head_command_bridge_is_unique_and_joint_feedback_is_preserved():
    entries = yaml.safe_load(BRIDGE.read_text())
    head = [
        entry for entry in entries
        if entry['ros_topic_name'] == '/head/joint_trajectory']
    joint_states = [
        entry for entry in entries
        if entry['ros_topic_name'] == '/joint_states']
    assert len(head) == 1
    assert head[0]['direction'] == 'ROS_TO_GZ'
    assert head[0]['ros_type_name'] == 'trajectory_msgs/msg/JointTrajectory'
    assert head[0]['gz_type_name'] == 'ignition.msgs.JointTrajectory'
    assert len(joint_states) == 1
    assert joint_states[0]['direction'] == 'GZ_TO_ROS'
