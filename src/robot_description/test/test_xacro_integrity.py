from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).parents[1]
MODEL = PACKAGE / 'urdf/sim/my_car.urdf.xacro'
RX200_MODEL = PACKAGE / 'urdf/sim/my_car_rx200.urdf.xacro'


def expanded_root(model=MODEL):
    output = subprocess.run(
        ['xacro', str(model)], capture_output=True, text=True, check=True,
        timeout=20)
    return ET.fromstring(output.stdout)


def test_model_is_a_single_tree_without_root_inertia():
    root = expanded_root()
    links = {link.get('name'): link for link in root.findall('link')}
    children = {joint.find('child').get('link') for joint in root.findall('joint')}
    roots = set(links) - children
    assert roots == {'base_footprint'}
    assert links['base_footprint'].find('inertial') is None


def test_complete_robot_frames_are_present():
    links = {link.get('name') for link in expanded_root().findall('link')}
    assert {
        'base_footprint', 'base_link', 'baseup_link', 'lidar_link',
        'camera_link', 'arm_base_link', 'link6', 'gripper_base',
    } <= links


def test_rx200_variant_replaces_piper_and_remains_connected():
    root = expanded_root(RX200_MODEL)
    links = {link.get('name'): link for link in root.findall('link')}
    children = {joint.find('child').get('link') for joint in root.findall('joint')}
    assert set(links) - children == {'base_footprint'}
    assert 'rx200/base_link' in links
    assert 'rx200/ee_gripper_link' in links
    assert 'arm_base_link' not in links
