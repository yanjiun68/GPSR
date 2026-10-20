from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).parents[1]
MODEL = PACKAGE / 'urdf/sim/my_car_rx200.urdf.xacro'
RX200_MODEL = MODEL
COMPAT_MODEL = PACKAGE / 'urdf/sim/my_car.urdf.xacro'


def expanded_root(model=MODEL, *xacro_args):
    output = subprocess.run(
        ['xacro', str(model), *xacro_args], capture_output=True, text=True, check=True,
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
    links = {link.get('name') for link in expanded_root(MODEL, 'arm_model:=piper').findall('link')}
    assert {
        'base_footprint', 'base_link', 'baseup_link', 'lidar_link',
        'camera_link', 'arm_base_link', 'link6', 'gripper_base',
        'arm_camera_link', 'arm_camera_depth_optical_frame',
        'arm_camera_color_optical_frame',
    } <= links


def test_eye_in_hand_camera_keeps_mount_and_gazebo_configuration():
    root = expanded_root(MODEL, 'arm_model:=piper')
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    camera_joint = joints['arm_camera_joint']
    assert camera_joint.find('parent').get('link') == 'link6'
    assert camera_joint.find('child').get('link') == 'arm_camera_bottom_screw_frame'
    assert camera_joint.find('origin').get('xyz') == '-0.0375 0.0075 0.0394'

    sensor = root.find("./gazebo[@reference='arm_camera_link']/sensor")
    assert sensor is not None
    assert sensor.get('name') == 'arm_rgbd_camera'
    assert sensor.findtext('topic') == '/arm_camera/arm_camera'
    assert sensor.findtext('gz_frame_id') == 'arm_camera_color_optical_frame'


def test_default_mobile_model_uses_rx200_and_shifted_mount():
    root = expanded_root()
    links = {link.get('name') for link in root.findall('link')}
    assert 'rx200/base_link' in links
    assert 'arm_base_link' not in links

    joint = next(joint for joint in root.findall('joint')
                 if joint.get('name') == 'arm_base_joint')
    assert joint.find('child').get('link') == 'rx200/base_link'
    assert joint.find('origin').get('xyz') == '0.112 0 0.445'


def test_legacy_model_path_forwards_to_renamed_model():
    root = expanded_root(COMPAT_MODEL)
    links = {link.get('name') for link in root.findall('link')}
    assert 'rx200/base_link' in links
    assert 'arm_base_link' not in links


def test_rx200_variant_replaces_piper_and_remains_connected():
    root = expanded_root(RX200_MODEL)
    links = {link.get('name'): link for link in root.findall('link')}
    children = {joint.find('child').get('link') for joint in root.findall('joint')}
    assert set(links) - children == {'base_footprint'}
    assert 'rx200/base_link' in links
    assert 'rx200/ee_gripper_link' in links
    assert 'lift_base_link' not in links
