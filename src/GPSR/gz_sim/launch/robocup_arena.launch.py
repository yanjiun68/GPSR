"""Launch the GPSR arena and spawn the robot relative to its entrance.

The world and entrance coordinates must refer to the same arena layout.
door_yaw is the direction from outside to inside. The 0.4 m default distance
is measured from the doorway centre plane to the robot's base_footprint.
Optional spawn_x/spawn_y/spawn_yaw arguments select fixed world coordinates.
"""

import math
import json
import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
    LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_arena(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world_path = os.path.abspath(os.path.expanduser(arg('world')))
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    if world is None or not world.get('name'):
        raise ValueError('world must be an SDF containing a named <world>')

    fixed_spawn_values = [arg(name) for name in ('spawn_x', 'spawn_y', 'spawn_yaw')]
    if any(fixed_spawn_values) and not all(fixed_spawn_values):
        raise ValueError('spawn_x, spawn_y, and spawn_yaw must be provided together')
    start_z = float(arg('spawn_z'))
    if not math.isfinite(start_z):
        raise ValueError('spawn_z must be finite')

    if all(fixed_spawn_values):
        start_x, start_y, start_yaw = (float(value) for value in fixed_spawn_values)
        if not all(math.isfinite(v) for v in (start_x, start_y, start_yaw)):
            raise ValueError('spawn_x, spawn_y, and spawn_yaw must be finite')
        spawn_description = 'fixed world coordinates'
        distance = None
        side = None
    else:
        entrance = world.find("frame[@name='arena_entrance']/pose")
        entrance_pose = [float(v) for v in entrance.text.split()] if entrance is not None else None
        door_values = [arg(name) for name in ('door_x', 'door_y', 'door_yaw')]
        if any(not value for value in door_values) and entrance_pose is None:
            raise ValueError('World needs an arena_entrance frame, or explicit door_x, door_y, door_yaw')
        door_x, door_y, door_yaw = (
            float(value) if value else entrance_pose[index]
            for value, index in zip(door_values, (0, 1, 5))
        )
        distance = float(arg('start_distance'))
        if not all(math.isfinite(v) for v in (door_x, door_y, door_yaw, distance)):
            raise ValueError('Door coordinates and start_distance must be finite')
        if distance <= 0:
            raise ValueError('start_distance must be positive')
        side = arg('start_side')
        if side not in ('outside', 'inside'):
            raise ValueError('start_side must be outside or inside')
        sign = -1 if side == 'outside' else 1
        start_x = door_x + sign * distance * math.cos(door_yaw)
        start_y = door_y + sign * distance * math.sin(door_yaw)
        start_yaw = door_yaw
        spawn_description = f'{distance:.3f} m {side} the door plane'

    sim_share = get_package_share_directory('gz_sim')
    mesh_share = get_package_share_directory('robot_description')
    agx_arm_share = get_package_share_directory('agx_arm_description')
    head_enabled = arg('enable_head_gimbal').lower()
    frame_enabled = arg('enable_aluminium_frame').lower()
    if head_enabled not in ('true', 'false'):
        raise ValueError('enable_head_gimbal must be true or false')
    if frame_enabled not in ('true', 'false'):
        raise ValueError('enable_aluminium_frame must be true or false')
    robot_xml = xacro.process_file(
        os.path.join(mesh_share, 'urdf', 'sim', 'my_car_rx200.urdf.xacro'),
        mappings={
            'arm_model': 'rx200',
            'enable_head_gimbal': head_enabled,
            'enable_aluminium_frame': frame_enabled,
        },
    ).toxml()
    camera_enabled = arg('enable_camera').lower()
    if camera_enabled not in ('true', 'false'):
        raise ValueError('enable_camera must be true or false')
    if camera_enabled == 'false':
        robot = ET.fromstring(robot_xml)
        for gazebo_tag in robot.findall('gazebo'):
            for sensor in gazebo_tag.findall("sensor[@type='rgbd_camera']"):
                gazebo_tag.remove(sensor)
        robot_xml = ET.tostring(robot, encoding='unicode')
    camera_actions = []
    if camera_enabled == 'true':
        # Fortress publishes RGB-D point coordinates in Gazebo's camera-body
        # convention (+X forward), even though gz_frame_id labels every RGB-D
        # output with the ROS optical frame (+Z forward). Keep that native
        # cloud on /camera/points_gz_raw and reconstruct the public cloud from
        # the aligned depth, color image, and CameraInfo. depth_image_proc then
        # publishes standards-compliant optical XYZ without changing camera TF.
        camera_actions.append(Node(
            package='depth_image_proc', executable='point_cloud_xyzrgb_node',
            name='sim_rgbd_point_cloud', output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('rgb/image_rect_color', '/camera/color/image_raw'),
                ('depth_registered/image_rect', '/camera/depth/image_raw'),
                ('points', '/camera/points'),
            ],
        ))
    with open(os.path.join(sim_share, 'config', 'robocup2026_arena.json')) as stream:
        washer_params = json.load(stream)['washer']
    resource_paths = os.pathsep.join([
        os.path.dirname(world_path),
        os.path.join(os.path.dirname(world_path), 'models'),
        os.path.join(sim_share, 'world', 'models'),
        os.path.dirname(mesh_share),
        os.path.dirname(agx_arm_share),
    ])

    headless = arg('headless').lower()
    if headless not in ('true', 'false'):
        raise ValueError('headless must be true or false')
    command = ['ign', 'gazebo', '-r', '-v', '3']
    if headless == 'true':
        command += ['-s', '--headless-rendering']
    command += [world_path]
    gazebo = ExecuteProcess(cmd=command, output='screen')
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-world', world.get('name'), '-name', 'my_car',
            '-string', robot_xml, '-allow_renaming', 'false',
            '-x', str(start_x), '-y', str(start_y), '-z', str(start_z),
            '-Y', str(start_yaw),
        ],
    )

    washer_actions = []
    if world.find("model[@name='washing_machine']/joint[@name='door_hinge']") is not None:
        washer_actions = [
            Node(package='gz_sim', executable='washer_controller.py',
                 parameters=[{'use_sim_time': True, **washer_params}], output='screen'),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='washer_bridge',
                 arguments=[
                     '/washing_machine/door_target@std_msgs/msg/Float64]ignition.msgs.Double',
                     '/washing_machine/drum_speed@std_msgs/msg/Float64]ignition.msgs.Double',
                     '/washing_machine/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                     *[f'/washing_machine/buttons/{name}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'
                       for name in ('power','start','door')],
                 ], output='screen'),
        ]
        if headless == 'false' and arg('washer_panel').lower() == 'true':
            washer_actions.append(Node(package='gz_sim', executable='washer_panel.py', output='screen'))

    return [
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_paths),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_paths),
        LogInfo(msg=(
            f'Arena: {world_path}; robot base at ({start_x:.3f}, {start_y:.3f}), '
            f'yaw={start_yaw:.4f}; {spawn_description}; z={start_z:.3f} m.'
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=gazebo, on_exit=[Shutdown(reason='Gazebo exited')]
        )),
        gazebo,
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'use_sim_time': True, 'robot_description': robot_xml}],
            output='screen',
        ),
        spawn,
        *washer_actions,
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='arena_bridge',
            parameters=[{'use_sim_time': True, 'config_file': os.path.join(
                sim_share, 'config', 'robocup_bridge.yaml')}], output='screen',
        ),
        *camera_actions,
    ]


def generate_launch_description():
    default_world = os.path.join(get_package_share_directory('gz_sim'),
                                 'world', 'robocup2026_incheon.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Path to the arena SDF world'),
        DeclareLaunchArgument('door_x', default_value='', description='Override entrance world X (m)'),
        DeclareLaunchArgument('door_y', default_value='', description='Override entrance world Y (m)'),
        DeclareLaunchArgument('door_yaw', default_value='', description='Override heading INTO arena (radians)'),
        DeclareLaunchArgument('start_distance', default_value='0.4',
                              description='Door plane to base_footprint distance (m)'),
        DeclareLaunchArgument('start_side', default_value='outside',
                              description='outside or inside; robot faces inward'),
        DeclareLaunchArgument('spawn_x', default_value='1.04',
                              description='Optional fixed world X coordinate (m)'),
        DeclareLaunchArgument('spawn_y', default_value='2.98',
                              description='Optional fixed world Y coordinate (m)'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                              description='Optional fixed world yaw (radians)'),
        DeclareLaunchArgument('spawn_z', default_value='0.15',
                              description='Robot spawn height (m)'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='Enable RGB-D rendering; disable for faster SLAM'),
        DeclareLaunchArgument('enable_head_gimbal', default_value='true',
                              description='Enable the two-axis mast camera head'),
        DeclareLaunchArgument('enable_aluminium_frame', default_value='true',
                              description='Enable the aluminium superstructure'),
        DeclareLaunchArgument('washer_panel', default_value='true',
                              description='Show washer controls when GUI is enabled'),
        OpaqueFunction(function=_launch_arena),
    ])
