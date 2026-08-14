"""Launch the dry-run-capable SMACC2 receptionist task stack."""

# Import os so package-share paths can be assembled.
import os

# Import package-share lookup for config defaults.
from ament_index_python.packages import get_package_share_directory

# Import launch description and launch arguments.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

# Import conditional launch support.
from launch.conditions import IfCondition

# Import launch substitution helpers.
from launch.substitutions import LaunchConfiguration

# Import ROS 2 node launch action.
from launch_ros.actions import Node


# Build the launch description used by ros2 launch.
def generate_launch_description():
    # Resolve package shares used by default config paths.
    receptionist_share = get_package_share_directory("receptionist_nodes")
    my_robot_nav_share = get_package_share_directory("my_robot_nav")

    # Resolve common config files once.
    task2_config = os.path.join(receptionist_share, "config", "task2.yaml")
    layout_file = os.path.join(receptionist_share, "config", "receptionist_layout.yaml")
    waypoints_file = os.path.join(my_robot_nav_share, "maps", "waypoints.yaml")

    # Create launch substitutions for runtime options.
    dry_run = LaunchConfiguration("dry_run")
    target_guest_count = LaunchConfiguration("target_guest_count")
    start_support_nodes = LaunchConfiguration("start_support_nodes")
    start_named_pose_server = LaunchConfiguration("start_named_pose_server")
    start_llm_advisor = LaunchConfiguration("start_llm_advisor")
    start_lifecycle_node = LaunchConfiguration("start_lifecycle_node")
    use_camera_topic = LaunchConfiguration("use_camera_topic")
    guest_enter_topic = LaunchConfiguration("guest_enter_topic")
    startup_dio13_topic = LaunchConfiguration("startup_dio13_topic")
    piper_model_path = LaunchConfiguration("piper_model_path")
    waypoint_1 = LaunchConfiguration("waypoint_1")
    waypoint_2 = LaunchConfiguration("waypoint_2")
    waypoint_3 = LaunchConfiguration("waypoint_3")
    waypoint_4 = LaunchConfiguration("waypoint_4")
    waypoint_5 = LaunchConfiguration("waypoint_5")

    # Declare all runtime options with dry-run-safe defaults.
    declared_arguments = [
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("target_guest_count", default_value="2"),
        DeclareLaunchArgument("start_support_nodes", default_value="true"),
        DeclareLaunchArgument("start_named_pose_server", default_value="true"),
        DeclareLaunchArgument("start_llm_advisor", default_value="true"),
        DeclareLaunchArgument("start_lifecycle_node", default_value="false"),
        DeclareLaunchArgument("use_camera_topic", default_value="/camera/rx200_camera/color/image_raw"),
        DeclareLaunchArgument("guest_enter_topic", default_value="/person_selected"),
        DeclareLaunchArgument("startup_dio13_topic", default_value="/dio13_state"),
        DeclareLaunchArgument("piper_model_path", default_value=""),
        DeclareLaunchArgument("waypoint_1", default_value="ENTRANCE"),
        DeclareLaunchArgument("waypoint_2", default_value="FACE_HOST"),
        DeclareLaunchArgument("waypoint_3", default_value="FACE_CHAIR1"),
        DeclareLaunchArgument("waypoint_4", default_value="FACE_CHAIR3"),
        DeclareLaunchArgument("waypoint_5", default_value="INITIAL_POINT"),
    ]

    # Start the existing named waypoint action server when requested.
    named_pose_server = Node(
        package="my_robot_nav",
        executable="navigate_named_pose_server.py",
        name="navigate_named_pose_server",
        output="screen",
        condition=IfCondition(start_named_pose_server),
        parameters=[{
            "waypoints_file": waypoints_file,
            "action_name": "/nav/navigate_named_pose",
            "navigate_to_pose_action": "/navigate_to_pose",
        }],
    )

    # Start the bounded LLM advisor service.
    llm_advisor = Node(
        package="llm_pkg",
        executable="receptionist_advisor_node",
        name="receptionist_advisor_node",
        output="screen",
        condition=IfCondition(start_llm_advisor),
        parameters=[{
            "enable_online_llm": True,
        }],
    )

    # Start the appearance action wrapper.
    appearance_action_server = Node(
        package="receptionist_nodes",
        executable="appearance_action_server",
        name="appearance_action_server",
        output="screen",
        condition=IfCondition(start_support_nodes),
        parameters=[task2_config],
    )

    # Start the receptionist memory node.
    guest_memory_node = Node(
        package="receptionist_nodes",
        executable="guest_memory_node",
        name="guest_memory_node",
        output="screen",
        condition=IfCondition(start_support_nodes),
    )

    # Start the dialogue action server for non-dry-run profile collection.
    dialogue_action_server = Node(
        package="receptionist_nodes",
        executable="dialogue_action_server",
        name="dialogue_action_server",
        output="screen",
        condition=IfCondition(start_support_nodes),
        parameters=[task2_config, {
            "piper_model_path": piper_model_path,
            "image_topic": use_camera_topic,
        }],
    )

    # Start the speech service bridge.
    tts_bridge_node = Node(
        package="receptionist_nodes",
        executable="tts_bridge_node",
        name="tts_bridge_node",
        output="screen",
        condition=IfCondition(start_support_nodes),
        parameters=[task2_config, {
            "piper_model_path": piper_model_path,
        }],
    )

    # Start navigation helper services.
    nav_controller_node = Node(
        package="receptionist_nodes",
        executable="nav_controller_node",
        name="nav_controller_node",
        output="screen",
        condition=IfCondition(start_support_nodes),
        parameters=[task2_config, {
            "dry_run": dry_run,
        }],
    )

    # Start seated-person target resolution.
    seated_people_node = Node(
        package="receptionist_nodes",
        executable="seated_people_node",
        name="seated_people_node",
        output="screen",
        condition=IfCondition(start_support_nodes),
        parameters=[task2_config, {
            "layout_file": layout_file,
        }],
    )

    # Start empty-chair selection.
    chair_manager_node = Node(
        package="receptionist_nodes",
        executable="chair_manager_node",
        name="chair_manager_node",
        output="screen",
        condition=IfCondition(start_support_nodes),
        parameters=[task2_config, {
            "layout_file": layout_file,
        }],
    )

    # Start safe seat pointing service.
    seat_pointing_node = Node(
        package="receptionist_nodes",
        executable="seat_pointing_node",
        name="seat_pointing_node",
        output="screen",
        condition=IfCondition(start_support_nodes),
        parameters=[task2_config, {
            "dry_run": dry_run,
        }],
    )

    # Start optional lifecycle manager.
    lifecycle_node = Node(
        package="sm_receptionist",
        executable="sm_receptionist_lifecycle_node",
        name="sm_receptionist_lifecycle",
        output="screen",
        condition=IfCondition(start_lifecycle_node),
        parameters=[{
            "dry_run": dry_run,
            "target_guest_count": target_guest_count,
        }],
    )

    # Start the SMACC2 receptionist state machine.
    smacc2_node = Node(
        package="sm_receptionist",
        executable="sm_receptionist_node",
        name="sm_receptionist",
        output="screen",
        parameters=[{
            "dry_run": dry_run,
            "target_guest_count": target_guest_count,
            "guest_enter_topic": guest_enter_topic,
            "startup_dio13_topic": startup_dio13_topic,
            "waypoint_1": waypoint_1,
            "waypoint_2": waypoint_2,
            "waypoint_3": waypoint_3,
            "waypoint_4": waypoint_4,
            "waypoint_5": waypoint_5,
        }],
    )

    # Return the full launch description in dependency-friendly order.
    return LaunchDescription([
        *declared_arguments,
        named_pose_server,
        llm_advisor,
        appearance_action_server,
        guest_memory_node,
        dialogue_action_server,
        tts_bridge_node,
        nav_controller_node,
        seated_people_node,
        chair_manager_node,
        seat_pointing_node,
        lifecycle_node,
        smacc2_node,
    ])
